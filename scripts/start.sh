#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx start phase.
#
# Two execution shapes; the choice is made up front from operator-set
# environment variables (see "How the choice is reached" below):
#
#   1. native  (jetson_orin)
#       Run atlas_bridge + ros2 launch directly on the host. Skips
#       `docker run` entirely. Used when the host already has a
#       compatible ROS2 Humble + ros-humble-rtabmap-ros toolchain
#       (the standard Jetson Orin image fits this exactly), so the
#       container only adds startup latency + an extra DDS hop with
#       no isolation upside.
#
#   2. docker  (everything else, default)
#       Original behaviour: `docker run --network host --ipc=host`
#       against the `robonix-mapping` image. This stays the safe
#       fallback for x86 desktop / isaac sim hosts where the local
#       ROS2 toolchain is not guaranteed to match the cap's needs.
#
# How the choice is reached:
#
#   The cap's runtime config (algo, sensors, platform, ...) is delivered
#   to the cap process exclusively through `Driver(CMD_INIT, config_json)`
#   over gRPC — that's the v0.1 invariant `rbnx start` / `rbnx boot`
#   honour, and `robonix-cli` does NOT materialize the config to a file
#   nor inject it into the start body's environment. So `start.sh`
#   can't read `platform` out of the cfg before the cap process starts.
#
#   Consequence: the docker-vs-native decision has to be made from
#   environment variables the operator sets in the shell that invokes
#   `rbnx boot` / `rbnx start`. There is no "automatic from the deploy
#   manifest" path here, by design of the cli (not by choice of this
#   script). If you want jetson_orin to skip docker, set
#   `ROBONIX_MAPPING_PLATFORM=jetson_orin` in the parent shell.
#
# Selection (in priority order):
#   ROBONIX_MAPPING_FORCE=native|docker     # explicit hard pin
#   ROBONIX_MAPPING_PLATFORM=<platform>     # match against NATIVE_PLATFORMS
#   default → docker                        # safe fallback
#
# Trap discipline: SIGTERM tears down whichever child is running
# (docker container OR native engine) so SLAM doesn't outlive the
# deploy.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

CT="${ROBONIX_MAPPING_CONTAINER:-robonix_mapping}"
IMG="${ROBONIX_MAPPING_IMAGE:-robonix-mapping}"

# ── Whitelist: which platforms are "native-capable" by default ─────────
# A platform is native-capable when its standard image ships ROS2 Humble
# + ros-humble-rtabmap-ros and the operator runs `rbnx boot` from a
# shell that has them sourced. Adding a new entry is a per-platform
# decision — don't blanket-enable, that defeats the docker fallback.
NATIVE_PLATFORMS=("jetson_orin")

# ── Discover platform (env-only — see header for why) ─────────────────
discover_platform() {
    echo "${ROBONIX_MAPPING_PLATFORM:-}"
}

is_native_platform() {
    local p="$1"
    for w in "${NATIVE_PLATFORMS[@]}"; do
        [[ "$p" == "$w" ]] && return 0
    done
    return 1
}

# ── Resolve mode ───────────────────────────────────────────────────────
MODE=""
case "${ROBONIX_MAPPING_FORCE:-}" in
    native)  MODE=native  ;;
    docker)  MODE=docker  ;;
    "")      ;;
    *)       echo "[start] ROBONIX_MAPPING_FORCE=${ROBONIX_MAPPING_FORCE} not in {native,docker}" >&2; exit 2 ;;
esac

if [[ -z "$MODE" ]]; then
    PLAT="$(discover_platform)"
    if is_native_platform "$PLAT"; then
        MODE=native
    else
        MODE=docker
    fi
    echo "[start] platform=${PLAT:-<unset>} → mode=${MODE}"
fi

# ── Native path: skip docker entirely ──────────────────────────────────
if [[ "$MODE" == "native" ]]; then
    exec bash "$PKG/scripts/start_native.sh"
fi

# ── Docker path (default) ──────────────────────────────────────────────
cleanup() {
    docker stop "$CT" >/dev/null 2>&1 || true
    kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Drop a stopped container from a previous run.
docker rm -f "$CT" >/dev/null 2>&1 || true

mkdir -p rbnx-build/data

declare -a EXTRA_MOUNTS=()
# Per v0.1 layering: the cap receives its full config exclusively
# through Driver(CMD_INIT, config_json) over gRPC. No config file is
# mounted into the container.

# X11 forwarding for rtabmap_viz inside the mapping container. We
# auto-detect DISPLAY when it's not in the env (the user ran
# `rbnx boot` from a fresh shell without exporting): probe the
# standard local Xorg slots, accept the first that responds. If
# none does, skip X11 wiring and rtabmap_viz won't render — the
# launch file's `enable_viz` flag still spawns it but Qt prints
# the "could not connect to display" warning we've seen before.
if [[ -z "${DISPLAY:-}" ]]; then
    if command -v xset &>/dev/null; then
        for d in :0 :1 :10; do
            if DISPLAY="$d" xset q &>/dev/null; then
                export DISPLAY="$d"
                break
            fi
        done
    fi
fi

declare -a X11_ARGS=()
if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
    xhost +local:docker >/dev/null 2>&1 || true
    X11_ARGS=(
        -e DISPLAY="$DISPLAY"
        -e QT_X11_NO_MITSHM=1
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    )
fi

exec docker run --rm \
    --name "$CT" \
    --network host \
    --ipc=host \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-mapping}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e MAPPING_GRPC_PORT="${MAPPING_GRPC_PORT:-50120}" \
    -e MAPPING_ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-true}" \
    "${X11_ARGS[@]}" \
    -v "$(pwd)":/mapping \
    -v "$(rbnx path robonix-api)":/robonix-api:ro \
    "${EXTRA_MOUNTS[@]}" \
    "$IMG"
