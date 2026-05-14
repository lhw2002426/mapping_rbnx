#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx native (no-docker) launcher.
#
# Equivalent of docker/entrypoint.sh, but executed directly on the host
# ROS2 install. Picked by scripts/start.sh when the deploy manifest's
# `service.mapping.config.platform` is in the native whitelist
# (currently: jetson_orin).
#
# Same four-stage flow as the container path, minus the parts that
# only make sense inside a container:
#
#   1. atlas_bridge        — Python: registers cap with atlas, parses
#                            CMD_INIT cfg, writes /tmp/<algo>_resolved.yaml.
#   2. wait for /tmp/mapping_algo + /tmp/<algo>_resolved.yaml
#   3. start_engine.sh     — ros2 launch <algo>'s launch file with the
#                            atlas-resolved sensor topics.
#
# Removed vs container path:
#   - rmw_zenoh router + zenoh_bridge_dds: native already shares the
#     host DDS bus with sensor processes, no bridging needed.
#   - X11 export: rtabmap_viz inherits host DISPLAY directly.
#   - SHM-disable FastRTPS profile: with all participants in the
#     same /dev/shm namespace (everyone is a host process now), SHM
#     transport works fine; UDP-only is no longer required. We
#     respect any user-set FASTRTPS_DEFAULT_PROFILES_FILE / RMW
#     in the parent shell instead of forcing a profile here.
#
# Pre-conditions (the operator is expected to have these on the host;
# we check + fail loud rather than silently degrading):
#   - ROS2 Humble sourced (or available at /opt/ros/humble)
#   - rtabmap_ros available from either:
#       * self-built old_cap/rtabmap_new/install/setup.bash overlay
#       * or system apt package ros-humble-rtabmap-ros
#   - python3 + grpcio + protobuf + pyyaml on PATH
#   - generated rbnx proto stubs available under rbnx-build/codegen
#
# Trap discipline mirrors the container's: SIGTERM tears down both
# atlas_bridge and the ros2 launch tree.

set -eo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# ── Pre-flight checks ─────────────────────────────────────────────────
# Source ROS2 if it isn't already (start.sh is typically invoked from
# rbnx boot's spawn body which doesn't inherit user's ros setup).
if [[ -z "${ROS_DISTRO:-}" ]]; then
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
    else
        echo "[start-native] ERR: ROS2 not sourced and /opt/ros/humble/setup.bash missing." >&2
        echo "[start-native]      For platform=jetson_orin we expect a host ROS2 Humble install." >&2
        echo "[start-native]      Either source ros2 in the launching shell, or set" >&2
        echo "[start-native]      ROBONIX_MAPPING_FORCE=docker to fall back to the containerised path." >&2
        exit 2
    fi
fi

# rtabmap_ros may be self-built, like old_cap/rtabmap_new/rbnx/start.sh:
#   source install/setup.bash
#   ros2 launch rtabmap_examples ...
# Source that overlay before checking packages so native mode doesn't depend
# on the apt-provided ros-humble-rtabmap-ros.
source_rtabmap_overlay() {
    local -a candidates=()
    local repo_root
    repo_root="$(cd "${PKG}/../.." && pwd)"

    if [[ -n "${MAPPING_RTABMAP_SETUP:-}" ]]; then
        candidates+=("${MAPPING_RTABMAP_SETUP}")
    fi
    if [[ -n "${MAPPING_RTABMAP_INSTALL:-}" ]]; then
        candidates+=("${MAPPING_RTABMAP_INSTALL%/}/setup.bash")
    fi
    if [[ -n "${MAPPING_RTABMAP_WS:-}" ]]; then
        candidates+=("${MAPPING_RTABMAP_WS%/}/install/setup.bash")
    fi

    candidates+=("${PKG}/third_party/rtabmap_new/install/setup.bash")
    if [[ "${MAPPING_RTABMAP_USE_OLD_CAP:-}" == "1" ]]; then
        candidates+=(
            "${repo_root}/old_cap/rtabmap_new/install/setup.bash"
            "/home/syswonder/wheatfox/old_cap/rtabmap_new/install/setup.bash"
        )
    fi

    local setup
    for setup in "${candidates[@]}"; do
        if [[ -f "$setup" ]]; then
            echo "[start-native] sourcing self-built rtabmap overlay: $setup"
            # shellcheck disable=SC1090
            source "$setup"
            export MAPPING_RTABMAP_SETUP_RESOLVED="$setup"
            return 0
        fi
    done
    echo "[start-native] WARN: self-built rtabmap overlay not found; falling back to system ROS packages" >&2
    return 1
}
source_rtabmap_overlay || true

# rtabmap packages must be available after sourcing either the custom overlay
# or the system ROS installation. The native path calls ros2 launch directly.
if ! ros2 pkg list 2>/dev/null | grep -q '^rtabmap_slam$'; then
    echo "[start-native] ERR: rtabmap_slam not found after sourcing ROS2/rtabmap overlays." >&2
    echo "[start-native]      Build/source old_cap/rtabmap_new, or set one of:" >&2
    echo "[start-native]        MAPPING_RTABMAP_WS=/path/to/rtabmap_new" >&2
    echo "[start-native]        MAPPING_RTABMAP_INSTALL=/path/to/rtabmap_new/install" >&2
    echo "[start-native]        MAPPING_RTABMAP_SETUP=/path/to/install/setup.bash" >&2
    echo "[start-native]      Fallback: sudo apt install ros-humble-rtabmap-ros" >&2
    exit 2
fi

# ── PYTHONPATH for atlas_bridge ───────────────────────────────────────
# Container path used /mapping/src + /mapping/rbnx-build/codegen/...
# Host paths line up 1:1 — codegen output is rooted under the package.
CODEGEN="${PKG}/rbnx-build/codegen"
if [[ ! -d "${CODEGEN}/proto_gen" ]]; then
    echo "[start-native] ERR: ${CODEGEN}/proto_gen missing — run \`bash scripts/build.sh\` first" >&2
    echo "[start-native]      (it runs \`rbnx codegen\` to generate atlas_pb2 etc.)" >&2
    exit 2
fi

export PYTHONPATH="${PKG}/src:${CODEGEN}/proto_gen:${CODEGEN}/robonix_mcp_types:${PYTHONPATH:-}"

# robonix_api: the container mounts it at /robonix-api; on host we
# locate it via `rbnx path` (same call the docker start.sh uses).
if command -v rbnx >/dev/null 2>&1; then
    if ROBONIX_API_DIR="$(rbnx path robonix-api 2>/dev/null)" && [[ -d "$ROBONIX_API_DIR" ]]; then
        export PYTHONPATH="${ROBONIX_API_DIR}:${PYTHONPATH}"
    fi
fi

mkdir -p "${PKG}/rbnx-build/data"

# ── Env defaults (mirror the docker -e block) ─────────────────────────
export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
export ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-mapping}"
export ROBONIX_PKG_HOST_DIR="${PKG}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export MAPPING_GRPC_PORT="${MAPPING_GRPC_PORT:-50120}"
# Default viz OFF on jetson — the Orin is usually headless and viz
# would just spam Qt-no-display warnings. Operator can flip with
# MAPPING_ENABLE_VIZ=true on the launching shell.
export MAPPING_ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-false}"

# RMW: don't override. On the Jetson host, sensor packages
# (mid360_lidar_rbnx, realsense_camera_rbnx, ranger_chassis_rbnx) all
# default to FastRTPS, and so does ROS2 Humble out of the box. Forcing
# rmw_fastrtps_cpp here is a no-op in the common case but would surprise
# a user who deliberately set rmw_zenoh_cpp on the launching shell.

# Clean any stale resolved-config sentinels from a previous run so the
# wait loops below don't pick up garbage.
rm -f /tmp/mapping_algo \
      /tmp/rtabmap_resolved.yaml \
      /tmp/dlio_resolved.yaml \
      /tmp/fastlio2_resolved.yaml

# ── Process supervision ────────────────────────────────────────────────
ATLAS_PID=
ENGINE_PID=

cleanup() {
    [[ -n "$ENGINE_PID" ]] && kill -TERM "$ENGINE_PID" 2>/dev/null || true
    [[ -n "$ATLAS_PID"  ]] && kill -TERM "$ATLAS_PID"  2>/dev/null || true
    # Best-effort kill of orphaned ros2 launch tree (rtabmap, icp_odom,
    # rtabmap_viz, tf_to_pose). Mirrors the docker `--rm` cleanup.
    pkill -TERM -P $$ 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ── 1. atlas_bridge ────────────────────────────────────────────────────
echo "[start-native] launching atlas_bridge (atlas=${ROBONIX_ATLAS}, cap=${ROBONIX_CAPABILITY_ID})"
python3 -m mapping_rbnx.atlas_bridge 2>&1 | sed 's/^/[bridge] /' &
ATLAS_PID=$!

# ── 2. Wait for CMD_INIT to land ──────────────────────────────────────
# Same gating signals as the container: atlas_bridge writes
# /tmp/mapping_algo when cfg is parsed, /tmp/<algo>_resolved.yaml when
# atlas-discovered topics are available. Without these, start_engine.sh
# wouldn't know which algo to launch nor which topics to remap.
for _ in $(seq 1 60); do
    [[ -f /tmp/mapping_algo ]] && break
    sleep 0.5
done
ALGO="$(cat /tmp/mapping_algo 2>/dev/null || echo rtabmap)"
export MAPPING_ALGO="$ALGO"
RESOLVED="/tmp/${ALGO}_resolved.yaml"
for _ in $(seq 1 60); do
    [[ -f "$RESOLVED" ]] && break
    sleep 0.5
done

if [[ ! -f "$RESOLVED" ]]; then
    echo "[start-native] ERR: ${RESOLVED} never appeared (atlas_bridge stuck?)" >&2
    exit 3
fi

# ── 3. SLAM engine ─────────────────────────────────────────────────────
echo "[start-native] handing off to start_engine.sh (algo=${ALGO})"
# start_engine.sh hardcoded /mapping/launch/... in its `ros2 launch`
# call, so make /mapping resolve to this package on host. A symlink
# avoids forking the script just to swap one path.
if [[ ! -e /mapping ]]; then
    if ln -s "${PKG}" /mapping 2>/dev/null; then
        echo "[start-native] symlinked /mapping -> ${PKG}"
    elif [[ "$EUID" -eq 0 ]] || command -v sudo >/dev/null 2>&1 \
            && sudo -n ln -s "${PKG}" /mapping 2>/dev/null; then
        echo "[start-native] symlinked /mapping -> ${PKG} (via sudo)"
    else
        # Fallback: rewrite the resolved launch path via env so we don't
        # need /mapping at all. start_engine.sh respects MAPPING_PKG_ROOT
        # when set (added together with this script).
        export MAPPING_PKG_ROOT="${PKG}"
        echo "[start-native] no /mapping symlink possible; using MAPPING_PKG_ROOT=${PKG}"
    fi
else
    # Already exists; trust it (could be a leftover symlink from prior
    # run, or a real /mapping mount on dev machines).
    :
fi

bash "${PKG}/scripts/start_engine.sh" 2>&1 | sed 's/^/[engine] /' &
ENGINE_PID=$!

wait "$ENGINE_PID"
