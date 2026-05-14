#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# mapping_rbnx build phase.
#
# Build modes mirror scripts/start.sh:
#   native — run rbnx codegen, update rtabmap_new submodules, apply
#            my_rtabmap_ros.patch, then ensure the self-built rtabmap_ros
#            overlay (old_cap/rtabmap_new/install/setup.bash) is available.
#   docker — run rbnx codegen then docker build robonix-mapping.
#
# Selection priority:
#   RBNX_BUILD_MODE=native|docker
#   ROBONIX_MAPPING_FORCE=native|docker
#   RBNX_BUILD_VARIANT=native          # compatibility alias
#   ROBONIX_MAPPING_PLATFORM=jetson_orin → native
#   default → docker
#
# RBNX_BUILD_CLEAN=1 nukes rbnx-build/ and rebuilds without docker cache.
# Docker variants: RBNX_BUILD_VARIANT=light|fastlio2_full.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# ── Resolve build mode ───────────────────────────────────────────────────────
NATIVE_PLATFORMS=("jetson_orin")

is_native_platform() {
    local p="$1"
    local w
    for w in "${NATIVE_PLATFORMS[@]}"; do
        [[ "$p" == "$w" ]] && return 0
    done
    return 1
}

MODE="${RBNX_BUILD_MODE:-}"
if [[ -z "$MODE" ]]; then
    case "${ROBONIX_MAPPING_FORCE:-}" in
        native) MODE=native ;;
        docker) MODE=docker ;;
        "") ;;
        *) echo "[build] ROBONIX_MAPPING_FORCE=${ROBONIX_MAPPING_FORCE} not in {native,docker}" >&2; exit 2 ;;
    esac
fi
if [[ -z "$MODE" && "${RBNX_BUILD_VARIANT:-}" == "native" ]]; then
    MODE=native
fi
if [[ -z "$MODE" ]]; then
    if is_native_platform "${ROBONIX_MAPPING_PLATFORM:-}"; then
        MODE=native
    else
        MODE=docker
    fi
fi
case "$MODE" in
    native|docker) ;;
    *) echo "[build] RBNX_BUILD_MODE=$MODE not in {native,docker}" >&2; exit 2 ;;
esac

echo "[build] mode=${MODE} platform=${ROBONIX_MAPPING_PLATFORM:-<unset>}"
if [[ "$MODE" == "native" ]]; then
    exec bash "$PKG/scripts/build_native.sh"
fi

BUILD="rbnx-build"
CLEAN="${RBNX_BUILD_CLEAN:-}"
VARIANT="${RBNX_BUILD_VARIANT:-light}"
IMG="${ROBONIX_MAPPING_IMAGE:-robonix-mapping}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"

# ── 1. Codegen (proto stubs for atlas + IDL types) ──────────────────────────
if command -v rbnx >/dev/null 2>&1; then
    FLAGS=()
    [[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
    echo "[build] rbnx codegen ${FLAGS[*]}"
    rbnx codegen -p "$PKG" "${FLAGS[@]}"
else
    echo "[build] WARNING: rbnx not in PATH — skipping proto codegen"
    echo "[build]   install robonix-cli + run \`rbnx setup\` once from the robonix source root"
fi

# ── 2. Docker image ─────────────────────────────────────────────────────────
if ! command -v docker >/dev/null 2>&1; then
    echo "[build] error: docker not found on PATH" >&2
    exit 1
fi

DOCKER_BUILD_FLAGS=(--network=host)
[[ "$CLEAN" == "1" ]] && DOCKER_BUILD_FLAGS+=(--no-cache)

case "$VARIANT" in
    light|native)  DF=docker/Dockerfile ;;
    fastlio2_full) DF=docker/Dockerfile.fastlio2_full ;;
    *) echo "[build] unknown RBNX_BUILD_VARIANT: $VARIANT (light|fastlio2_full|native)" >&2; exit 2 ;;
esac

# `docker build` is idempotent so this is a soft optimisation.
if [[ "$CLEAN" != "1" ]] && docker image inspect "$IMG" >/dev/null 2>&1; then
    echo "[build] image $IMG present; rebuilding incrementally"
fi

echo "[build] docker build -f $DF -t $IMG (variant=$VARIANT)"
docker build "${DOCKER_BUILD_FLAGS[@]}" -f "$DF" -t "$IMG" docker/

echo "[build] done."
