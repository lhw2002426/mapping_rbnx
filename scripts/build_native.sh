#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Native build phase for mapping_rbnx.
#
# This path is for hosts that run mapping directly instead of through the
# robonix-mapping Docker image. It mirrors old_cap/rtabmap_new/rbnx/build.sh:
#   1) make rbnx codegen outputs for atlas_bridge
#   2) provision rtabmap_new/src/{rtabmap,rtabmap_ros} at the exact old_cap
#      commits (same URLs/commits as old_cap/rtabmap_new/.gitmodules)
#   3) apply src/my_rtabmap_ros.patch to src/rtabmap_ros when needed
#   4) colcon build the rtabmap_new workspace when install is absent/stale
#   5) source its install/setup.bash and verify rtabmap_slam is visible
#
# Knobs:
#   RBNX_BUILD_CLEAN=1              clean mapping_rbnx/rbnx-build before codegen
#   MAPPING_RTABMAP_WS=/path/ws     workspace containing src/ and install/
#   MAPPING_RTABMAP_INSTALL=/path   existing install dir containing setup.bash
#   MAPPING_RTABMAP_SETUP=/path     exact setup.bash to source/verify
#   MAPPING_RTABMAP_PATCH=/path     patch file; default: <ws>/src/my_rtabmap_ros.patch
#   MAPPING_RTABMAP_USE_OLD_CAP=1   prefer repo_root/old_cap/rtabmap_new over vendored workspace
#   MAPPING_RTABMAP_REBUILD=1       force colcon build in MAPPING_RTABMAP_WS
#   MAPPING_RTABMAP_SKIP_PATCH=1    do not apply/validate rtabmap_ros patch
#   MAPPING_RTABMAP_JOBS=-j8        default MAKEFLAGS for colcon
#   MAPPING_RTABMAP_COLCON_ARGS=... extra args appended to colcon build
set -euo pipefail

source_setup_bash() {
    local setup="$1"
    # colcon/ament setup scripts may reference optional variables such as
    # COLCON_TRACE. They are not nounset-safe, so source them with `set +u`.
    export COLCON_TRACE="${COLCON_TRACE:-}"
    set +u
    # shellcheck disable=SC1090
    source "$setup"
    set -u
}

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

BUILD="rbnx-build"
CLEAN="${RBNX_BUILD_CLEAN:-}"

# Keep these pinned to old_cap/rtabmap_new/.gitmodules. mapping_rbnx/.gitmodules
# intentionally remains unrelated (it only tracks mapping's own submodules).
RTABMAP_URL="https://github.com/introlab/rtabmap.git"
RTABMAP_COMMIT="f44a4fc4786bd3c859a599718f42d3dee476ba8a"
RTABMAP_ROS_URL="https://github.com/introlab/rtabmap_ros.git"
RTABMAP_ROS_COMMIT="f3cf0d20d08597d00cf527334ffe7ee10eb47f29"
DEFAULT_RTABMAP_WS="${PKG}/third_party/rtabmap_new"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build-native] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"

# ── 1. Codegen for atlas_bridge ──────────────────────────────────────────────
if command -v rbnx >/dev/null 2>&1; then
    FLAGS=()
    [[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
    echo "[build-native] rbnx codegen ${FLAGS[*]}"
    rbnx codegen -p "$PKG" "${FLAGS[@]}"
else
    echo "[build-native] WARNING: rbnx not in PATH — skipping proto codegen" >&2
    echo "[build-native]   atlas_bridge needs rbnx-build/codegen/proto_gen at runtime" >&2
fi

# ── 2. Source ROS2 base ──────────────────────────────────────────────────────
if [[ -z "${ROS_DISTRO:-}" ]]; then
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        source_setup_bash /opt/ros/humble/setup.bash
    else
        echo "[build-native] ERR: ROS2 not sourced and /opt/ros/humble/setup.bash missing" >&2
        exit 2
    fi
fi

# ── 3. Locate the self-built rtabmap workspace/install ───────────────────────
repo_root="$(cd "${PKG}/../.." && pwd)"
RTABMAP_WS=""
RTABMAP_SETUP=""

if [[ -n "${MAPPING_RTABMAP_SETUP:-}" ]]; then
    RTABMAP_SETUP="$MAPPING_RTABMAP_SETUP"
elif [[ -n "${MAPPING_RTABMAP_INSTALL:-}" ]]; then
    RTABMAP_SETUP="${MAPPING_RTABMAP_INSTALL%/}/setup.bash"
elif [[ -n "${MAPPING_RTABMAP_WS:-}" ]]; then
    RTABMAP_WS="${MAPPING_RTABMAP_WS%/}"
    RTABMAP_SETUP="${RTABMAP_WS}/install/setup.bash"
elif [[ "${MAPPING_RTABMAP_USE_OLD_CAP:-}" == "1" ]]; then
    for candidate in \
        "${repo_root}/old_cap/rtabmap_new" \
        "/home/syswonder/wheatfox/old_cap/rtabmap_new"; do
        if [[ -d "$candidate" ]]; then
            RTABMAP_WS="$candidate"
            RTABMAP_SETUP="${candidate}/install/setup.bash"
            break
        fi
    done
else
    RTABMAP_WS="$DEFAULT_RTABMAP_WS"
    RTABMAP_SETUP="${RTABMAP_WS}/install/setup.bash"
fi

if [[ -z "$RTABMAP_WS" && -n "$RTABMAP_SETUP" ]]; then
    # setup.bash usually lives at <ws>/install/setup.bash.
    RTABMAP_WS="$(cd "$(dirname "$RTABMAP_SETUP")/.." 2>/dev/null && pwd || true)"
fi

if [[ -z "$RTABMAP_SETUP" ]]; then
    echo "[build-native] ERR: no self-built rtabmap workspace/install found" >&2
    echo "[build-native]      Set one of:" >&2
    echo "[build-native]        MAPPING_RTABMAP_WS=/path/to/rtabmap_new" >&2
    echo "[build-native]        MAPPING_RTABMAP_INSTALL=/path/to/rtabmap_new/install" >&2
    echo "[build-native]        MAPPING_RTABMAP_SETUP=/path/to/install/setup.bash" >&2
    exit 2
fi

NEED_REBUILD=0
if [[ ! -f "$RTABMAP_SETUP" || "${MAPPING_RTABMAP_REBUILD:-}" == "1" ]]; then
    NEED_REBUILD=1
fi

ensure_git_checkout() {
    local path="$1"
    local url="$2"
    local commit="$3"
    local name="$4"

    if [[ ! -d "$path/.git" ]]; then
        echo "[build-native] cloning $name: $url -> $path"
        rm -rf "$path"
        git clone "$url" "$path"
        NEED_REBUILD=1
    fi

    pushd "$path" >/dev/null
    git fetch --tags origin
    local current
    current="$(git rev-parse HEAD)"
    if [[ "$current" != "$commit" ]]; then
        echo "[build-native] checkout $name $commit"
        git checkout "$commit"
        NEED_REBUILD=1
    fi
    popd >/dev/null
}

# Keep the source workspace in the same shape as old_cap/rtabmap_new/rbnx/build.sh,
# but do not rely on mapping_rbnx/.gitmodules: provision the exact same upstream
# repos and commits explicitly, then apply the same patch.
if [[ -n "$RTABMAP_WS" ]]; then
    echo "[build-native] preparing self-built rtabmap workspace: $RTABMAP_WS"
    mkdir -p "$RTABMAP_WS/src"
    cd "$RTABMAP_WS"

    if [[ -d .git ]]; then
        echo "[build-native] git submodule update --init --recursive"
        git submodule update --init --recursive
    fi

    ensure_git_checkout "$RTABMAP_WS/src/rtabmap" "$RTABMAP_URL" "$RTABMAP_COMMIT" "rtabmap"
    ensure_git_checkout "$RTABMAP_WS/src/rtabmap_ros" "$RTABMAP_ROS_URL" "$RTABMAP_ROS_COMMIT" "rtabmap_ros"

    if [[ ! -f "$RTABMAP_WS/src/my_rtabmap_ros.patch" && -f "$PKG/third_party/rtabmap_new/src/my_rtabmap_ros.patch" ]]; then
        cp "$PKG/third_party/rtabmap_new/src/my_rtabmap_ros.patch" "$RTABMAP_WS/src/my_rtabmap_ros.patch"
    fi

    PATCH="${MAPPING_RTABMAP_PATCH:-${RTABMAP_WS}/src/my_rtabmap_ros.patch}"
    if [[ "${MAPPING_RTABMAP_SKIP_PATCH:-}" == "1" ]]; then
        echo "[build-native] skipping rtabmap_ros patch by MAPPING_RTABMAP_SKIP_PATCH=1"
    elif [[ -f "$PATCH" ]]; then
        pushd "$RTABMAP_WS/src/rtabmap_ros" >/dev/null
        if git apply --reverse --check "$PATCH" >/dev/null 2>&1; then
            echo "[build-native] patch already applied: $PATCH"
        elif git apply --check "$PATCH" >/dev/null 2>&1; then
            echo "[build-native] applying patch: $PATCH"
            git apply "$PATCH"
            NEED_REBUILD=1
        else
            echo "[build-native] ERR: patch is neither applicable nor already applied: $PATCH" >&2
            echo "[build-native]      If this divergence is intentional, set MAPPING_RTABMAP_SKIP_PATCH=1" >&2
            popd >/dev/null
            exit 2
        fi
        popd >/dev/null
    else
        echo "[build-native] WARN: rtabmap_ros patch not found: $PATCH" >&2
    fi
else
    echo "[build-native] ERR: no rtabmap workspace resolved" >&2
    exit 2
fi

if [[ "$NEED_REBUILD" == "1" ]]; then
    echo "[build-native] colcon build self-built rtabmap workspace: $RTABMAP_WS"
    cd "$RTABMAP_WS"
    export MAKEFLAGS="${MAKEFLAGS:-${MAPPING_RTABMAP_JOBS:--j8}}"
    # shellcheck disable=SC2206
    EXTRA_COLCON_ARGS=(${MAPPING_RTABMAP_COLCON_ARGS:-})
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release "${EXTRA_COLCON_ARGS[@]}"
else
    echo "[build-native] using existing self-built rtabmap overlay: $RTABMAP_SETUP"
fi

if [[ ! -f "$RTABMAP_SETUP" ]]; then
    echo "[build-native] ERR: rtabmap setup.bash missing after build: $RTABMAP_SETUP" >&2
    exit 2
fi

source_setup_bash "$RTABMAP_SETUP"

if ! ros2 pkg list 2>/dev/null | grep -q '^rtabmap_slam$'; then
    echo "[build-native] ERR: rtabmap_slam not visible after sourcing $RTABMAP_SETUP" >&2
    exit 2
fi

if ! ros2 pkg list 2>/dev/null | grep -q '^rtabmap_examples$'; then
    echo "[build-native] WARN: rtabmap_examples not visible; old example launches may be unavailable" >&2
fi

echo "[build-native] done. Runtime overlay: $RTABMAP_SETUP"
