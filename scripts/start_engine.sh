#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# In-container SLAM engine launcher. Picks the algo from MAPPING_ALGO
# (set by atlas_bridge after parsing config) and execs the
# corresponding ros2 launch.
#
# Supported algos:
#   rtabmap   — Sensor-agnostic graph SLAM. Subscribes to whatever
#               sensors.* the deploy enabled (lidar2d / lidar3d / rgbd
#               / odom). Webots tiago = lidar2d + rgbd; real robot
#               (mid360) = lidar3d + rgbd.
#   dlio      — Direct LiDAR-Inertial Odometry (3D Livox; real robot)
#   fastlio2  — [BROKEN: drift] kept for repro/debug only
#
# Pre-conditions: atlas_bridge has already written
# /tmp/<algo>_resolved.yaml with discovered topic names.
set -eo pipefail

ALGO="${MAPPING_ALGO:-rtabmap}"
RESOLVED="/tmp/${ALGO}_resolved.yaml"

# Package root: in the docker path everything is bind-mounted under
# /mapping; in the native path we may not have that symlink so callers
# (scripts/start_native.sh) export MAPPING_PKG_ROOT instead. Default
# preserves the original docker behaviour.
PKG_ROOT="${MAPPING_PKG_ROOT:-/mapping}"

# Source ROS2 only if the parent shell didn't already (the native
# path sources it before this point; docker entrypoint always needs it).
if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

read_y() {
    # Never fail under `set -e`: grep returns 1 on no match, which
    # would otherwise abort the script before we even get to the
    # case branch. Default-empty is what every caller wants anyway.
    { grep -E "^$1:" "$RESOLVED" 2>/dev/null || true; } | head -1 | awk '{print $2}' || true
}

case "$ALGO" in
    rtabmap)
        # Sensor inputs are deploy-driven. Every topic comes from
        # /tmp/<algo>_resolved.yaml, written by atlas_bridge after it
        # resolved the contracts enabled by `sensors.*` in the deploy
        # manifest. Keys (see atlas_bridge:_SENSOR_CONTRACTS):
        #   scan_topic       ← robonix/primitive/lidar/lidar    (lidar2d)
        #   lidar_topic      ← robonix/primitive/lidar/lidar3d  (lidar3d)
        #   odom_topic       ← robonix/primitive/chassis/odom
        #   rgb_topic        ← robonix/primitive/camera/rgb
        #   depth_topic      ← robonix/primitive/camera/depth
        # Webots tiago = lidar2d + rgbd + odom.
        # Real robot   = lidar3d + rgbd + odom (+ imu, unused by rtabmap).
        # Anything not present in resolved.yaml passes through as the
        # `<none>` sentinel and the launch file disables that subscription.
        SCAN_TOPIC=$(read_y scan_topic)
        SCAN_CLOUD_TOPIC=$(read_y lidar_topic)
        ODOM_TOPIC=$(read_y odom_topic)
        RGB_TOPIC=$(read_y rgb_topic)
        DEPTH_TOPIC=$(read_y depth_topic)
        SCAN_TOPIC="${SCAN_TOPIC:-<none>}"
        SCAN_CLOUD_TOPIC="${SCAN_CLOUD_TOPIC:-<none>}"
        ODOM_TOPIC="${ODOM_TOPIC:-<none>}"
        RGB_TOPIC="${RGB_TOPIC:-<none>}"
        DEPTH_TOPIC="${DEPTH_TOPIC:-<none>}"
        # tf + time-source from atlas_bridge's resolved.yaml (cfg-driven
        # in the deploy manifest); fall back to legacy env / defaults.
        # Real-robot bring-ups without a chassis TF override base_frame
        # to the lidar's own frame so rtabmap doesn't block on base_link.
        BASE_FRAME=$(read_y base_frame)
        ODOM_FRAME=$(read_y odom_frame)
        USE_SIM_TIME_R=$(read_y use_sim_time)
        BASE_FRAME="${BASE_FRAME:-base_link}"
        ODOM_FRAME="${ODOM_FRAME:-odom}"
        USE_SIM_TIME="${USE_SIM_TIME_R:-${MAPPING_USE_SIM_TIME:-true}}"
        ENABLE_VIZ="${MAPPING_ENABLE_VIZ:-false}"
        echo "[start_engine] rtabmap scan2d=$SCAN_TOPIC scan3d=$SCAN_CLOUD_TOPIC odom=$ODOM_TOPIC rgb=$RGB_TOPIC depth=$DEPTH_TOPIC base=$BASE_FRAME odomf=$ODOM_FRAME use_sim_time=$USE_SIM_TIME viz=$ENABLE_VIZ"
        exec ros2 launch "${PKG_ROOT}/launch/rtabmap_2d.launch.py" \
            scan_topic:="$SCAN_TOPIC" \
            scan_cloud_topic:="$SCAN_CLOUD_TOPIC" \
            odom_topic:="$ODOM_TOPIC" \
            rgb_topic:="$RGB_TOPIC" \
            depth_topic:="$DEPTH_TOPIC" \
            base_frame:="$BASE_FRAME" \
            odom_frame:="$ODOM_FRAME" \
            use_sim_time:="$USE_SIM_TIME" \
            enable_viz:="$ENABLE_VIZ"
        ;;

    dlio)
        # Direct LiDAR-Inertial Odometry — real-robot 3D livox path.
        # Requires the `dlio` ros2 package mounted/installed in the
        # image; for desktop deploys bind-mount the slam_alt_ws colcon
        # build at /ws/install.
        if [ -f /ws/install/setup.bash ]; then
            source /ws/install/setup.bash
        else
            echo "[start_engine] ERR: dlio needs /ws/install (slam_alt_ws colcon build)" >&2
            exit 2
        fi
        echo "[start_engine] dlio (3D Livox + IMU)"
        exec ros2 launch dlio dlio.launch.py
        ;;

    fastlio2)
        # [BROKEN: drift] kept only so the repro path stays reachable.
        # Image must include /ws/install built from FASTLIO2_ROS2
        # (the heavier Dockerfile.fastlio2_full path). Do NOT pick
        # fastlio2 for any production deploy until the global drift
        # issue is fixed.
        if [ -f /ws/install/setup.bash ]; then
            source /ws/install/setup.bash
        fi
        echo "[start_engine] WARN fastlio2 has known drift; use for repro only"
        exec ros2 launch "${PKG_ROOT}/launch/slam_mapping_native.launch.py"
        ;;

    *)
        echo "[start_engine] unknown algo: $ALGO (supported: rtabmap | dlio | fastlio2)" >&2
        exit 2
        ;;
esac
