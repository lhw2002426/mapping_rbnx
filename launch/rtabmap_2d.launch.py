# SPDX-License-Identifier: MulanPSL-2.0
"""RTAB-Map launch — sensor-agnostic, deploy-driven.

The launch file does not assume any sensor combination. It branches on
which input topics the deploy actually wired up via atlas_bridge's
resolved.yaml:

    sensors.lidar2d=true   → subscribe_scan          (LaserScan)
    sensors.lidar3d=true   → subscribe_scan_cloud    (PointCloud2)
    sensors.rgb + .rgbd    → subscribe_rgb + _depth  (RGBD fusion)
    sensors.odom=true      → external odom (else rtabmap odometry node)

Webots tiago = lidar2d + rgbd + odom (LaserScan + Astra + diff-drive).
Real robot  = lidar3d + rgbd + odom + imu (Mid360 + RealSense).

start_engine.sh reads `/tmp/<algo>_resolved.yaml` and passes each topic
as a launch arg. Sentinel `<none>` means "this sensor is not in the
deploy" — the corresponding subscription is disabled.

Launch args:
    scan_topic       LaserScan      (lidar2d)         | <none> = disabled
    scan_cloud_topic PointCloud2    (lidar3d)         | <none> = disabled
    rgb_topic        Image          (camera/rgb)      | <none> = disabled
    rgb_info_topic   CameraInfo     (paired w/ rgb)   | <none> = derive
    depth_topic      Image          (camera/depth)    | <none> = disabled
    odom_topic       Odometry       (chassis/odom)    | <none> = rtabmap
                                                         runs its own
                                                         odometry node
    use_sim_time, enable_viz: standard

Outputs (declared on atlas by atlas_bridge — see _ALGO_TOPIC_BINDINGS):
    /map                 nav_msgs/OccupancyGrid (2D, lidar + depth proj)
    /rtabmap/cloud_map   sensor_msgs/PointCloud2 (3D fused cloud)
    /tf                  map→odom transform
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_NONE = "<none>"  # sentinel for "no such topic in this deploy"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value=_NONE),
        DeclareLaunchArgument("scan_cloud_topic", default_value=_NONE),
        DeclareLaunchArgument("odom_topic", default_value=_NONE),
        DeclareLaunchArgument("rgb_topic", default_value=_NONE),
        DeclareLaunchArgument("rgb_info_topic", default_value=_NONE),
        DeclareLaunchArgument("depth_topic", default_value=_NONE),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("enable_viz", default_value="false"),
        DeclareLaunchArgument("database_path", default_value=""),
        DeclareLaunchArgument("map_mode", default_value="mapping"),
        OpaqueFunction(function=_make_nodes),
    ])


def _make_nodes(context, *args, **kwargs):
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    scan_topic = LaunchConfiguration("scan_topic").perform(context)
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    rgb_topic = LaunchConfiguration("rgb_topic").perform(context)
    rgb_info_topic = LaunchConfiguration("rgb_info_topic").perform(context)
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context)
    odom_frame = LaunchConfiguration("odom_frame").perform(context)
    database_path = LaunchConfiguration("database_path").perform(context).strip()
    map_mode = LaunchConfiguration("map_mode").perform(context).strip().lower()
    enable_viz = LaunchConfiguration("enable_viz").perform(context).lower() == "true"
    use_sim_time = use_sim_time_str.lower() == "true"
    use_existing_db = bool(database_path) and os.path.exists(database_path)
    localization_mode = use_existing_db and map_mode in ("localization", "localisation")

    have_scan = bool(scan_topic) and scan_topic != _NONE
    have_scan_cloud = bool(scan_cloud_topic) and scan_cloud_topic != _NONE
    have_rgb = bool(rgb_topic) and rgb_topic != _NONE
    have_depth = bool(depth_topic) and depth_topic != _NONE
    have_rgbd = have_rgb and have_depth
    have_odom = bool(odom_topic) and odom_topic != _NONE

    if not (have_scan or have_scan_cloud or have_rgbd):
        # rtabmap with neither lidar nor RGBD has nothing to map. Bail
        # loudly so the operator notices (instead of rtabmap silently
        # idling waiting for topics that will never arrive).
        raise RuntimeError(
            "rtabmap launch: no sensor inputs enabled. Set at least one "
            "of sensors.lidar2d / sensors.lidar3d / sensors.rgbd in the "
            "deploy manifest."
        )

    rtabmap_params = {
        "use_sim_time": use_sim_time,
        "frame_id": base_frame,
        "odom_frame_id": odom_frame,
        "map_frame_id": "map",
        "publish_tf": True,
        # Sensor subscriptions branch on what the deploy actually has.
        # rtabmap accepts EITHER 2D scan OR 3D scan_cloud (or both); the
        # 3D path is what real-robot Mid360 deployments use.
        "subscribe_scan": have_scan,
        "subscribe_scan_cloud": have_scan_cloud,
        "subscribe_rgbd": False,
        "subscribe_rgb": have_rgbd,
        "subscribe_depth": have_rgbd,
        "subscribe_odom_info": False,
        "approx_sync": True,
        "queue_size": 30,
        # webots emits image stamps slightly ahead of the dynamic TF
        # for the camera chain (head_2_link → Astra → ...), causing
        # "extrapolation into the future" + "TF of received image is
        # not set" errors when wait_for_transform is short. 1.5s gives
        # the TF buffer plenty of room to catch up.
        "wait_for_transform": 1.5,
        # Build occupancy grid from BOTH scan and depth — depth fills in
        # obstacles below the lidar plane (tables, chairs) that the 2D
        # scan misses entirely. RTAB-Map 0.21+: scan goes into grid via
        # subscribe_scan, depth via Grid/FromDepth + Grid/Sensor=1.
        "Grid/Sensor": "1",
        "Grid/FromDepth": "true" if have_rgbd else "false",
        "Grid/RangeMax": "8.0",
        "Grid/CellSize": "0.05",
        "Grid/RayTracing": "true",
        # 3D pointcloud → 2D grid: when the only lidar is 3D, rtabmap
        # projects it to the planar grid via Grid/FromObstacles using
        # the same height clamp as the depth path.
        "Grid/3D": "false",
        "Grid/NormalsSegmentation": "false",
        "Grid/MaxObstacleHeight": "1.5",
        "Grid/MaxGroundHeight": "0.05",
        "Mem/IncrementalMemory": "false" if localization_mode else "true",
        "Mem/InitWMWithAllNodes": "true" if localization_mode else "false",
        "Reg/Strategy": "1",        # 0=Visual, 1=ICP, 2=Visual+ICP
        "Reg/Force3DoF": "true",
        "Optimizer/Strategy": "1",  # g2o
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/ProximityBySpace": "true",
        "RGBD/AngularUpdate": "0.05",
        "RGBD/LinearUpdate": "0.05",
        "Vis/MinInliers": "12",
        # Default DetectionRate is 1Hz — WAY too slow for tiago at
        # 0.4m/s. Between updates the robot drifts ~40cm uncorrected on
        # webots wheel odom (which has small but accumulating slip).
        # 5Hz cuts inter-frame drift to ~8cm, well within ICP capture
        # range, so scan-matching keeps the map locked.
        "Rtabmap/DetectionRate": "5.0",
        # Loosen scan-matching tolerance: with 5Hz frames the relative
        # motion is small enough that 20cm correspondence is generous,
        # not noisy. Default 0.1 was tuned for 1Hz.
        "Icp/MaxCorrespondenceDistance": "0.2",
        "Icp/MaxTranslation": "0.5",
        "Icp/MaxRotation": "0.78",  # ~45°
    }

    rtabmap_remappings = [
        # rviz "2D Pose Estimate" → /initialpose: rtabmap defaults to
        # the node-relative ~initialpose, remap to global so the rviz
        # tool reaches us without rviz config gymnastics.
        ("initialpose", "/initialpose"),
    ]
    if have_scan:
        rtabmap_remappings.append(("scan", scan_topic))
    if have_scan_cloud:
        rtabmap_remappings.append(("scan_cloud", scan_cloud_topic))
    if have_odom:
        rtabmap_remappings.append(("odom", odom_topic))
    if have_rgbd:
        rtabmap_remappings += [
            ("rgb/image", rgb_topic),
            ("rgb/camera_info", rgb_info_topic if rgb_info_topic != _NONE
                                else _derive_camera_info(rgb_topic)),
            ("depth/image", depth_topic),
        ]

    if use_existing_db:
        rtabmap_params["database_path"] = database_path

    rtabmap_args = [] if use_existing_db else ["--delete_db_on_start"]

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[rtabmap_params],
        arguments=rtabmap_args,
        remappings=rtabmap_remappings,
    )

    nodes = [rtabmap_node]

    # When the deploy didn't supply external odom, run rtabmap's own
    # ICP-odometry node off whichever lidar source we have. icp_odometry
    # consumes either /scan (LaserScan) or /scan_cloud (PointCloud2);
    # we pick based on what's wired up.
    if not have_odom and (have_scan or have_scan_cloud):
        icp_odom_remappings = []
        if have_scan_cloud:
            icp_odom_remappings.append(("scan_cloud", scan_cloud_topic))
        elif have_scan:
            icp_odom_remappings.append(("scan", scan_topic))
        icp_odom = Node(
            package="rtabmap_odom",
            executable="icp_odometry",
            name="icp_odometry",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "frame_id": base_frame,
                "odom_frame_id": odom_frame,
                "publish_tf": True,
                "approx_sync": True,
                "wait_for_transform": 1.5,
                "deskewing": False,
            }],
            remappings=icp_odom_remappings,
        )
        nodes.append(icp_odom)

    if enable_viz:
        viz_params = {
            "use_sim_time": use_sim_time,
            "frame_id": base_frame,
            "odom_frame_id": odom_frame,
            "subscribe_scan": have_scan,
            "subscribe_scan_cloud": have_scan_cloud,
            "subscribe_rgb": False,
            "subscribe_depth": False,
            "approx_sync": True,
            "queue_size": 30,
            "wait_for_transform": 1.5,
        }
        viz_remappings = []
        if have_scan:
            viz_remappings.append(("scan", scan_topic))
        if have_scan_cloud:
            viz_remappings.append(("scan_cloud", scan_cloud_topic))
        if have_odom:
            viz_remappings.append(("odom", odom_topic))
        viz = Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            name="rtabmap_viz",
            output="screen",
            parameters=[viz_params],
            remappings=viz_remappings,
        )
        nodes.append(viz)

    # tf2 → /robonix/map/pose adapter. rtabmap in mapping mode does
    # NOT publish /localization_pose; the SLAM-corrected pose is
    # only on the tf2 chain. The robonix `service/map/pose` contract
    # promises a topic-out PoseWithCovarianceStamped, so we run a
    # small adapter that polls tf2 and republishes. Without this
    # scene's self-tracker silently fell back to chassis /odom and
    # the web UI's robot dot drifted from rviz once SLAM corrected.
    #
    # ExecuteProcess (not launch_ros.Node) because the script is a
    # standalone Python file under scripts/, not a ros2 entrypoint
    # registered in a setup.py — there's no `package + executable`
    # to look up.
    pkg_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    tf_adapter = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(pkg_root, "scripts", "tf_to_pose.py"),
            "--ros-args",
            "-p", f"use_sim_time:={'true' if use_sim_time else 'false'}",
            "-p", "map_frame:=map",
            "-p", f"base_frame:={base_frame}",
            "-p", "publish_rate_hz:=10.0",
            "-p", "topic:=/robonix/map/pose",
        ],
        name="tf_to_pose",
        output="screen",
    )
    nodes.append(tf_adapter)

    return nodes


def _derive_camera_info(rgb_topic: str) -> str:
    """When the deploy doesn't tell us a camera_info topic explicitly,
    derive it by ROS convention: replace the leaf with `camera_info`.
    e.g. /head_front_camera/rgb/image_raw → /head_front_camera/rgb/camera_info.
    """
    parts = rgb_topic.rstrip("/").split("/")
    if len(parts) >= 2:
        parts[-1] = "camera_info"
        return "/".join(parts)
    return rgb_topic + "/camera_info"
