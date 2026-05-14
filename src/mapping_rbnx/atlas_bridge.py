# SPDX-License-Identifier: MulanPSL-2.0
# pyright: reportMissingImports=false, reportMissingModuleSource=false
"""mapping_rbnx atlas bridge — robonix v0.1 Capability flow.

Responsibilities (kept tight; SLAM nodes do the actual work):
  1. Register `mapping` as a capability with atlas, declare a
     `service/map/driver` interface.
  2. Wait for `Driver(CMD_INIT, config_json)` from rbnx — config arrives
     ONLY through this gRPC channel (NEVER from disk / env). The cfg
     dict carries:
       - algo:    rtabmap | dlio | fastlio2[broken]
       - sensors: lidar2d / lidar3d / rgb / rgbd / imu / odom booleans
       - platform: x86_desktop / jetson_orin
  3. Resolve sensor primitives via atlas, write `/tmp/<algo>_resolved.yaml`
     for the launch file (start_engine.sh greps these out).
  4. Declare the algo-appropriate output endpoints on atlas, ALL under
     the SAME contract surface (robonix/service/map/*) so consumers
     (scene, nav) are algo-agnostic.

slam_toolbox + cartographer were removed in favour of rtabmap after
extensive head-to-head testing in webots: rtabmap handles curve-motion
mapping, loop closure, and depth fusion better than either alternative
for the cost of one extra dependency.
"""
from __future__ import annotations

import json
import logging
import os
import signal
import sys
import threading
import time
from concurrent import futures
from pathlib import Path
from typing import Any, Optional

logging.basicConfig(level=logging.INFO,
                    format="[atlas_bridge] %(levelname)s %(message)s")
log = logging.getLogger("mapping_rbnx.atlas_bridge")


def _ensure_proto_gen() -> None:
    """Make direct atlas/lifecycle stubs importable when PYTHONPATH was not prewired."""
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "rbnx-build" / "codegen" / "proto_gen"
        if pg.is_dir() and (pg / "atlas_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent


_ensure_proto_gen()

# ── Generated proto stubs ─────────────────────────────────────────────────────
# Codegen drops them under <pkg>/rbnx-build/codegen/proto_gen/. PYTHONPATH
# is set by start_native.sh / docker entrypoint.sh; _ensure_proto_gen() is a
# fallback for direct `python3 -m mapping_rbnx.atlas_bridge` runs.
import grpc  # noqa: E402

import atlas_pb2 as pb  # type: ignore  # noqa: E402
import atlas_pb2_grpc as pb_grpc  # type: ignore  # noqa: E402
import lifecycle_pb2  # type: ignore  # noqa: E402
import robonix_contracts_pb2_grpc as contracts_grpc  # type: ignore  # noqa: E402


# ── Config ────────────────────────────────────────────────────────────────────
ATLAS_ENDPOINT = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
CAP_ID = os.environ.get("ROBONIX_CAPABILITY_ID", "mapping")
NAMESPACE = "robonix/service/map"
PKG_HOST_DIR = os.environ.get("ROBONIX_PKG_HOST_DIR", "/mapping")
RESOLVED_DIR = os.environ.get("MAPPING_RESOLVED_DIR", "/tmp")
HEARTBEAT_PERIOD_S = 10.0

CMD_INIT = 0
CMD_ACTIVATE = 1
CMD_DEACTIVATE = 2
CMD_SHUTDOWN = 3

_state_lock = threading.Lock()
_atlas_stub: pb_grpc.AtlasStub | None = None
_grpc_server: grpc.Server | None = None
_initialized = False


def _truthy(s: str) -> bool:
    return str(s).strip().lower() in ("1", "true", "yes", "on")


# ── Per-algo output endpoint map ──────────────────────────────────────────────
# Each algo publishes its outputs on different topic names; the bridge
# collapses them onto the SAME contract surface so consumers don't need
# to know which algo is running. Only declare a contract when the algo
# actually publishes it.
# The exported capability surface is FIXED regardless of algo — all
# algos must back the same set of contracts so consumers (scene, nav,
# etc.) only care about contracts, never about which algo is running.
# When an algo can't natively publish a contract, the launch file is
# expected to spawn an adapter node so the topic still exists at the
# bound name. Adding a new contract to this surface is a versioning
# event for the package — bump package.version and update every algo.
_EXPORTED_CONTRACTS: tuple[str, ...] = (
    "robonix/service/map/occupancy_grid",
    "robonix/service/map/pointcloud",
    # SLAM-corrected map-frame pose (loop-closed; can jump on closure).
    # Consumed by scene self-tracker / nav / explore so they don't fall
    # back to raw chassis odom.
    "robonix/service/map/pose",
    # SLAM-corrected continuous odom (frame: odom, smooth between loop
    # closures). For high-rate velocity controllers and trajectory
    # tracking that can't tolerate the jumps in /map/pose.
    "robonix/service/map/odom",
)

# Per-algo: which internal ROS topic backs each exported contract.
# Each entry MUST cover every contract in _EXPORTED_CONTRACTS — that
# invariant is asserted at startup so we fail loud on a misconfigured
# algo rather than silently exporting fewer caps.
_ALGO_TOPIC_BINDINGS: dict[str, dict[str, str]] = {
    "rtabmap": {
        # 2D OccupancyGrid on /map (lidar + depth fusion); 3D fused
        # cloud on /rtabmap/cloud_map (useful for scene's table/chair
        # occlusion layer).
        "robonix/service/map/occupancy_grid": "/map",
        "robonix/service/map/pointcloud":     "/rtabmap/cloud_map",
        # /robonix/map/pose: PoseWithCovarianceStamped published by our
        # tf_to_pose adapter (scripts/tf_to_pose.py, launched alongside
        # rtabmap in rtabmap_2d.launch.py). The adapter polls tf2 for
        # `map → base_link` at 10 Hz and republishes — necessary
        # because rtabmap does NOT publish /localization_pose in
        # mapping mode (only in localization mode after a db is loaded).
        # Earlier this contract was bound to /rtabmap/localization_pose,
        # which silently never produced messages, and consumers
        # (scene's self-tracker) fell back to raw chassis /odom.
        "robonix/service/map/pose":           "/robonix/map/pose",
        # /rtabmap/odom: continuous odometry. When external odom is
        # supplied, rtabmap republishes it under this name with the
        # same map→odom correction baked in via /tf. When icp_odometry
        # is running internally, this IS the odom source.
        "robonix/service/map/odom":           "/rtabmap/odom",
    },
    "dlio": {
        # Direct LiDAR-Inertial Odometry: real-robot 3D livox path.
        # No native 2D OccupancyGrid — projected from /dlio's cloud
        # by a pointcloud_to_grid adapter spawned in the launch.
        "robonix/service/map/occupancy_grid": "/robonix/map/occupancy_grid",
        "robonix/service/map/pointcloud":     "/dlio/odom_node/pointcloud/deskewed",
        # DLIO publishes a single Odometry topic (no separate
        # loop-closed pose stream). We bind both contracts to it; the
        # docstring on /map/pose notes "can jump on closure" but DLIO
        # has no loop closure so it never jumps anyway.
        "robonix/service/map/pose":           "/dlio/odom_node/pose",
        "robonix/service/map/odom":           "/dlio/odom_node/odom",
    },
    "fastlio2": {
        # [BROKEN: drift] same shape as dlio, kept for repro only.
        "robonix/service/map/occupancy_grid": "/robonix/map/occupancy_grid",
        "robonix/service/map/pointcloud":     "/fastlio2/world_cloud",
        "robonix/service/map/pose":           "/fastlio2/lio_odom",
        "robonix/service/map/odom":           "/fastlio2/lio_odom",
    },
}


def _check_binding_complete(algo: str) -> None:
    """Fail loud if an algo doesn't bind every exported contract."""
    bindings = _ALGO_TOPIC_BINDINGS.get(algo, {})
    missing = [c for c in _EXPORTED_CONTRACTS if c not in bindings]
    if missing:
        raise RuntimeError(
            f"algo={algo!r} missing topic binding for contracts: {missing}. "
            f"Every algo must back the full exported surface "
            f"{list(_EXPORTED_CONTRACTS)}; add an adapter node to the "
            f"launch file or update _ALGO_TOPIC_BINDINGS."
        )


# ── Sensor → contract resolution ──────────────────────────────────────────────
# Maps `sensors.*` config flags to the atlas contract IDs scene asks
# atlas to resolve. Each entry: (config_key, contract_id, role_in_lio_yaml).
_SENSOR_CONTRACTS = [
    # (config-key,   contract,                                  yaml-key)
    ("lidar3d",  "robonix/primitive/lidar/lidar3d",       "lidar_topic"),
    ("lidar2d",  "robonix/primitive/lidar/lidar",         "scan_topic"),
    ("imu",      "robonix/primitive/imu/imu",             "imu_topic"),
    ("rgbd",     "robonix/primitive/camera/depth",        "depth_topic"),
    ("rgb",      "robonix/primitive/camera/rgb",          "rgb_topic"),
    ("odom",     "robonix/primitive/chassis/odom",        "odom_topic"),
]


def _enabled_sensors(cfg: dict[str, Any]) -> dict[str, bool]:
    """Read config.sensors. The deploy manifest is authoritative —
    every robot has different sensors, so we refuse to guess.

    A missing or empty `sensors:` block is a configuration error: the
    operator forgot to declare what the robot has, and silently
    picking "lidar2d + rgbd" would mask Mid360 deploys (where the
    correct answer is lidar3d + rgbd) and headless deploys (where the
    correct answer is rgbd-only). Fail loud instead.
    """
    sensors = cfg.get("sensors")
    if not isinstance(sensors, dict) or not sensors:
        raise RuntimeError(
            "mapping config has no `sensors:` block. Declare which "
            "sensors the robot has, e.g.:\n"
            "  sensors: { lidar2d: true, rgbd: true, odom: true }     # webots tiago\n"
            "  sensors: { lidar3d: true, rgbd: true, odom: true, imu: true }  # mid360 robot\n"
            "Supported keys: " + ", ".join(k for k, _, _ in _SENSOR_CONTRACTS)
        )
    out = {}
    for key, _contract, _yaml in _SENSOR_CONTRACTS:
        out[key] = _truthy(sensors.get(key, False))
    if not any(out.values()):
        raise RuntimeError(
            "mapping config has `sensors:` block but every entry is "
            "false/missing — at least one sensor must be enabled."
        )
    return out


# ── Atlas helpers (direct AtlasStub, no robonix_api wrapper) ──────────────────
def _resolve_sensor_endpoint(contract_id: str) -> Optional[str]:
    """Query atlas + connect one ROS2 contract; return the resolved topic."""
    if _atlas_stub is None:
        log.warning("atlas stub not ready; cannot resolve %s", contract_id)
        return None
    try:
        resp = _atlas_stub.QueryCapabilities(pb.QueryCapabilitiesRequest(
            contract_id=contract_id,
            transport=pb.TRANSPORT_ROS2,
        ))
    except grpc.RpcError as e:
        log.warning("query %s failed: %s", contract_id, e)
        return None

    for rec in resp.records:
        has_iface = any(
            iface.contract_id == contract_id and iface.transport == pb.TRANSPORT_ROS2
            for iface in rec.interfaces
        )
        if not has_iface:
            continue
        try:
            conn = _atlas_stub.ConnectCapability(pb.ConnectCapabilityRequest(
                consumer_id=CAP_ID,
                capability_id=rec.capability_id,
                contract_id=contract_id,
                transport=pb.TRANSPORT_ROS2,
            ))
        except grpc.RpcError as e:
            log.warning("connect %s/%s failed: %s", rec.capability_id, contract_id, e)
            continue
        endpoint = (conn.endpoint or "").strip()
        if conn.channel_id:
            try:
                _atlas_stub.DisconnectCapability(pb.DisconnectCapabilityRequest(
                    channel_id=conn.channel_id,
                ))
            except grpc.RpcError as e:
                log.debug("disconnect %s: %s", conn.channel_id, e)
        if endpoint:
            return endpoint
    return None


def _resolve_sensors(cfg: dict[str, Any]) -> dict[str, str]:
    """For each enabled sensor, ask atlas for the topic. Empty when
    the primitive isn't online yet (resolved.yaml still useful — the
    launch picks a sane default)."""
    enabled = _enabled_sensors(cfg)
    resolved: dict[str, str] = {}
    for cfg_key, contract_id, yaml_key in _SENSOR_CONTRACTS:
        if not enabled.get(cfg_key):
            continue
        ep = _resolve_sensor_endpoint(contract_id)
        if ep:
            resolved[yaml_key] = ep
            log.info("resolved %s → %s = %s", contract_id, yaml_key, ep)
        else:
            log.info("sensor %s (%s) not available on atlas yet", cfg_key, contract_id)
    return resolved


def _retry_resolve(cfg: dict[str, Any], deadline_s: float = 30.0,
                   settle_s: float = 8.0) -> dict[str, str]:
    """Wait until at least one sensor lands, then absorb late arrivals
    for `settle_s` so all enabled inputs end up in resolved.yaml."""
    deadline = time.time() + deadline_s
    resolved: dict[str, str] = {}
    while time.time() < deadline:
        resolved = _resolve_sensors(cfg)
        if resolved:
            break
        time.sleep(2.0)
    if not resolved:
        log.warning("no sensors discovered within %.1fs — launch will use defaults", deadline_s)
        return {}
    settle_until = time.time() + settle_s
    while time.time() < settle_until:
        time.sleep(2.0)
        more = _resolve_sensors(cfg)
        if len(more) > len(resolved):
            log.info("sensor-resolve settle: %d → %d", len(resolved), len(more))
            resolved = more
    return resolved


def _declare_ros2(contract_id: str, topic: str, qos: str = "reliable") -> str:
    if _atlas_stub is None:
        raise RuntimeError("atlas stub not ready")
    try:
        resp = _atlas_stub.DeclareInterface(pb.DeclareInterfaceRequest(
            capability_id=CAP_ID,
            contract_id=contract_id,
            transport=pb.TRANSPORT_ROS2,
            endpoint=topic,
            params=pb.TransportParams(ros2=pb.Ros2Params(qos_profile=qos)),
        ))
        return resp.endpoint or topic
    except grpc.RpcError as e:
        if e.code() == grpc.StatusCode.ALREADY_EXISTS:
            log.info("interface already declared: %s → %s", contract_id, topic)
            return topic
        raise


def _declare_outputs(algo: str) -> None:
    """DeclareInterface(transport=ROS2) for every exported contract.
    All algos publish the same contract surface — only the topic differs."""
    bindings = _ALGO_TOPIC_BINDINGS[algo]
    declared = 0
    for contract_id in _EXPORTED_CONTRACTS:
        topic = bindings[contract_id]
        try:
            actual = _declare_ros2(contract_id, topic, qos="reliable")
            declared += 1
            log.info("declared %s → ROS2 topic %s", contract_id, actual)
        except Exception as e:  # noqa: BLE001
            log.warning("declare %s failed: %s", contract_id, e)
    log.info("declared %d/%d output(s) for algo=%s",
             declared, len(_EXPORTED_CONTRACTS), algo)


def _write_resolved_yaml(algo: str, resolved: dict[str, str]) -> str:
    """Write /tmp/<algo>_resolved.yaml with k=v lines. start_engine.sh
    `grep`s these out — yaml-lite is fine, no parser needed."""
    out = os.path.join(RESOLVED_DIR, f"{algo}_resolved.yaml")
    # tf frames default for webots tiago. Real-robot deploys without a
    # base_link TF (no chassis driver, no soma URDF yet) override
    # base_frame to the lidar's own frame (e.g. livox_frame for MID-360)
    # via cfg in the deploy manifest. use_sim_time follows the same
    # cfg path so real-robot bring-ups don't get stuck waiting for a
    # /clock that never comes.
    defaults = {
        "base_frame": "base_link",
        "odom_frame": "odom",
        "map_frame": "map",
        "use_sim_time": "true",
    }
    merged = {**defaults, **resolved}
    with open(out, "w") as f:
        for k, v in merged.items():
            f.write(f"{k}: {v}\n")
    log.info("wrote resolved config → %s (%d keys)", out, len(merged))
    return out


# ── Capability + lifecycle ────────────────────────────────────────────────────
def _driver_response(ok: bool, state: str, error: str = ""):
    return lifecycle_pb2.Driver_Response(ok=ok, state=state, error=error)


def _handle_init(cfg: dict[str, Any]):
    """REGISTERED → INITIALIZED. Receives the config dict from
    rbnx via Driver(CMD_INIT, config_json). The mapping cap NEVER reads
    config from disk / env — this gRPC channel is the only sanctioned
    delivery path.

    Order:
      1. Validate algo + sensors block.
      2. Persist algo for start_engine.sh (it greps /tmp/mapping_algo).
      3. Resolve enabled sensor topics from atlas (with retry+settle —
         primitives may still be warming up when CMD_INIT lands).
      4. Write /tmp/<algo>_resolved.yaml for the launch file.
      5. DeclareInterface(ROS2) for every exported map output.
    """
    global _initialized
    with _state_lock:
        if _initialized:
            return _driver_response(True, "inactive")

    algo = cfg.get("algo", "rtabmap")
    if algo not in _ALGO_TOPIC_BINDINGS:
        return _driver_response(
            False,
            "error",
            f"unknown algo {algo!r} — supported: {list(_ALGO_TOPIC_BINDINGS)}",
        )
    try:
        _check_binding_complete(algo)
    except RuntimeError as e:
        return _driver_response(False, "error", str(e))
    if algo == "fastlio2":
        log.warning("algo=fastlio2 is BROKEN (drift); use only for repro/debug")

    log.info("CMD_INIT: algo=%s atlas=%s cap=%s", algo, ATLAS_ENDPOINT, CAP_ID)

    # Persist algo for start_engine.sh.
    os.environ["MAPPING_ALGO"] = algo
    Path("/tmp/mapping_algo").write_text(algo)

    # Discover sensors → resolved.yaml. Raises if `sensors:` block missing.
    try:
        resolved = _retry_resolve(cfg)
    except RuntimeError as e:
        return _driver_response(False, "error", str(e))
    # TF / time-source overrides from cfg. Real-robot bring-ups without
    # a chassis driver pass base_frame=livox_frame so rtabmap doesn't
    # block waiting for base_link. use_sim_time=false on real hardware.
    for key in ("base_frame", "odom_frame", "map_frame", "use_sim_time"):
        if key in cfg:
            resolved[key] = str(cfg[key]).lower() if key == "use_sim_time" else str(cfg[key])
    _write_resolved_yaml(algo, resolved)

    # Declare outputs (after resolved.yaml so launch can start in parallel).
    _declare_outputs(algo)
    with _state_lock:
        _initialized = True
    return _driver_response(True, "inactive")


class _MapDriverServicer(contracts_grpc.RobonixServiceMapDriverServicer):
    def Driver(self, request, context):
        cmd = int(request.command)
        if cmd == CMD_INIT:
            try:
                cfg = json.loads(request.config_json) if request.config_json else {}
            except json.JSONDecodeError as e:
                return _driver_response(False, "error", f"bad config_json: {e}")
            try:
                return _handle_init(cfg)
            except Exception as e:  # noqa: BLE001
                log.exception("CMD_INIT failed")
                return _driver_response(False, "error", str(e))
        if cmd == CMD_ACTIVATE:
            return _driver_response(True, "active")
        if cmd == CMD_DEACTIVATE:
            return _driver_response(True, "inactive")
        if cmd == CMD_SHUTDOWN:
            return _driver_response(True, "terminated")
        return _driver_response(False, "error", f"invalid command {cmd}")


def _start_grpc(requested_port: int) -> int:
    global _grpc_server
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    contracts_grpc.add_RobonixServiceMapDriverServicer_to_server(
        _MapDriverServicer(), server)
    bound = server.add_insecure_port(f"0.0.0.0:{requested_port}")
    log.info("add_insecure_port(0.0.0.0:%d) returned %d", requested_port, bound)
    if bound == 0:
        log.warning("bind to 0.0.0.0:%d failed; retrying with OS-assigned port", requested_port)
        bound = server.add_insecure_port("0.0.0.0:0")
        log.info("add_insecure_port(0.0.0.0:0) returned %d", bound)
        if bound == 0:
            log.error("gRPC cannot bind any port; aborting")
            sys.exit(2)
    server.start()
    _grpc_server = server
    log.info("Mapping gRPC serving on 0.0.0.0:%d (requested %d)", bound, requested_port)
    return bound


def _declare_grpc(contract_id: str, port: int, service_name: str, method: str) -> None:
    if _atlas_stub is None:
        raise RuntimeError("atlas stub not ready")
    _atlas_stub.DeclareInterface(pb.DeclareInterfaceRequest(
        capability_id=CAP_ID,
        contract_id=contract_id,
        transport=pb.TRANSPORT_GRPC,
        endpoint=f"127.0.0.1:{port}",
        params=pb.TransportParams(grpc=pb.GrpcParams(
            proto_file="robonix_contracts.proto",
            service_name=service_name,
            method=method,
        )),
    ))


def _heartbeat_loop() -> None:
    while True:
        time.sleep(HEARTBEAT_PERIOD_S)
        if _atlas_stub is None:
            continue
        try:
            _atlas_stub.Heartbeat(pb.HeartbeatRequest(capability_id=CAP_ID))
        except Exception as e:  # noqa: BLE001
            log.debug("heartbeat: %s", e)


def _on_signal(signum, _frame):
    log.info("signal %d — shutting down", signum)
    sys.exit(0)


def main() -> int:
    global _atlas_stub

    requested_port = int(os.environ.get("MAPPING_GRPC_PORT", "50120"))
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    driver_port = _start_grpc(requested_port)
    os.environ["MAPPING_GRPC_PORT"] = str(driver_port)

    channel = grpc.insecure_channel(ATLAS_ENDPOINT)
    stub = pb_grpc.AtlasStub(channel)
    _atlas_stub = stub
    md_path = f"{PKG_HOST_DIR}/CAPABILITY.md" if PKG_HOST_DIR else ""

    try:
        stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=CAP_ID,
            namespace=NAMESPACE,
            capability_md_path=md_path,
        ))
        log.info("registered cap %s namespace=%s", CAP_ID, NAMESPACE)
    except grpc.RpcError as e:
        if e.code() == grpc.StatusCode.ALREADY_EXISTS:
            log.info("cap %s already registered; continuing", CAP_ID)
        else:
            log.warning("atlas RegisterCapability failed: %s", e)

    try:
        _declare_grpc("robonix/service/map/driver", driver_port,
                      "ServiceMapDriver", "Driver")
        log.info("declared map driver iface on :%d (awaiting INIT)", driver_port)
    except grpc.RpcError as e:
        if e.code() == grpc.StatusCode.ALREADY_EXISTS:
            log.info("map driver iface already declared; continuing")
        else:
            log.warning("atlas DeclareInterface(driver) failed: %s", e)

    threading.Thread(target=_heartbeat_loop, daemon=True).start()
    log.info("ready — awaiting Driver(CMD_INIT)")
    while True:
        time.sleep(60.0)


if __name__ == "__main__":
    sys.exit(main())
