# mapping_rbnx

Robonix v0.1 SLAM service. Algo + sensor inputs are config-driven via
`Driver(CMD_INIT, config_json)` from `rbnx boot`; this README only
covers what the **deployment shape** looks like (docker vs native) and
how to flip between them.

For the runtime contract (`robonix/service/map/{occupancy_grid, pose,
odom, pointcloud}`), per-algo topic bindings, and sensor-resolution
flow, see `src/mapping_rbnx/atlas_bridge.py` and the package manifest.

## Two execution shapes

`scripts/start.sh` picks between two paths at startup:

| Mode    | When                                                                    | Why                                                                                                       |
|---------|-------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------|
| native  | `ROBONIX_MAPPING_PLATFORM=jetson_orin` (or `ROBONIX_MAPPING_FORCE=native`) | Host already has ROS2 Humble + `ros-humble-rtabmap-ros`; container adds latency without isolation upside |
| docker  | default (no env set, or platform not in whitelist, or `ROBONIX_MAPPING_FORCE=docker`) | Safe fallback when the host toolchain isn't guaranteed (x86 dev box, isaac sim hosts, fresh installs)    |

The native-capable platform whitelist lives at the top of
`scripts/start.sh` (`NATIVE_PLATFORMS`). Adding a new entry is a
per-platform decision — don't blanket-enable, that defeats the docker
fallback.

### How the mode is decided

The cap's runtime config (which contains `platform`, `algo`, `sensors`,
…) is delivered over gRPC via `Driver(CMD_INIT, config_json)` and is
**not** materialized to a file or env var by `robonix-cli`. That means
`start.sh` cannot read `platform` out of the deploy manifest before the
cap process boots. The docker-vs-native decision is therefore made
from environment variables the operator sets in the parent shell:

```bash
# Native on Jetson:
ROBONIX_MAPPING_PLATFORM=jetson_orin rbnx boot -f robonix_manifest.yaml

# Force native on a non-whitelisted platform (you're on your own):
ROBONIX_MAPPING_FORCE=native rbnx boot -f robonix_manifest.yaml

# Force docker even if the env says jetson_orin:
ROBONIX_MAPPING_FORCE=docker rbnx boot -f robonix_manifest.yaml
```

Priority: `ROBONIX_MAPPING_FORCE` > `ROBONIX_MAPPING_PLATFORM` >
docker default.

> **Note on automation.** A future `robonix-cli` change could inject
> `ROBONIX_MAPPING_PLATFORM` automatically from the deploy manifest's
> `service.mapping.config.platform`. Until that lands, the operator
> must export it in the boot shell. See "Open follow-up" at the
> bottom.

## Native path requirements (jetson_orin)

The Jetson Orin standard image ships everything; on a custom host
make sure these are present, otherwise the native path bails loud
(`ROBONIX_MAPPING_FORCE=docker` is the fallback):

- ROS2 Humble (sourced or at `/opt/ros/humble`)
- `ros-humble-rtabmap-ros` (apt)
- `python3` with `grpcio`, `protobuf`, `pyyaml`, `numpy`
- `robonix-api` importable (run `rbnx setup` once from the robonix
  source root, or have `rbnx path robonix-api` resolvable)
- `bash scripts/build.sh` was run at least once so
  `rbnx-build/codegen/proto_gen/` is populated

## Why skip docker on jetson but keep it elsewhere

The container's value is **environment isolation**, not security or
resource sandboxing. On Jetson Orin specifically:

- The host ROS2 Humble apt feed already provides the exact same
  `rtabmap-ros` binary the Dockerfile installs.
- `--network host --ipc=host` already drops the only two isolation
  walls that would matter for ROS2.
- ARM64 docker startup overhead on Orin is non-trivial (~3–5s extra
  cold start before atlas_bridge even begins listening).
- Sharing FastRTPS SHM with sensor processes on the same host works
  trivially native; cross-container SHM was the reason
  `no_shm_profile.xml` exists in the docker path. Native sidesteps
  that complexity.

On x86 desktop / isaac sim hosts none of those points hold the same
way (toolchain drift is real, isaac sim eats the GPU + RMW config),
so the docker fallback stays the default.

## File map

| File                              | Owner of                                              |
|-----------------------------------|-------------------------------------------------------|
| `scripts/start.sh`                | Mode selection (docker vs native), trap discipline    |
| `scripts/start_native.sh`         | Host-direct atlas_bridge + start_engine, replaces docker entrypoint |
| `scripts/start_engine.sh`         | Algo dispatch (`rtabmap` / `dlio` / `fastlio2`); shared between docker and native paths |
| `scripts/build.sh`                | `rbnx codegen` + `docker build`                        |
| `docker/entrypoint.sh`            | In-container variant (only used by docker path)        |
| `docker/Dockerfile{,.jetson,.fastlio2_full}` | image variants                              |
| `src/mapping_rbnx/atlas_bridge.py`| Cap registration, CMD_INIT handling, resolved.yaml writer |
| `launch/rtabmap_2d.launch.py`     | rtabmap launch (sensor-agnostic, deploy-driven)        |

## v0.1 layering note

`scripts/start.sh` only reads operator-set environment variables
(`ROBONIX_MAPPING_FORCE`, `ROBONIX_MAPPING_PLATFORM`) to pick the
deployment shape. It does **not** parse `robonix_manifest.yaml` and
does **not** read the cap's runtime config. The cap process itself
still receives its full config exclusively through
`Driver(CMD_INIT, config_json)` over gRPC — that's the v0.1 invariant
and it is preserved.

## Open follow-up

`robonix-cli` currently delivers the per-package config block in
memory only (`build_start_config_json` in
`robonix-cli/src/cmd/run_package.rs`). A small targeted change there
to **also** export `ROBONIX_MAPPING_PLATFORM=<cfg.platform>` to the
start body for packages that opt in (or unconditionally, since the
env var is namespaced) would let `platform: jetson_orin` in the
deploy manifest drive the native path automatically, without the
operator having to remember the export. Out of scope for this commit.
