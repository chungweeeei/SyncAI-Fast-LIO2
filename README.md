# SyncAI-Fast-LIO2

The FAST-LIO2 / Point-LIO stack of the SyncAI robot: lidar-inertial odometry,
pose-graph loop closure, map-based relocalization and offline map refinement,
as ROS 2 Humble packages. It is a fork of
[liangheming/FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2)
(itself a ROS 2 refactor of [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)),
reworked for a Livox MID360 mounted on a quadruped and for a multi-robot DDS
domain.

It is not built on its own. It is checked out by vcstool into
`src/third-party/FASTLIO2_ROS2` of the
[SyncAI-Robot-Workspace](https://github.com/chungweeeei/SyncAI-Robot-Workspace)
and built there with `colcon` inside the robot container. The workspace's
`third-party.repos` pins the branch (`dev`); the workspace README covers
clone / build / run for the whole stack. This file covers only what is in this
repository.

## Packages

```
                       /<robot_id>/livox/{lidar,imu}          (livox_ros_driver2)
                                    │
                                    ▼
                              pointlio_node                    Point-LIO front end
                       /<robot_id>/pointlio/{lio_odom,body_cloud}
                       TF  <robot_id>/pointlio_odom → <robot_id>/pointlio_body
                          │                               │
          mapping         │                               │        navigation
          ────────────────┤                               ├────────────────────
                          ▼                               ▼
                       pgo_node                     localizer_node
              loop closure (GTSAM iSAM2)      two-stage GICP against map.pcd
              TF  map → <robot_id>/pointlio_odom    TF  map → <robot_id>/pointlio_odom
              save_maps / reset_mapping             relocalize / relocalize_check
              map_cloud_file (live merge)           initialpose
                          │
                          ▼   (offline, by hand)
                       hba_node                      refine_map over patches/ + poses.txt
```

| Package | Executable | Role | Status |
|---|---|---|---|
| `pointlio` | `pointlio_node` | **The LIO front end in use.** Point-LIO "output model": the IMU is a measurement and every lidar point is processed at its own timestamp, so there is no scan undistortion step. Publishes `lio_odom`, `body_cloud`, `world_cloud`, `lio_path` and the `world_frame → body_frame` TF. Serves `reset` (`interface/srv/ResetLIO`). | active |
| `pgo` | `pgo_node` | Keyframe selection, radius-search loop detection with ICP verification, GTSAM iSAM2 smoothing. Broadcasts the `map → local_frame` correction and serves `save_maps` and `reset_mapping`. Publishes the "map so far" merge for the operator console. | active (mapping session) |
| `localizer` | `localizer_node` | Relocalization against a saved `map.pcd`: rough GICP (0.25 m voxels) then refine GICP (0.1 m) with [small_gicp](https://github.com/koide3/small_gicp) as the backend. Broadcasts `map → local_frame`, serves `relocalize` / `relocalize_check`, listens on `initialpose`. | active (nav session) |
| `hba` | `hba_node` | Hierarchical bundle adjustment ([HBA](https://github.com/hku-mars/HBA) / [BALM](https://github.com/hku-mars/BALM)) over the patches a `save_maps` with `save_patches: true` wrote. Offline refinement. | shelved, not in any session |
| `interface` | — | The `.srv` definitions the nodes above share: `SaveMaps`, `SavePoses`, `Relocalize`, `IsValid`, `RefineMap`, `ResetLIO`, `ResetMapping`. Distinct from the workspace's `syncai_common`. | active |

The upstream `fastlio2` package (`lio_node`, the iterated-ESKF front end that
`pointlio` was refactored from) was removed in 2026-09 once Point-LIO was the
only front end in use; it is in git history.

## Dependencies

| | Where it comes from |
|---|---|
| ROS 2 Humble, PCL, Eigen, `pcl_conversions`, `message_filters`, `tf2_ros` | apt / `rosdep` |
| `livox_ros_driver2` (for `CustomMsg`) | `src/third-party/livox_ros_driver2` in the workspace |
| `small_gicp` v1.0.1 (`localizer`) | `src/third-party/small_gicp` in the workspace, built by colcon as a plain CMake package |
| GTSAM 4.2.0 (`pgo`, `hba`) | source-built into `/usr/local` by the workspace `Dockerfile` |
| Sophus 1.22.10 with `SOPHUS_USE_BASIC_LOGGING=ON` (`pointlio`, `hba`) | source-built by the workspace `Dockerfile` |
| `yaml-cpp` (`pgo`, `hba`) | apt |

GTSAM and Sophus are the two dependencies `rosdep` does not cover. Recreating
the robot container from the image restores both; a container that had them
installed by hand loses them.

## How the workspace runs it

Both real-robot launch files read `[system] robot_id` from the system INI
(default `~/robot_ws/config/system.ini`, overridable with `system_config:=`)
and use it as the node namespace, the topic prefix and the TF frame prefix.
Neither one starts `pointlio_node` itself: they `include()` `pointlio_launch.py`,
which is the single definition of how the front end is configured.

| Session | Launch | What comes up |
|---|---|---|
| mapping (`start_mapping.yaml`) | `ros2 launch pgo pgo_launch.py` | `pointlio_node` + `pgo_node` |
| navigation (`start_nav.yaml`) | `ros2 launch localizer localizer_launch.py` | `pointlio_node` + `localizer_node` |

`localizer_launch.py` also reads `[map] pcd` and the optional `[initial_pose]`
section from the same INI. The map is mandatory: the localizer loads it during
construction, so if the file is missing the launch returns an empty description
and starts nothing, on purpose. With `[initial_pose]` present, the localizer
applies that x / y / yaw as its first GICP guess on the first odom sample, so a
robot standing at its known start pose localizes without a `relocalize` call.

Resolved names, for `robot_id = robot01`:

| Kind | Name |
|---|---|
| LIO odometry | `/robot01/pointlio/lio_odom` (`nav_msgs/Odometry`, lidar clock, angular velocity filled from the output-model state) |
| LIO scan in body frame | `/robot01/pointlio/body_cloud` |
| LIO TF | `robot01/pointlio_odom → robot01/pointlio_body` |
| Map correction TF | `map → robot01/pointlio_odom` (from `pgo_node` while mapping, from `localizer_node` while navigating) |
| Services | `/robot01/pointlio/reset`, `/robot01/pgo/{save_maps,reset_mapping}`, `/robot01/localizer/{relocalize,relocalize_check}` |
| Topics for the console | `/robot01/pgo/map_cloud_file` (JSON notice), `/robot01/pgo/map_cloud` (PointCloud2, rviz only), `/robot01/localizer/map_cloud` (latched), `/robot01/localizer/initialpose` |

`pointlio_body` is physically the lidar, not the robot base. The workspace's
`syncai_lio_bridge` applies the mount extrinsic and projects the chain to 2D for
the planar nav stack; nothing in this repo knows about `base_link`.

### Mapping

```bash
# in the mapping session (or by hand, from the workspace root)
ros2 launch pgo pgo_launch.py

# drive the robot, then serialise the keyframes. The directory must exist.
ros2 service call /<robot_id>/pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/abs/path/map/<name>', save_patches: true}"
```

`save_maps` writes `map.pcd` (the merged, loop-closure-corrected cloud) and,
with `save_patches`, `patches/<i>.pcd` plus `poses.txt` (one `patch t.xyz q.wxyz`
line per keyframe, bare basenames, no absolute paths). Those two are what `hba`
consumes, and what the workspace's map catalogue expects to find in
`map/<name>/`.

`pgo_node` holds its keyframes in RAM; `save_maps` is the only thing that
serialises them. Whatever has not been saved when the process ends is gone.

**Starting a new map** is a service call, not a restart:

```bash
ros2 service call /<robot_id>/pgo/reset_mapping interface/srv/ResetMapping "{reset_lio: true}"
```

It pauses intake, resets the LIO front end over `pointlio/reset`, rebuilds the
pose graph, publishes an empty map on both map-cloud outputs and resumes with a
timestamp gate that drops the old run's tail. The map is **not** saved first.
**The robot must be standing still**: `pointlio` re-runs its static,
gravity-aligning IMU initialisation, and one done in motion produces a
permanently tilted map with no error anywhere. `reset_lio: false` resets the
pose graph alone over an unchanged odometry stream, a bag-replay and debugging
affordance.

While mapping, `pgo_node` publishes the merged "map so far" every keyframe (rate
floored at `map_cloud_pub_period`, 3 s), only when someone is subscribed. It
goes out two ways: a `PointCloud2` on `pgo/map_cloud` for rviz, and a binary PCD
written to `/dev/shm/syncai_pgo/<robot_id>/map_cloud_<seq>.pcd` announced by a
~200 B JSON notice on `pgo/map_cloud_file` (RELIABLE, TRANSIENT_LOCAL, depth 1).
The file path exists because a large site's merge is 16-45 MB, which CycloneDDS
over UDP on loopback drops through the kernel's default socket buffer. A
consumer in another container needs `ipc: host` so `/dev/shm` is the same
tmpfs. The newest two PCDs are kept; `reset_mapping` publishes a
`"points": 0` notice and removes them.

### Localization

```bash
ros2 launch localizer localizer_launch.py     # needs [map] pcd in the INI

# coarse pose in the map frame, radians. Loads the PCD again as a side effect.
ros2 service call /<robot_id>/localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/abs/path/map/<name>/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"

# did the first registration after that guess succeed?
ros2 service call /<robot_id>/localizer/relocalize_check interface/srv/IsValid "{code: 0}"
```

`relocalize` returning success is a receipt, not a result: registration runs
asynchronously on the timer and `relocalize_check` is the only thing that
reports the outcome (`code: 1` always answers `valid: true`, for callers that
only want to know the service is up).

`relocalize` takes the request's raw 6-DOF pose. The `initialpose` topic
(`geometry_msgs/PoseWithCovarianceStamped`, what rviz's "2D Pose Estimate" and
the operator console publish) and the INI `[initial_pose]` instead take only
x / y / yaw and fill z / roll / pitch from the current estimate. That matters
because the lidar is tilted and the map is gravity-aligned, so the true
`map → body` always carries the mount pitch, and a flat guess leaves the rough
stage failing its score threshold forever. A `relocalize` call is therefore
normally followed by an `initialpose` publish.

Registration runs at most `update_hz` (5 Hz) and is additionally gated on odom
motion: when the robot has moved less than `min_update_trans` / `min_update_rot`
since the last accepted registration, the previous correction is rebroadcast and
GICP is skipped, with a `max_update_interval` (2 s) backstop blended in at
`static_blend_alpha`. `post_reloc_settle` (3 s) suspends the gate after a
guess. The rationale and the measurements behind each value are in
`localizer/config/localizer.yaml`.

### Bag replay

Bags are recorded with the `robot_id` prefix the driver publishes, so a bag from
the robot replays against the launches unchanged. Bags recorded on the raw
`/livox/{lidar,imu}` topics need the remap on the player side:

```bash
ros2 bag play <bag> --remap /livox/lidar:=/<robot_id>/livox/lidar /livox/imu:=/<robot_id>/livox/imu
```

### Offline refinement (HBA)

```bash
ros2 launch hba hba_launch.py        # namespace /hba, includes rviz2
ros2 service call /hba/refine_map interface/srv/RefineMap "{maps_path: '/abs/path/map/<name>'}"
ros2 service call /hba/save_poses interface/srv/SavePoses "{file_path: '/abs/path/poses_refined.txt'}"
```

`refine_map` needs the `patches/` + `poses.txt` pair that `save_maps` writes
with `save_patches: true`. The optimisation runs on the node's timer after the
service returns; `map_points` shows progress. `hba_launch.py` has no `robot_id`
handling and is not part of any session.

## Configuration

Three different mechanisms, one per generation of the code. Which one a node
uses decides how a value can be overridden:

| Node | Mechanism | Override with |
|---|---|---|
| `pointlio_node`, `localizer_node` | Declared ROS parameters; `config/*.yaml` are `/**/<node>:` params files | launch `parameters=[...]`, `--params-file`, `ros2 param set` |
| `pgo_node`, `hba_node` | One `config_path` parameter, file hand-parsed with yaml-cpp | editing the file. `pgo_launch.py` rewrites a copy to `/tmp/syncai_pgo/pgo_<robot_id>.yaml` to inject the `robot_id`-dependent keys |

Every value that depends on `robot_id` (input topics, TF frames, the LIO reset
service name, the `/dev/shm` subdirectory) is injected by the launch file. The
values in the shipped YAML files are fallbacks for running a node bare.

Parameter meanings and tuning history are documented inline in
`pointlio/config/pointlio.yaml`, `localizer/config/localizer.yaml` and
`pgo/config/pgo.yaml`.

## Acknowledgements

- [FAST_LIO](https://github.com/hku-mars/FAST_LIO) and
  [Point-LIO](https://github.com/hku-mars/Point-LIO), HKU MaRS Lab
- [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2), liangheming,
  the ROS 2 refactor this repository forked from
- [BALM](https://github.com/hku-mars/BALM) and [HBA](https://github.com/hku-mars/HBA)
- [small_gicp](https://github.com/koide3/small_gicp), Kenji Koide
