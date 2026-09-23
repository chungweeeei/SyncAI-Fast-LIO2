# CLAUDE.md

Guidance for Claude Code when working in this repository. `README.md` says what
the packages do and how they are run; this file is about editing them without
breaking the things that depend on them.

## What this repository is

The FAST-LIO2 / Point-LIO fork of the SyncAI robot: `pointlio` (the LIO front
end in use), `pgo` (loop closure, map saving, the live map hand-off), `localizer`
(GICP relocalization against a saved map), `hba` (offline refinement, shelved),
`fastlio2` (the original iterated-ESKF node, legacy) and `interface` (the shared
`.srv` files). Forked from `liangheming/FASTLIO2_ROS2`; remote is
`chungweeeei/SyncAI-Fast-LIO2`, working branch `dev`.

It lives at `src/third-party/FASTLIO2_ROS2` inside `SyncAI-Robot-Workspace`, put
there by `vcs import < third-party.repos`, and is built by that workspace's
`colcon`. It has no build of its own. The workspace's `CLAUDE.md` (two
directories up) governs the conventions this repo shares with the rest of the
stack (the `robot_id` namespacing rule, English-only comments and docs, "why"
comments, builds happen inside the robot container, do not run `colcon build`
unprompted). Read it first; this file only adds what is specific here.

## The one fact that shapes everything: two consumers, one layer up

Nothing here is an end product. Two things outside this repo read its ROS
surface, and neither is in this checkout:

- **`syncai_lio_bridge`** (in the workspace) turns `pointlio/lio_odom` +
  `pointlio_odom → pointlio_body` + `map → pointlio_odom` into the planar
  `odom → base_link` and `map → odom` the nav stack needs. It reads the LIO odom
  frame from the message header, not from a parameter, and assumes
  `pointlio_body` is physically the lidar.
- **The operator backend** (`chungweeeei/SyncAI-Robot-Backend`, its own
  container, host networking, DDS domain 1) calls `pgo/save_maps`,
  `pgo/reset_mapping`, `localizer/relocalize`, `relocalize_check`, publishes
  `initialpose`, reads `pgo/map_cloud_file` and `pointlio/body_cloud`, and
  reads the PCD files under `/dev/shm/syncai_pgo/<robot_id>/`. It builds
  against the `interface` package as checked out in the workspace.

So a service or topic rename, a changed `.srv` field, a different frame name, a
different `map_cloud_file` JSON shape, a different `save_maps` directory layout
or a different `/dev/shm` path is a **cross-repository change**. Say so in the
commit message and keep the old shape working until the consumers have moved.
`README.md`'s "Resolved names" table is the surface; treat it as an API.

## Package layout and which code is live

| Package | Live? | Notes |
|---|---|---|
| `pointlio` | yes, both sessions | Refactored "in the fastlio2 style" from Point-LIO. `map_builder/` holds the math (`point_ekf`, `imu_initializer`, `lidar_processor`, `ikd_Tree`); `pointlio_node.cpp` is the ROS shell. |
| `pgo` | yes, mapping session | `pgos/simple_pgo.*` is the graph; `pgo_node.cpp` is the shell plus the map-cloud hand-off and the reset orchestration. |
| `localizer` | yes, nav session | `localizers/icp_localizer.*` wraps `small_gicp::RegistrationPCL`; `localizer_node.cpp` owns the motion gate and the guess paths. |
| `hba` | no | Upstream code, 4-space indentation, untouched by the workspace `.clang-format`. `src/hba_node copy.cpp` is a stray upstream file that `CMakeLists.txt` does not build. Leave both alone unless the package is revived. |
| `fastlio2` | no | Reference implementation. `config/lio_isaac.yaml` and its header comment refer to Isaac launches deleted in `890a54e`. Do not "fix" the pointlio behaviour by editing this package. |
| `interface` | yes | The `.srv` comments are the design record for the reset contract; edit them with the code. |

The Isaac Sim variants (`*_isaac_launch.py`, `pointlio_isaac.yaml`, the
`pcd_publisher` package) were removed in `890a54e` / `f3f752f`. Recover them from
git history rather than re-deriving them; `lidar_type: 1` (PointCloud2 input)
and `imu_acc_scale` still exist in the node for that path.

## Three configuration mechanisms, and which nodes use which

This is the most common source of "I changed the value and nothing happened":

1. **Declared ROS parameters** (`pointlio_node`, `localizer_node`).
   `config/pointlio.yaml` and `config/localizer.yaml` are `/**/<node_name>:`
   params files loaded through `parameters=[file, overrides]`. Every key is a
   `declare_parameter` with the struct default as its default, so a missing key
   degrades to the default instead of throwing. Numeric types matter: `cube_len:
   300` in a params file is an int64 and the node dies with
   `InvalidParameterTypeException`; write `300.0`. `r_il` is a flat 9-element
   row-major list because ROS parameters have no matrix type, and the node
   validates the length.
2. **Hand-parsed yaml-cpp behind a single `config_path` parameter** (`pgo_node`,
   `hba_node`, `lio_node`). Nothing but `config_path` is visible to `ros2 param`;
   a missing required key is an `InvalidNode` exception at construction. Because
   values cannot be layered as overrides, `pgo_launch.py` copies `pgo.yaml` to
   `/tmp/syncai_pgo/pgo_<robot_id>.yaml` with the `robot_id`-dependent keys
   rewritten and points `config_path` at that. New keys in `pgo_node` should be
   read defensively (`if (config["key"])`), because a stale generated copy from
   an older launch will lack them.
3. **Launch-time injection of everything `robot_id`-dependent**: input topics
   (remapping for `pointlio`, absolute names for `pgo` / `localizer` because
   they point into `pointlio`'s namespace), TF frames (`<robot_id>/pointlio_odom`
   / `<robot_id>/pointlio_body`), `lio_reset_service`, `map_cloud_dir`,
   `map_path`, `set_initial_pose` and `initial_pose.*`. The values in the
   shipped YAML files are fallbacks for running a node bare and **must never
   contain a robot_id**.

`pgo_launch.py` and `localizer_launch.py` do not declare `pointlio_node`
themselves; they `IncludeLaunchDescription` `pointlio_launch.py`. That was a fix
for a real bug (a second, drifted copy of the pointlio definition passed a
`config_path` nobody read, so the LIO fell back to struct defaults and
subscribed to topics nothing published). Keep it that way: one definition.

## Frames

- `pointlio` publishes `world_frame → body_frame`, overridden by launch to
  `<robot_id>/pointlio_odom → <robot_id>/pointlio_body`. The body frame is
  deliberately **not** `base_link` and not `laser`: `base_link` already has a
  parent from `syncai_lio_bridge`, and tf2 keys its cache by child frame, so two
  parents interleave and the answer depends on lookup time (that bug showed up
  as a ~15° pitch disagreement between rviz and the console).
- `pgo` broadcasts `map → local_frame` with `local_frame` taken from its config
  (rewritten by launch). It does **not** adopt the frame from the incoming odom
  message, so `pgo.yaml`'s `local_frame` rewrite must match pointlio's
  `world_frame` override or the correction lands on a frame nobody looks up.
- `localizer` broadcasts `map → local_frame` and **does** adopt `local_frame`
  from the first odom message's `header.frame_id`; its YAML value is only a
  placeholder.
- `map` is never prefixed.
- `pgo/rviz/pgo.rviz` still has `Fixed Frame: lidar`, the upstream frame name;
  the other rviz files use `map`. rviz configs here are workstation conveniences,
  not part of any session.

## Executors, locks and the reset contract

Each node has a deliberate threading arrangement. Changing an executor or a
callback group changes correctness, not just performance:

- **`pointlio_node`** runs on `rclcpp::spin()` (single thread). `resetCB` takes
  **no lock** against `timerCB` and relies on that: moving it to a
  `MultiThreadedExecutor` or its own callback group needs a mutex shared with
  the timer around `m_builder` / `m_kf`.
- **`localizer_node`** runs a 2-thread `MultiThreadedExecutor`: timer +
  subscriptions on the default group, `relocalize` / `relocalize_check` /
  `initialpose` on a separate MutuallyExclusive group so a multi-second
  `loadMap` never gaps the TF rebroadcast. `m_target_mutex` in `ICPLocalizer`
  exists for that.
- **`pgo_node`** runs a 3-thread `MultiThreadedExecutor`: default group (timer,
  both subscriptions, `save_maps`), a group for `reset_mapping`, and a group for
  the `ResetLIO` client. The client group is what stops `resetMappingCB`
  deadlocking on its own future. `m_pgo_mutex` guards `m_pgo`, which the reset
  **replaces** rather than mutates, so every reader takes it (`timerCB` and
  `saveMapsCB` whole-body). It is never held while waiting on the LIO future.
  The intake gate is two atomics, not a lock, on purpose. `main()` keeps a named
  `shared_ptr` to the node because `Executor::add_node` holds a weak_ptr.
- `std::lock_guard<std::mutex>(m);` without a variable name is a temporary that
  unlocks immediately. This file had two of those and got away with it only
  under the single-threaded executor. Always name the guard.

**The reset ordering contract** (`ResetLIO.srv`, `ResetMapping.srv`, and the
comments in both nodes): `pgo` pauses intake → calls `pointlio/reset` → gets
back `last_odom_time` (the lidar stamp of the last published odom) → rebuilds
the graph → resumes, dropping every pair with stamp `<= last_odom_time`. The
stamp is the lidar header stamp on both sides, so the comparison is exact.
`pointlio` records `m_last_odom_time` in `timerCB` past the MAPPING gate, not
inside the subscriber-gated `publishOdometry`, and clears `lidar_pushed` so the
first new-run frame cannot reuse an old package. The only fallible step runs
before anything is destroyed. If you touch any of this, re-read those two `.srv`
files first; they are the specification.

The reset re-runs a **static** IMU initialisation. The robot must be still, and
nothing enforces that, by decision: a stationarity check cannot tell a still
robot from a vibrating one, and refusing a still robot is worse than warning.

## The map-cloud file hand-off (`pgo`)

`pgo/map_cloud` (PointCloud2) stays for rviz. The backend reads
`pgo/map_cloud_file`: a hand-formatted JSON notice with exactly five fields
(`seq`, `path`, `points`, `frame_id`, `stamp{sec,nanosec}`), RELIABLE +
TRANSIENT_LOCAL depth 1, naming a binary PCD written as `.tmp` + rename under
`map_cloud_dir` (`/dev/shm/syncai_pgo/<robot_id>`). Newest two kept, by `seq`.
The empty-map notice from a reset is published **ungated** and **before** the
files are removed. The merge runs on a dedicated `std::thread` from a by-value
snapshot of the keyframes, claimed by an atomic, and every exception is caught
inside the worker because one escaping a `std::thread` is `std::terminate`. Do
not add a JSON library for this; do not make `setupMapCloudDir` failure fatal
(the node also owns the TF broadcast). `ipc: host` on both containers is what
makes the path mean the same file; without it the reader gets ENOENT.

## Localizer specifics

- `relocalize` success is a receipt. Registration is async on the timer;
  `relocalize_check` reports the outcome. Do not turn TF presence into a
  quality signal anywhere.
- `relocCB` uses the raw 6-DOF request. `initialpose` and the INI
  `[initial_pose]` go through `applyPlanarGuess`, which keeps the current
  roll / pitch / z and only sets x / y / yaw, because the tilted mount plus a
  gravity-aligned map means a flat guess never passes the rough score. Keep
  both paths; the backend's map switch calls `relocalize` then publishes
  `initialpose` for exactly this reason.
- `small_gicp` reports `converged` only on a small update step; hitting
  `max_iteration` is *not converged*, unlike PCL ICP, and `align()` treats
  `hasConverged()` as a hard condition. Iteration counts below the small_gicp
  default of 20 silently stop the TF from ever updating. `num_threads` is read
  once at construction (the KD-tree is built in `setInputTarget`).
- Motion gate (`min_update_trans`, `min_update_rot`, `max_update_interval`,
  `static_blend_alpha`, `post_reloc_settle`): the failure direction chosen is
  "jitter partly suppressed", never "frozen on a stale pose". A relocalize
  bypasses the gate and the blend and opens a full-rate settle window. The
  rejected alternatives (plain EMA, plain deadband) and the measurements are in
  `localizer.yaml`; do not re-propose them without new data.
- `localizer_launch.py` returns an empty description when `[map] pcd` is
  missing or absent on disk. That is intentional (a localizer with no map fails
  every call silently); do not make it start anyway.

## Conventions

- **English only**, comments and docs included. The workspace translated its
  last Chinese remnants in 2026-09; this repo still carries Chinese in
  `localizer_node.cpp`, `localizer.yaml`, `icp_localizer.h`, parts of
  `pgo_node.cpp`, `pointlio_node.cpp`, `lio_node.cpp` and two launch headers.
  Anything **new** is written in English; translating a block you are already
  editing is welcome, mass-translating untouched code is a separate change.
- Comments explain **why**, at the density you see in `pgo_node.cpp` and
  `localizer.yaml`: the bug, the measurement, the rejected alternative, the
  date. A bare tuning change with no rationale is out of place. Tuning notes
  cite the robot and the run (`record/loc_run_YYYYMMDD_HHMMSS.csv` in the
  workspace; that directory is gitignored, so the citation is the only record).
- Formatting: the workspace root `.clang-format` (ROS 2 style, 100 columns)
  applies to `pointlio`, `pgo`, `localizer` and `fastlio2`. `hba` is upstream
  4-space code and is left as is. Launch files are linted by the workspace root
  `ruff.toml`.
- There are **no tests** in this repo beyond the ament linter templates. Behaviour
  is verified on the robot or against a bag; say which in the commit.
- No `.gitignore` here: `**/launch/__pycache__/` shows up as untracked after a
  launch runs (`e31ee58` removed committed ones). Do not commit it.
- Commit messages follow `type(scope): summary` (`feat(localizer): …`,
  `feat(pgo): …`, `chore: …`).

## Build and bump

```bash
# inside the robot container, cwd = workspace root
colcon build --symlink-install --packages-select interface pointlio pgo localizer
source install/setup.bash
```

Build `interface` first (or let colcon order it) whenever a `.srv` changes;
`pgo`, `localizer`, `pointlio` and `hba` all depend on it. `small_gicp` must be
present in `src/third-party/` for `localizer` to configure. Do not run the build
unprompted; the container is usually a live robot.

The workspace consumes this repo through `third-party.repos` at `version: dev`.
A change here reaches the robot when the workspace re-imports; the pin's own
comment says a SHA would be the reproducible choice. Nothing in this repo needs
to change for a bump, but the workspace `CLAUDE.md`, its `README.md` and the
package READMEs of `syncai_lio_bridge` / `syncai_bringup` describe this stack's
behaviour and may need the same edit.
