# Pose-graph-optimisation launch, paired with pointlio_launch.py.
#
# robot_id is read from the system config INI at launch time (the workspace-wide
# convention) and is used as the namespace for both nodes, exactly like the
# pointlio / localizer launches:
#   /<robot_id>/pointlio/...   (body_cloud, lio_odom, ...)
#   /<robot_id>/pgo/...        (loop_markers, save_maps)
#
# pointlio is NOT declared here — this launch includes pointlio_launch.py, which
# is the single definition of how pointlio_node is configured (params file +
# topic remappings + robot_id frame overrides). It used to spawn pointlio_node
# itself with a `config_path` parameter pointing at pointlio.yaml, which was left
# over from the era when pointlio_node parsed that YAML itself with yaml-cpp.
# That node now takes plain ROS parameters and relative topic names, so the
# config_path here was read by nobody: every LIO tuning value fell back to the
# struct defaults and the subscriptions fell back to
# /<robot_id>/pointlio/{lidar,imu}, which nothing publishes — no odom, no
# body_cloud, no TF, and therefore nothing for pgo to optimise. Including
# pointlio's own launch keeps that definition in one place.
#
# pgo_node, unlike pointlio_node, still hand-parses its YAML with yaml-cpp
# (`config_path`), so the robot_id-dependent keys cannot be passed as parameter
# overrides. They are rewritten into a generated copy of pgo.yaml under /tmp —
# derived state, same treatment as syncai_bringup's livox JSON. The file name is
# deterministic (one per robot_id) so relaunching overwrites instead of littering
# /tmp, which the old tempfile.NamedTemporaryFile(delete=False) did.
#
# Frames: TF frame ids are not namespaced by ROS, so local_frame has to be
# prefixed by hand. It must match pointlio's world_frame
# (<robot_id>/pointlio_odom) — pgo broadcasts map -> local_frame and, unlike the
# localizer, does NOT adopt that frame from the incoming odom messages, so a
# mismatch here means the correction lands on a frame nobody looks up. map_frame
# stays plain `map`.

import configparser
import os

import yaml

import launch
import launch_ros.actions
from launch import logging as launch_logging
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"

# The generated pgo config lives under /tmp because it is derived state: rebuilt
# from the installed pgo.yaml on every launch, never edited by hand.
GENERATED_CONFIG_DIR = "/tmp/syncai_pgo"

logger = launch_logging.get_logger("pgo.launch")


def read_robot_id(config_path: str) -> str:
    config = configparser.ConfigParser()
    if not config.read(config_path):
        logger.warning(
            f"System config '{config_path}' not found; "
            f"falling back to robot_id '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    robot_id = config.get("system", "robot_id", fallback="").strip()
    if not robot_id:
        logger.warning(
            f"No [system] robot_id in '{config_path}'; "
            f"falling back to '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    return robot_id


def pointlio_launch_file() -> str:
    """pointlio's own launch file — the single definition of how pointlio_node is
    configured (params file + topic remappings + robot_id frame overrides)."""
    pkg_pointlio = FindPackageShare("pointlio").find("pointlio")
    return os.path.join(pkg_pointlio, "launch", "pointlio_launch.py")


def generate_pgo_config(robot_id: str) -> str:
    """Copy the installed pgo.yaml with the robot_id-dependent keys rewritten,
    and return the generated path. pgo_node reads this file directly with
    yaml-cpp, so the values have to be baked in rather than layered as parameter
    overrides."""
    pkg_pgo = FindPackageShare("pgo").find("pgo")
    source = os.path.join(pkg_pgo, "config", "pgo.yaml")
    with open(source, "r") as f:
        config = yaml.safe_load(f)

    # pointlio's outputs live in its own namespace, not pgo's, so relative names
    # cannot reach them.
    config["cloud_topic"] = f"/{robot_id}/pointlio/body_cloud"
    config["odom_topic"] = f"/{robot_id}/pointlio/lio_odom"
    config["local_frame"] = f"{robot_id}/pointlio_odom"

    os.makedirs(GENERATED_CONFIG_DIR, exist_ok=True)
    generated = os.path.join(GENERATED_CONFIG_DIR, f"pgo_{robot_id}.yaml")
    with open(generated, "w") as f:
        yaml.safe_dump(config, f)
    logger.info(f"generated pgo config: {generated}")
    return generated


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pointlio_launch_file()),
            launch_arguments={"system_config": config_path}.items(),
        ),
        launch_ros.actions.Node(
            package="pgo",
            namespace=f"{robot_id}/pgo",
            executable="pgo_node",
            name="pgo_node",
            output="screen",
            parameters=[{"config_path": generate_pgo_config(robot_id)}],
        ),
    ]


def generate_launch_description():
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "system_config",
                default_value=DEFAULT_SYSTEM_INI,
                description="Path to the system INI file providing [system] robot_id",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
