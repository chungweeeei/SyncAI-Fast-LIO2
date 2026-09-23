# Real Livox sensor variant of the localizer launch (paired with
# pointlio_launch.py).
#
# robot_id is read from the system config INI at launch time (the workspace-wide
# convention) and is used as the namespace for both nodes:
#   /<robot_id>/pointlio/...   (body_cloud, lio_odom, ...)
#   /<robot_id>/localizer/...  (relocalize, relocalize_check, map_cloud)
#
# localizer takes standard ROS 2 parameters — config/localizer.yaml is a
# /**/localizer_node params file, and the robot_id-dependent values (pointlio's
# topics, the map PCD path) are passed as a second parameters dict that overrides
# the file. No generated files involved.
#
# pointlio is NOT declared here — this launch includes pointlio_launch.py. It
# used to spawn pointlio_node itself with a `config_path` parameter pointing at a
# /tmp rewrite of pointlio.yaml, a leftover from the days when pointlio_node
# still parsed its config file with yaml-cpp itself. That node moved to pure ROS
# parameters plus relative topic names ("lidar" / "imu") with launch remappings
# long ago, which turned the config_path here into a parameter nobody reads: the
# whole pointlio tuning fell back to the struct defaults, and the subscriptions
# fell back to the relative names resolved as /<robot_id>/pointlio/{lidar,imu}
# (which have no publisher at all), so LIO received not a single message,
# published no odom / body_cloud / TF, and the localizer never had any input to
# localize with. Including rather than copying the Node definition is precisely
# so there is no longer a second definition that can drift out of step.
#
# The [map] pcd from the same INI is mandatory: the localizer loads it during
# construction, so a missing file means neither node starts. The optional
# [initial_pose] section (same one syncai_amcl reads) becomes the localizer's
# boot guess, so a robot standing at its known start pose localizes itself
# without a relocalize call.

import configparser
import os

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

logger = launch_logging.get_logger("localizer.launch")


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


def read_map_pcd(config_path: str) -> str:
    config = configparser.ConfigParser()
    if not config.read(config_path):
        return ""
    pcd = config.get("map", "pcd", fallback="").strip()
    if not pcd or os.path.isabs(pcd):
        return pcd
    workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(config_path)))
    return os.path.join(workspace_root, pcd)


def read_initial_pose(config_path: str):
    config = configparser.ConfigParser()
    if not config.read(config_path) or not config.has_section("initial_pose"):
        logger.info(
            f"No [initial_pose] in '{config_path}'; the localizer will wait for "
            "a relocalize call or an initialpose message"
        )
        return None

    try:
        pose = {
            key: config.getfloat("initial_pose", key, fallback=0.0)
            for key in ("x", "y", "yaw")
        }
    except ValueError as err:
        logger.warning(
            f"Malformed [initial_pose] in '{config_path}' ({err}); the localizer "
            "will wait for a relocalize call or an initialpose message"
        )
        return None

    logger.info(f"Initial pose from '{config_path}': {pose}")
    return {
        "set_initial_pose": True,
        "initial_pose.x": pose["x"],
        "initial_pose.y": pose["y"],
        "initial_pose.yaw": pose["yaw"],
    }


def pointlio_launch_file() -> str:
    """pointlio's own launch file — the single definition of how pointlio_node is
    configured (params file + topic remappings + robot_id frame overrides)."""
    pkg_pointlio = FindPackageShare("pointlio").find("pointlio")
    return os.path.join(pkg_pointlio, "launch", "pointlio_launch.py")


def localizer_params_file() -> str:
    """The installed /**/localizer_node params file — holds every localizer
    parameter that does not depend on robot_id."""
    pkg_localizer = FindPackageShare("localizer").find("localizer")
    return os.path.join(pkg_localizer, "config", "localizer.yaml")


def localizer_overrides(robot_id: str, map_pcd: str, initial_pose: dict) -> dict:
    """The instance-dependent localizer parameters, layered on top of the params
    file. cloud_topic / odom_topic live under pointlio's namespace, not the
    localizer's, so relative names cannot reach them. local_frame needs no
    override — the node adopts the frame_id of the first odom message.
    map_pcd is already absolute and verified to exist (see launch_setup);
    initial_pose is read_initial_pose()'s dict, or None to leave
    set_initial_pose at the params-file default (false)."""
    overrides = {
        "cloud_topic": f"/{robot_id}/pointlio/body_cloud",
        "odom_topic": f"/{robot_id}/pointlio/lio_odom",
        "map_path": map_pcd,
    }
    if initial_pose:
        overrides.update(initial_pose)
    return overrides


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    # The map is a hard requirement: the localizer calls loadMap during
    # construction (see loadInitialMap in localizer_node.cpp), and without a map
    # the whole 3D localization chain can do nothing. So when the file is missing
    # no node is started at all (equivalent to returning an empty
    # LaunchDescription) — easier to diagnose than a localizer that comes up and
    # then fails every relocalize / initialpose. The check lives here rather than
    # in generate_launch_description() because the INI path comes from the
    # system_config launch argument, whose value can only be resolved inside the
    # context.
    map_pcd = read_map_pcd(config_path)
    if not map_pcd:
        logger.error(f"No [map] pcd in '{config_path}'; nothing to launch")
        return []
    if not os.path.isfile(map_pcd):
        logger.error(f"[map] pcd '{map_pcd}' does not exist; nothing to launch")
        return []

    initial_pose = read_initial_pose(config_path)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pointlio_launch_file()),
            launch_arguments={"system_config": config_path}.items(),
        ),
        launch_ros.actions.Node(
            package="localizer",
            namespace=f"{robot_id}",
            executable="localizer_node",
            name="localizer_node",
            output="screen",
            # params file first, robot_id overrides second — later entries win
            parameters=[
                localizer_params_file(),
                localizer_overrides(robot_id, map_pcd, initial_pose),
            ],
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
