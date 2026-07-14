# Isaac Sim variant of the LIO (mapping) launch.
#
# robot_id is read from the system config INI at launch time (same convention
# as the syncai_* launches) and is used as the node namespace and as the
# prefix for both the sensor topics and the LIO TF frames:
#   topics: /<robot_id>/livox/lidar, /<robot_id>/livox/imu
#   frames: <robot_id>/lio_odom -> <robot_id>/lio_body
# lio_node reads topics/frames from its own YAML (absolute names, not affected
# by ROS namespaces), so the launch rewrites lio_isaac.yaml with the robot_id
# prefix into a generated file under /tmp before starting.

import configparser
import os
import tempfile

import yaml

import launch
import launch_ros.actions
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

DEFAULT_SYSTEM_INI = "config/system.ini"
FALLBACK_ROBOT_ID = "default_robot"

logger = launch_logging.get_logger("lio_isaac.launch")


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


def generate_lio_config(robot_id: str) -> str:
    """Rewrite lio_isaac.yaml topics/frames with the robot_id prefix and
    return the path of the generated file."""
    pkg_fastlio2 = FindPackageShare("fastlio2").find("fastlio2")
    src = os.path.join(pkg_fastlio2, "config", "lio_isaac.yaml")
    with open(src, "r") as f:
        cfg = yaml.safe_load(f)
    cfg["lidar_topic"] = f"/{robot_id}/livox/lidar"
    cfg["imu_topic"] = f"/{robot_id}/livox/imu"
    cfg["world_frame"] = f"{robot_id}/lio_odom"
    cfg["body_frame"] = f"{robot_id}/lio_body"
    generated = tempfile.NamedTemporaryFile(
        mode="w", prefix=f"lio_{robot_id}_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(cfg, generated)
    generated.close()
    logger.info(f"generated lio config: {generated.name}")
    return generated.name


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)
    lio_config = generate_lio_config(robot_id)

    return [
        launch_ros.actions.Node(
            package="fastlio2",
            namespace=f"{robot_id}/fastlio2",
            executable="lio_node",
            name="lio_node",
            output="screen",
            parameters=[{"config_path": lio_config}],
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
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz to watch the map being built",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
