# Isaac Sim variant of the PGO (mapping + loop closure) launch.
#
# Starts lio_node (Isaac config) + pgo_node, all namespaced with the robot_id
# read from the system config INI (same convention as the syncai_* launches):
#   /<robot_id>/fastlio2/...  (body_cloud, lio_odom, ...)
#   /<robot_id>/pgo/...       (save_maps service, loop markers, ...)
#   frames: map -> <robot_id>/lio_odom -> <robot_id>/lio_body
# Both nodes read topics/frames from their own YAMLs (absolute names, not
# affected by ROS namespaces), so the launch rewrites them with the robot_id
# prefix into generated files under /tmp before starting.

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

logger = launch_logging.get_logger("pgo_isaac.launch")


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


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    pkg_pgo = FindPackageShare("pgo").perform(context)
    pkg_fastlio2 = FindPackageShare("fastlio2").perform(context)

    # lio: sensor topics + TF frames get the robot_id prefix.
    lio_config_src = os.path.join(pkg_fastlio2, "config", "lio_isaac.yaml")
    with open(lio_config_src, "r") as f:
        lio_config = yaml.safe_load(f)
    lio_config["lidar_topic"] = f"/{robot_id}/livox/lidar"
    lio_config["imu_topic"] = f"/{robot_id}/livox/imu"
    lio_config["world_frame"] = f"{robot_id}/lio_odom"
    lio_config["body_frame"] = f"{robot_id}/lio_body"
    lio_generated = tempfile.NamedTemporaryFile(
        mode="w", prefix=f"lio_{robot_id}_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(lio_config, lio_generated)
    lio_generated.close()
    logger.info(f"generated lio config: {lio_generated.name}")

    # pgo: input topics follow lio_node's namespace; local_frame must match
    # the rewritten LIO world frame (pgo broadcasts map -> local_frame and,
    # unlike the localizer, does NOT adopt it from the odom messages).
    pgo_config_src = os.path.join(pkg_pgo, "config", "pgo.yaml")
    with open(pgo_config_src, "r") as f:
        pgo_config = yaml.safe_load(f)
    pgo_config["cloud_topic"] = f"/{robot_id}/fastlio2/body_cloud"
    pgo_config["odom_topic"] = f"/{robot_id}/fastlio2/lio_odom"
    pgo_config["local_frame"] = f"{robot_id}/lio_odom"
    pgo_generated = tempfile.NamedTemporaryFile(
        mode="w", prefix=f"pgo_{robot_id}_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(pgo_config, pgo_generated)
    pgo_generated.close()
    logger.info(f"generated pgo config: {pgo_generated.name}")

    return [
        launch_ros.actions.Node(
            package="fastlio2",
            namespace=f"{robot_id}/fastlio2",
            executable="lio_node",
            name="lio_node",
            output="screen",
            parameters=[{"config_path": lio_generated.name}],
        ),
        launch_ros.actions.Node(
            package="pgo",
            namespace=f"{robot_id}/pgo",
            executable="pgo_node",
            name="pgo_node",
            output="screen",
            parameters=[{"config_path": pgo_generated.name}],
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
                description="Launch RViz to watch mapping + loop closure",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
