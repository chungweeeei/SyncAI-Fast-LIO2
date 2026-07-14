# Isaac Sim (robot06) variant: lio_node uses lio_isaac.yaml (PointCloud2 input,
# imu_acc_scale 1.0, frames lio_odom/lio_body). localizer's local_frame is taken
# from the first odom message's frame_id, so it follows lio_odom automatically
# and broadcasts map -> lio_odom once relocalized.
#
# robot_id is read from the system config INI at launch time (same convention
# as the syncai_* launches) and is used as the namespace for all nodes:
#   /<robot_id>/fastlio2/...  (body_cloud, lio_odom, ...)
#   /<robot_id>/localizer/... (relocalize, relocalize_check, map_cloud)
# and as the prefix for the LIO TF frames:
#   map -> <robot_id>/lio_odom -> <robot_id>/lio_body
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

logger = launch_logging.get_logger("localizer_isaac.launch")


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

    use_rviz = LaunchConfiguration("rviz")

    pkg_localizer = FindPackageShare("localizer").perform(context)
    pkg_fastlio2 = FindPackageShare("fastlio2").perform(context)

    rviz_cfg = os.path.join(pkg_localizer, "rviz", "localizer_isaac.rviz")

    # Both nodes read topics/frames from their own YAMLs as absolute names
    # (not affected by ROS namespaces), so rewrite them with the robot_id
    # prefix into generated files under /tmp.

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

    # localizer: input topics follow lio_node's namespace (it publishes
    # body_cloud / lio_odom relative). local_frame needs no rewrite — the
    # node adopts the frame_id of the first odom message automatically.
    localizer_config_src = os.path.join(pkg_localizer, "config", "localizer.yaml")
    with open(localizer_config_src, "r") as f:
        localizer_config = yaml.safe_load(f)
    localizer_config["cloud_topic"] = f"/{robot_id}/fastlio2/body_cloud"
    localizer_config["odom_topic"] = f"/{robot_id}/fastlio2/lio_odom"
    generated = tempfile.NamedTemporaryFile(
        mode="w",
        prefix=f"localizer_{robot_id}_",
        suffix=".yaml",
        delete=False,
    )
    yaml.safe_dump(localizer_config, generated)
    generated.close()
    logger.info(f"generated localizer config: {generated.name}")

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
            package="localizer",
            namespace=f"{robot_id}/localizer",
            executable="localizer_node",
            name="localizer_node",
            output="screen",
            parameters=[{"config_path": generated.name}],
        ),
        launch_ros.actions.Node(
            package="rviz2",
            namespace=f"{robot_id}/localizer",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_cfg],
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
                description="Launch RViz to watch localization",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
