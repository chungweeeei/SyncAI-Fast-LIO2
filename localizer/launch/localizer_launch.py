# Real Livox sensor variant of the localizer launch (paired with
# pointlio_launch.py).
#
# robot_id is read from the system config INI at launch time (same convention
# as localizer_isaac_launch.py) and is used as the namespace for both nodes:
#   /<robot_id>/pointlio/...   (body_cloud, lio_odom, ...)
#   /<robot_id>/localizer/...  (relocalize, relocalize_check, map_cloud)
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
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

DEFAULT_SYSTEM_INI = "config/system.ini"
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
    """[map] pcd from the system INI — the PCD the localizer should load when
    it receives an initialpose before any relocalize (e.g. right after a
    restart). Empty string if not configured; initialpose then requires a
    prior relocalize call."""
    config = configparser.ConfigParser()
    if not config.read(config_path):
        return ""
    return config.get("map", "pcd", fallback="").strip()


def generate_pointlio_config(robot_id: str) -> str:
    """Rewrite pointlio.yaml topics/frames with the robot_id prefix and
    return the path of the generated file."""
    pkg_pointlio = FindPackageShare("pointlio").find("pointlio")
    src = os.path.join(pkg_pointlio, "config", "pointlio.yaml")
    with open(src, "r") as f:
        cfg = yaml.safe_load(f)
    cfg["lidar_topic"] = f"/{robot_id}/livox/lidar"
    cfg["imu_topic"] = f"/{robot_id}/livox/imu"
    cfg["world_frame"] = f"{robot_id}/laser"
    cfg["body_frame"] = f"{robot_id}/base_link"
    generated = tempfile.NamedTemporaryFile(
        mode="w", prefix=f"pointlio_{robot_id}_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(cfg, generated)
    generated.close()
    logger.info(f"generated pointlio config: {generated.name}")
    return generated.name


def generate_localizer_config(robot_id: str, map_pcd: str) -> str:
    """Rewrite localizer.yaml input topics to follow pointlio's namespace and
    return the path of the generated file. local_frame needs no rewrite - the
    node adopts the frame_id of the first odom message automatically."""
    pkg_localizer = FindPackageShare("localizer").find("localizer")
    src = os.path.join(pkg_localizer, "config", "localizer.yaml")
    with open(src, "r") as f:
        cfg = yaml.safe_load(f)
    cfg["cloud_topic"] = f"/{robot_id}/pointlio/body_cloud"
    cfg["odom_topic"] = f"/{robot_id}/pointlio/lio_odom"
    if map_pcd:
        # INI 裡是相對 workspace root 的路徑（processes 以 workspace root 為
        # cwd 的慣例）；launch 也在 workspace root 跑，這裡轉絕對路徑，
        # 讓 node 不依賴自己的 cwd
        cfg["map_path"] = os.path.abspath(map_pcd)
    else:
        logger.warning(
            "No [map] pcd in the system INI; initialpose will only work "
            "after a relocalize call has loaded the map"
        )
    generated = tempfile.NamedTemporaryFile(
        mode="w", prefix=f"localizer_{robot_id}_", suffix=".yaml", delete=False
    )
    yaml.safe_dump(cfg, generated)
    generated.close()
    logger.info(f"generated localizer config: {generated.name}")
    return generated.name


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    pointlio_config = generate_pointlio_config(robot_id)
    localizer_config = generate_localizer_config(robot_id, read_map_pcd(config_path))

    return [
        launch_ros.actions.Node(
            package="pointlio",
            namespace=f"{robot_id}/pointlio",
            executable="pointlio_node",
            name="pointlio_node",
            output="screen",
            parameters=[{"config_path": pointlio_config}],
        ),
        launch_ros.actions.Node(
            package="localizer",
            namespace=f"{robot_id}/localizer",
            executable="localizer_node",
            name="localizer_node",
            output="screen",
            parameters=[{"config_path": localizer_config}],
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
