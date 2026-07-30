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
# /tmp rewrite of pointlio.yaml, which是 pointlio_node 還自己用 yaml-cpp 解析
# 設定檔的年代留下來的。那個 node 早就改成純 ROS parameters + 相對 topic 名稱
# ("lidar" / "imu") 加 launch remapping，於是這裡的 config_path 變成一個沒人讀的
# 參數：pointlio 全套調參退回 struct defaults，訂閱退回相對名稱解出來的
# /<robot_id>/pointlio/{lidar,imu}（沒有任何 publisher），LIO 收不到一筆資料，
# 不發 odom / body_cloud / TF，localizer 也就永遠沒有輸入可以定位。
# 用 include 而不是複製一份 Node 定義，就是為了不再有第二份會走鐘的定義。
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

    # 地圖是硬需求：localizer 在建構期就 loadMap（見 localizer_node.cpp 的
    # loadInitialMap），沒有地圖整條 3D 定位鏈都做不了事。所以缺檔就一個 node
    # 都不啟動（等同回傳空的 LaunchDescription）——比讓 localizer 起來、
    # 之後每次 relocalize / initialpose 都失敗要好判斷。
    # 檢查放在這裡而不是 generate_launch_description()，是因為 INI 路徑來自
    # system_config launch argument，只有進到 context 才解得出值。
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
