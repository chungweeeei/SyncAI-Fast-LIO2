# Real Livox sensor variant of the localizer launch (paired with
# pointlio_launch.py).
#
# robot_id is read from the system config INI at launch time (the workspace-wide
# convention) and is used as the namespace for both nodes:
#   /<robot_id>/pointlio/...   (body_cloud, lio_odom, ...)
#   /<robot_id>/localizer/...  (relocalize, relocalize_check, map_cloud)
#
# The two nodes configure themselves differently:
#   * localizer takes standard ROS 2 parameters — config/localizer.yaml is a
#     /**/localizer_node params file, and the robot_id-dependent values
#     (pointlio's topics, the map PCD path) are passed as a second parameters
#     dict that overrides the file. No generated files involved.
#   * pointlio still reads its own flat YAML through a config_path parameter
#     (absolute topic/frame names, unaffected by ROS namespaces), so the launch
#     keeps rewriting that one with the robot_id prefix into /tmp.
#
# The [map] pcd from the same INI is mandatory: the localizer loads it during
# construction, so a missing file means neither node starts. The optional
# [initial_pose] section (same one syncai_amcl reads) becomes the localizer's
# boot guess, so a robot standing at its known start pose localizes itself
# without a relocalize call.

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
    """[map] pcd from the system INI as an absolute path — the map the localizer
    loads at construction time. Empty string if not configured.

    INI 裡寫的是相對 workspace root 的路徑（processes 以 workspace root 為 cwd 的
    慣例），但這裡不用 cwd 解析：system.ini 就在 <workspace>/config/ 底下，直接
    用 INI 自己的位置回推 workspace root。在 workspace root 啟動時兩者同值，而
    launch 現在缺檔就整組不啟動——用 cwd 解析會讓「從別的目錄啟動」變成假的
    缺檔失敗。"""
    config = configparser.ConfigParser()
    if not config.read(config_path):
        return ""
    pcd = config.get("map", "pcd", fallback="").strip()
    if not pcd or os.path.isabs(pcd):
        return pcd
    workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(config_path)))
    return os.path.join(workspace_root, pcd)


def read_initial_pose(config_path: str):
    """Read the [initial_pose] section from the INI — the robot's known start
    pose in the map frame, same section syncai_amcl reads.

    Returns a dict of localizer parameter overrides, or None when the section is
    absent or malformed (the localizer then waits for a relocalize / initialpose
    instead of auto-localizing at boot).

    只送 x / y / yaw：localizer 的 z / roll / pitch 一律取自當下估計，INI 的 z
    對它沒有意義（雷達斜裝 + map 重力對齊 → map_T_body 恆帶 mount pitch，
    詳見 localizer_node.cpp 的 applyPlanarGuess）。AMCL 是平面 filter，
    所以那邊連 z 一起送。
    """
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


def generate_pointlio_config(robot_id: str) -> str:
    """Rewrite pointlio.yaml topics/frames with the robot_id prefix and
    return the path of the generated file."""
    pkg_pointlio = FindPackageShare("pointlio").find("pointlio")
    src = os.path.join(pkg_pointlio, "config", "pointlio.yaml")
    with open(src, "r") as f:
        cfg = yaml.safe_load(f)
    cfg["lidar_topic"] = f"/{robot_id}/livox/lidar"
    cfg["imu_topic"] = f"/{robot_id}/livox/imu"
    # Standalone frames — must stay in sync with pointlio_launch.py, which
    # carries the full rationale. Short version: naming the body frame
    # base_link gave that frame two TF parents (syncai_lio_bridge also
    # broadcasts odom -> base_link), so which chain a lookup resolved through
    # depended on the query time, and the backend's body_cloud silently went
    # through lio_bridge's 2D-projected chain instead of the 6DOF LIO one.
    cfg["world_frame"] = f"{robot_id}/pointlio_odom"
    cfg["body_frame"] = f"{robot_id}/pointlio_body"

    generated = tempfile.NamedTemporaryFile(
        mode="w", prefix=f"pointlio_{robot_id}_", suffix=".yaml", delete=False
    )

    yaml.safe_dump(cfg, generated)
    generated.close()
    logger.info(f"generated pointlio config: {generated.name}")
    return generated.name


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
    pointlio_config = generate_pointlio_config(robot_id)

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
