import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_cfg = PathJoinSubstitution([FindPackageShare("pgo"), "rviz", "pgo.rviz"])
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo.yaml"]
    )

    point_lio_config_path = PathJoinSubstitution(
        [FindPackageShare("pointlio"), "config", "pointlio.yaml"]
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="pointlio",
                namespace="pointlio",
                executable="pointlio_node",
                name="pointlio_node",
                output="screen",
                parameters=[
                    {
                        "config_path": point_lio_config_path.perform(
                            launch.LaunchContext()
                        )
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="pgo",
                namespace="pgo",
                executable="pgo_node",
                name="pgo_node",
                output="screen",
                parameters=[
                    {"config_path": pgo_config_path.perform(launch.LaunchContext())}
                ],
            ),
        ]
    )
