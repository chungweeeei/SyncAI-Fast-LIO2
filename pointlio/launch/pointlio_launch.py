import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("pointlio"), "rviz", "pointlio.rviz"]
    )

    config_path = PathJoinSubstitution(
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
                    {"config_path": config_path.perform(launch.LaunchContext())}
                ],
            ),
            # launch_ros.actions.Node(
            #     package="rviz2",
            #     namespace="pointlio",
            #     executable="rviz2",
            #     name="rviz2",
            #     output="screen",
            #     arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            # ),
        ]
    )
