import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("pcd_publisher"), "rviz", "pcd_publisher.rviz"]
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "pcd_path",
                default_value="",
                description="Absolute path to the .pcd file to publish.",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="map",
                description="TF frame the cloud is published in.",
            ),
            DeclareLaunchArgument(
                "topic",
                default_value="/pcd_map",
                description="Topic to publish the PointCloud2 on.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Whether to launch RViz2.",
            ),
            launch_ros.actions.Node(
                package="pcd_publisher",
                executable="pcd_publisher_node",
                name="pcd_publisher",
                output="screen",
                parameters=[
                    {
                        "pcd_path": LaunchConfiguration("pcd_path"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "topic": LaunchConfiguration("topic"),
                    }
                ],
            ),
            # Publish an identity TF so the cloud's frame exists in the TF tree.
            # Without this, RViz reports "Fixed Frame [map] does not exist" and
            # refuses to render the cloud even though its frame_id matches.
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="pcd_static_tf",
                output="screen",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--yaw", "0", "--pitch", "0", "--roll", "0",
                    "--frame-id", LaunchConfiguration("frame_id"),
                    "--child-frame-id", "pcd_link",
                ],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            ),
        ]
    )
