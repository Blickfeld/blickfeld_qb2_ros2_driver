import sys
import os

import launch
import launch.actions
from launch import LaunchDescription
from launch import LaunchService
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    container = ComposableNodeContainer(
        name="blickfeld_qb2_component",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="blickfeld_qb2_ros2_driver",
                plugin="blickfeld::ros_interop::Qb2Driver",
                name="blickfeld_qb2_driver",
                parameters=[
                    {
                        "fqdn": "qb2",
                        "frame_id": "lidar",
                        "use_lidar_timestamp": False,
                        "publish_intensity": True,
                        "publish_point_id": True,
                    }
                ],
            ),
        ],
        output="screen",
        on_exit=launch.actions.Shutdown(),
    )
    return LaunchDescription([container])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == "__main__":
    main()
