#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


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
                        "point_cloud_topic": "/bf/points_raw",
                        "use_measurement_timestamp": False,
                        "publish_intensity": True,
                        "publish_point_id": True,
                    }
                ],
            ),
        ],
        output="screen",
    )
    return LaunchDescription([container])
