from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_tracking_project',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
        Node(
            package='robot_tracking_project',
            executable='target_publisher',
            name='target_publisher',
            output='screen'
        ),
    ])