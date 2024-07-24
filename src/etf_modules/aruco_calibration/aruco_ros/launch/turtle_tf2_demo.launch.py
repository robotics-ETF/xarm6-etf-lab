from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_ros',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
        ),
    ])