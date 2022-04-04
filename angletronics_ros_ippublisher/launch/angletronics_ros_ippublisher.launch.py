from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='angletronics_ros_ippublisher',
            executable='simple_angletronics_ros_ippublisher',
            output='screen'),
    ])