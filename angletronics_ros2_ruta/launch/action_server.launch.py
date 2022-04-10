from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='angletronics_ros2_ruta',
            executable='action_server',
            output='screen'
        ),
    ])