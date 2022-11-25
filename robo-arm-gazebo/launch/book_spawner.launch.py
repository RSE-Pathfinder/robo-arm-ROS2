from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robo-arm-gazebo',
            executable='book_spawner'
        )
    ])