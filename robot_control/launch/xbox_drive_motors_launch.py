from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='motors_drive',
            name='motors_drive'
        ),
        Node(
            package='xbox_controller',
            executable='controller_to_twist',
            name='controller_to_twist'
        ),
        Node(
            package='xbox_controller',
            executable='controller_input_mapper',
            name='controller_input_mapper'
        )
    ])