from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_control',
            executable='my_robot_control_lidar_exec',
            name='robot_control_lidar',
            output='screen'
        )
    ])
