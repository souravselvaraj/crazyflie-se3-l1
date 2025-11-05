from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crazyflie_controller',
            executable='hover_controller.py',
            output='screen',
            parameters=[{
                'target_x': 0.0,
                'target_y': 0.0,
                'target_z': 1.0,

            }]
        )
    ])
