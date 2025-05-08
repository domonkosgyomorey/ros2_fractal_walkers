from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='hilbert_walker',
            executable='walker',
            name='fractal_walker',
            output='screen'
        )
    ])
