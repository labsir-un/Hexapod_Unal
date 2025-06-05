from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dynamixel_node',
            executable='dynamixel_node',
            name='dynamixel',
            output='screen',
            arguments=['--ros-args', '--log-level', 'fatal']
        ),
        Node(
            package='transformation_node',
            executable='transformation_node',
            name='transformation',
            output='screen',
            arguments=['--ros-args', '--log-level', 'fatal']
        ),
        Node(
            package='cinematica_node',
            executable='cinematica_node',
            name='cinematica',
            output='screen',
            arguments=['--ros-args', '--log-level', 'fatal']
        )
    ])