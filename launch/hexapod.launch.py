from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cinematica_node",
            executable="cinematica_node",
            name="cinematica"
        ),
        Node(
            package="transformation_node",
            executable="transformation_node",
            name="transformation"
        ),
        Node(
            package="dynamixel_node",
            executable="dynamixel_node",
            name="dynamixel"
        )
    ])
