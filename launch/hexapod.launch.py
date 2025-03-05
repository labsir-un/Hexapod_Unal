from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="gui_node",
            executable="gui_client",
            name="gui"
        ),
        Node(
            package="transformation_node",
            executable="transformation_node",
            name="transformacion"
        ),
        Node(
            package="cinematica_node",
            executable="cinematica_node",
            name="cinematica"
        )
    ])
