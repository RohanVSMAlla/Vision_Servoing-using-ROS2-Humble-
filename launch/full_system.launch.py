from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        
        # Turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle_sim'
        ),

        # Camera node
        Node(
            package='vision_servoing',
            executable='camera_node',
            name='camera_node'
        ),

        # Vision tracker node
        Node(
            package='vision_servoing',
            executable='vision_tracker_node',
            name='vision_tracker'
        ),

        # Controller node (C++)
        Node(
            package='vision_servoing',
            executable='controller_node',
            name='controller'
        ),
    ])
