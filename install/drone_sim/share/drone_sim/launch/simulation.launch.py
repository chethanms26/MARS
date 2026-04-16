from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='drone_sim',
            executable='mavros_control'
        ),

        Node(
            package='drone_sim',
            executable='detection_node'
        ),

    ])
