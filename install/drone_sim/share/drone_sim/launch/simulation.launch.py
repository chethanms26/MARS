from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to your world file — update this to match where you put disaster.world
    world_file = os.path.join(
        os.environ.get('HOME', '/root'),
        'drone_ws', 'src', 'drone_sim', 'worlds', 'disaster.world'
    )

    return LaunchDescription([

        # 1. Launch Gazebo with the disaster world
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', world_file,
                '-s', 'libgazebo_ros_factory.so',
                '-s', 'libgazebo_ros_init.so'
            ],
            output='screen'
        ),

        # 2. Patrol node — handles arming, OFFBOARD mode, and grid waypoints
        Node(
            package='drone_sim',
            executable='patrol_node',
            output='screen'
        ),

        # 3. Detection node — subscribes to camera, detects red victims
        Node(
            package='drone_sim',
            executable='detection_node',
            output='screen'
        ),

    ])
