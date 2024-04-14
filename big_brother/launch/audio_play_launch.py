from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wishing_well',
            executable='audio_play_node',
            name='audio_play_node',
            output='screen'
        ),
        Node(
            package='big_brother',
            executable='osc_client_node',
            name='osc_client_node',
            output='screen'
        )
    ])