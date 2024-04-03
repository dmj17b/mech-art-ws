from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='big_brother',
            executable='camera_feed_node',
            name='camera_feed_node',
            output='screen'
        )
    ])