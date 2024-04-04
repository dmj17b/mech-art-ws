from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='big_brother',
            executable='camera_feed_node',
            name='camera_feed_node',
            output='screen'
        ),
        Node(
            package='big_brother',
            executable='web_image_node',
            name='web_image_node',
            output='screen',
            parameters=[
                {'image_topic': 'camera_feed'},
                {'port': 5001},
                {'update_rate': 30.0}
            ]
        ),
    ])