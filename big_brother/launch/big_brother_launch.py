from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='big_brother',
            executable='person_locator_node',
            name='person_locator_node',
            output='screen'
        ),
        Node(
            package='big_brother',
            executable='heat_map_node',
            name='heat_map_node',
            output='screen'
        ),
        Node(
            package='big_brother',
            executable='web_image_node',
            name='web_image_node',
            output='screen',
            parameters=[
                {'image_topic': '/video_source/raw'},
                {'port': 5001},
                {'update_rate': 30.0}
            ]
        ),
        Node(
            package='big_brother',
            executable='web_image_node',
            name='web_image_node',
            output='screen',
            parameters=[
                {'image_topic': '/detectnet/overlay'},
                {'port': 5002},
                {'update_rate': 30.0}
            ]
        ),
        Node(
            package='big_brother',
            executable='web_image_node',
            name='web_image_node',
            output='screen',
            parameters=[
                {'image_topic': 'heat_map'},
                {'port': 5003},
                {'update_rate': 30.0}
            ]
        ),
        Node(
            package='big_brother',
            executable='person_counter_node',
            name='person_counter_node',
            output='screen'
        )
    ])