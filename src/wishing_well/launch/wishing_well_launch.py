from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wishing_well',
            executable='wish_extraction_node',
            name='wish_extraction_node',
            output='screen'
        ),
        Node(
            package='wishing_well',
            executable='prompt_generation_node',
            name='prompt_generation_node',
            output='screen'
        ),
        Node(
            package='wishing_well',
            executable='image_generation_node',
            name='image_generation_node',
            output='screen'
        ),
        Node(
            package='wishing_well',
            executable='image_display_node',
            name='image_display_node',
            output='screen'
        )
    ])