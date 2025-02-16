from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='listen_node',
            executable='listener_node',
            name='listener_node',
            output='screen'
        ),
        
        Node(
            package='think_node',
            executable='think_node',
            name='think_node',
            output='screen'
        ),
        
        Node(
            package='speak_node',
            executable='speak_node',
            name='speak_node',
            output='screen'
        )
    ])
