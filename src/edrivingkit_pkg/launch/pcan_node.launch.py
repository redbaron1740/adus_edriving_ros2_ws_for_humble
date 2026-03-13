from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='edrivingkit_pkg',
            executable='edrivingkit_pcan_node',
            name='edrivingkit_pcan_node',
            parameters=['../config/pcan_config.yaml']
        )
    ])