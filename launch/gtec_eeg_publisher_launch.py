
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gtec_eeg_publisher',
            executable='gtec_eeg_publisher',
            name='gtec_eeg_publisher',
            output='screen'
        ),
    ])
