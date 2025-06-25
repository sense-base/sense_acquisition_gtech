
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sense_gtec_eeg',
            executable='sense_eeg',
            name='gtec_eeg_publisher',
            output='screen'
        ),
    ])
