
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launches the gtec_eeg_publisher node."""
    return LaunchDescription([
        Node(
            package='gtec_eeg_publisher',
            executable='gtec_eeg_publisher',
            name='gtec_eeg_publisher',
            output='screen'
        ),
    ])
