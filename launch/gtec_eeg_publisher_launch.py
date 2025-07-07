from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gtec_eeg_publisher'),
        'config',
        'gtec_publisher.yaml'
    )

    """Launch the gtec_eeg_publisher node."""
    return LaunchDescription([
        Node(
            package='gtec_eeg_publisher',
            executable='gtec_eeg_publisher',
            name='gtec_eeg_publisher',
            parameters=[config],
            output='screen'
        ),
    ])
