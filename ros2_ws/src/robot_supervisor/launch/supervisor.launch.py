"""Launch only the ROS2-010 supervisor adapter with synthetic parameters."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('robot_supervisor'))
    return LaunchDescription(
        [
            Node(
                package='robot_supervisor',
                executable='supervisor_node',
                name='supervisor_node',
                output='screen',
                parameters=[
                    str(share / 'config' / 'ros2_010.synthetic.yaml')
                ],
            )
        ]
    )
