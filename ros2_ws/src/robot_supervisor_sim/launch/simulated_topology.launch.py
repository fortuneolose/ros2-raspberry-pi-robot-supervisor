"""Launch the complete explicitly synthetic ROS2-010 topology."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    supervisor_share = Path(
        get_package_share_directory('robot_supervisor')
    )
    simulator_share = Path(
        get_package_share_directory('robot_supervisor_sim')
    )
    return LaunchDescription(
        [
            Node(
                package='robot_supervisor_sim',
                executable='simulator_node',
                name='simulator_node',
                output='screen',
                parameters=[
                    str(
                        simulator_share
                        / 'config'
                        / 'ros2_010_sim.synthetic.yaml'
                    )
                ],
            ),
            Node(
                package='robot_supervisor',
                executable='supervisor_node',
                name='supervisor_node',
                output='screen',
                parameters=[
                    str(
                        supervisor_share
                        / 'config'
                        / 'ros2_010.synthetic.yaml'
                    )
                ],
            ),
        ]
    )
