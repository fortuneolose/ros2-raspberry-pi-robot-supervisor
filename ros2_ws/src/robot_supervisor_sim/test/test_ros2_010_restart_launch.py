"""Launch test proving supervisor-process restart returns to safe startup."""

from __future__ import annotations

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from robot_supervisor_interfaces.msg import ActuatorCommand


@pytest.mark.launch_test
def generate_test_description():
    simulator = launch_ros.actions.Node(
        package='robot_supervisor_sim',
        executable='simulator_node',
        name='simulator_node',
        output='screen',
        parameters=[
            {
                'synthetic_configuration': True,
                'tick_period_s': 0.005,
                'auto_publish_command': True,
            }
        ],
    )
    supervisor = launch_ros.actions.Node(
        package='robot_supervisor',
        executable='supervisor_node',
        name='supervisor_node',
        output='screen',
        respawn=True,
        respawn_delay=0.1,
        parameters=[
            {
                'synthetic_configuration': True,
                'tick_period_s': 0.02,
                'synthetic_exit_after_ticks': 50,
            }
        ],
    )
    return launch.LaunchDescription(
        [simulator, supervisor, launch_testing.actions.ReadyToTest()]
    )


class TestSupervisorRestart(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node('ros2_010_restart_test')
        cls.actuators: list[ActuatorCommand] = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.node.create_subscription(
            ActuatorCommand,
            '/robot_supervisor/actuator_command',
            cls.actuators.append,
            qos,
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_restart_and_exit_outputs_are_safe(self) -> None:
        deadline = time.monotonic() + 8.0
        restart_messages: list[ActuatorCommand] = []
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            shutdown = [
                item
                for item in self.actuators
                if item.reason == 'synthetic_test_exit'
            ]
            restart_messages = [
                current
                for previous, current in zip(
                    self.actuators, self.actuators[1:]
                )
                if current.sample_index < previous.sample_index
            ]
            if restart_messages and shutdown:
                break
        else:
            self.fail('supervisor did not exit and respawn in time')

        for message in restart_messages + shutdown:
            self.assertFalse(message.relay_enable)
            self.assertEqual(message.motor_voltage_v, 0.0)
        self.assertTrue(restart_messages)
