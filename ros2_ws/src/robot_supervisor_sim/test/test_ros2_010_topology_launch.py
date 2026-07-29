"""ROS 2 launch test for the complete synthetic ROS2-010 topology."""

from __future__ import annotations

import math
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from robot_supervisor_interfaces.msg import (
    ActuatorCommand,
    FaultTelemetry,
    SafetyStatus,
    SupervisorCommand,
)
from robot_supervisor_interfaces.srv import (
    ResetFault,
    SetFaultInjection,
)


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
                'auto_publish_command': False,
            }
        ],
    )
    supervisor = launch_ros.actions.Node(
        package='robot_supervisor',
        executable='supervisor_node',
        name='supervisor_node',
        output='screen',
        parameters=[
            {
                'synthetic_configuration': True,
                'tick_period_s': 0.02,
                'command_stale_samples': 3,
            }
        ],
    )
    return (
        launch.LaunchDescription(
            [simulator, supervisor, launch_testing.actions.ReadyToTest()]
        ),
        {'simulator': simulator, 'supervisor': supervisor},
    )


class TestRos2010Topology(unittest.TestCase):
    """Exercise faults and recovery through ROS topics and services."""

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node('ros2_010_launch_test')
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.command_pub = cls.node.create_publisher(
            SupervisorCommand, '/robot_supervisor/command', input_qos
        )
        cls.statuses: list[SafetyStatus] = []
        cls.actuators: list[ActuatorCommand] = []
        cls.faults: list[FaultTelemetry] = []
        cls.node.create_subscription(
            SafetyStatus,
            '/robot_supervisor/safety_status',
            cls.statuses.append,
            output_qos,
        )
        cls.node.create_subscription(
            ActuatorCommand,
            '/robot_supervisor/actuator_command',
            cls.actuators.append,
            output_qos,
        )
        cls.node.create_subscription(
            FaultTelemetry,
            '/robot_supervisor/faults',
            cls.faults.append,
            output_qos,
        )
        cls.reset_client = cls.node.create_client(
            ResetFault, '/robot_supervisor/reset_fault'
        )
        cls.injection_client = cls.node.create_client(
            SetFaultInjection,
            '/robot_supervisor/set_fault_injection',
        )
        cls.command_sequence = 0

    @classmethod
    def tearDownClass(cls) -> None:
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def publish_command(
        cls, arm: bool, run: bool, voltage: float
    ) -> None:
        cls.command_sequence += 1
        message = SupervisorCommand()
        message.stamp = cls.node.get_clock().now().to_msg()
        message.sequence = cls.command_sequence
        message.arm_request = arm
        message.run_request = run
        message.shutdown_request = False
        message.requested_motor_voltage_v = voltage
        cls.command_pub.publish(message)

    @classmethod
    def wait_for(
        cls,
        predicate,
        *,
        timeout: float = 5.0,
        command: tuple[bool, bool, float] | None = None,
    ):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if command is not None:
                cls.publish_command(*command)
            rclpy.spin_once(cls.node, timeout_sec=0.02)
            value = predicate()
            if value:
                return value
        raise AssertionError('timed out waiting for ROS2-010 condition')

    @classmethod
    def call_injection(cls, fault: str, active: bool) -> None:
        assert cls.injection_client.wait_for_service(timeout_sec=3.0)
        request = SetFaultInjection.Request()
        request.fault = fault
        request.active = active
        request.duration_samples = 0
        future = cls.injection_client.call_async(request)
        rclpy.spin_until_future_complete(
            cls.node, future, timeout_sec=3.0
        )
        assert future.done()
        response = future.result()
        assert response is not None and response.accepted

    @classmethod
    def call_reset(cls) -> ResetFault.Response:
        assert cls.reset_client.wait_for_service(timeout_sec=3.0)
        request = ResetFault.Request()
        request.requester = 'launch_test'
        future = cls.reset_client.call_async(request)
        rclpy.spin_until_future_complete(
            cls.node, future, timeout_sec=3.0
        )
        assert future.done()
        response = future.result()
        assert response is not None
        return response

    @classmethod
    def latest_state(cls) -> str:
        return cls.statuses[-1].state if cls.statuses else ''

    @classmethod
    def wait_running(cls, voltage: float = 2.0) -> None:
        cls.wait_for(
            lambda: cls.latest_state() == 'READY',
            command=(False, False, 0.0),
        )
        ready_sample = cls.statuses[-1].sample_index
        cls.wait_for(
            lambda: (
                cls.latest_state() == 'READY'
                and cls.statuses[-1].sample_index > ready_sample
            ),
            command=(False, False, 0.0),
        )
        cls.wait_for(
            lambda: cls.latest_state() == 'RUNNING',
            command=(True, True, voltage),
        )

    @classmethod
    def wait_fault(cls, expected: str) -> None:
        def predicate():
            if not cls.faults or not cls.actuators:
                return False
            fault = cls.faults[-1]
            actuator = cls.actuators[-1]
            return (
                expected in fault.latched_faults
                and not actuator.relay_enable
                and actuator.motor_voltage_v == 0.0
            )

        cls.wait_for(
            predicate, command=(True, True, 2.0), timeout=6.0
        )

    @classmethod
    def recover(cls, faults_to_clear: tuple[str, ...] = ()) -> None:
        for fault in faults_to_clear:
            cls.call_injection(fault, False)

        def raw_sources_clear():
            return (
                bool(cls.faults)
                and not cls.faults[-1].detected_faults
                and not cls.faults[-1].active_raw_fault_sources
            )

        cls.wait_for(
            raw_sources_clear, command=(False, False, 0.0), timeout=5.0
        )
        cls.publish_command(False, False, 0.0)
        time.sleep(0.03)
        response = cls.call_reset()
        assert response.accepted, response.reason
        cls.wait_for(
            lambda: cls.latest_state() == 'READY',
            command=(False, False, 0.0),
            timeout=5.0,
        )
        cls.wait_running()

    def test_complete_multi_node_fault_and_recovery_behaviour(self) -> None:
        self.wait_running()
        self.assertTrue(self.actuators[-1].relay_enable)
        self.assertNotEqual(self.actuators[-1].motor_voltage_v, 0.0)

        self.wait_for(
            lambda: bool(self.actuators)
            and self.actuators[-1].saturated
            and self.actuators[-1].motor_voltage_v == 6.0,
            command=(True, True, 7.0),
        )
        self.assertEqual(self.latest_state(), 'RUNNING')

        self.call_injection('software_estop', True)
        self.wait_fault('EMERGENCY_STOP')
        unsafe = self.call_reset()
        self.assertFalse(unsafe.accepted)
        self.assertIn('arm_or_run_active', unsafe.reason)
        self.recover(('software_estop',))

        self.wait_for(
            lambda: bool(self.faults)
            and 'INVALID_COMMAND' in self.faults[-1].latched_faults,
            command=(True, True, math.nan),
        )
        self.assertFalse(self.actuators[-1].relay_enable)
        self.assertEqual(self.actuators[-1].motor_voltage_v, 0.0)
        self.recover()

        for injection, expected in (
            ('heartbeat_loss', 'WATCHDOG_TIMEOUT'),
            ('encoder_duplicate', 'ENCODER_STALE'),
            ('encoder_out_of_order', 'ENCODER_INVALID'),
            ('encoder_failed', 'ENCODER_FAILURE'),
            ('relay_feedback_failed', 'RELAY_FEEDBACK_FAILURE'),
            ('undervoltage', 'UNDERVOLTAGE'),
        ):
            with self.subTest(injection=injection):
                self.call_injection(injection, True)
                self.wait_fault(expected)
                self.recover((injection,))

        self.call_injection('encoder_suppressed', True)
        self.call_injection('safety_suppressed', True)
        self.wait_for(
            lambda: (
                bool(self.faults)
                and (
                    'ENCODER_STALE' in self.faults[-1].latched_faults
                    or 'WATCHDOG_TIMEOUT'
                    in self.faults[-1].latched_faults
                )
                and bool(self.actuators)
                and not self.actuators[-1].relay_enable
                and self.actuators[-1].motor_voltage_v == 0.0
            ),
            command=(True, True, 2.0),
            timeout=6.0,
        )


@launch_testing.post_shutdown_test()
class TestRos2010TopologyShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info) -> None:
        launch_testing.asserts.assertExitCodes(proc_info)
