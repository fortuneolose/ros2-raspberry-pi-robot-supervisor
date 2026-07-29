"""Synthetic ROS2-010 input producer, fault injector, and output consumer."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from robot_supervisor_interfaces.msg import (
    ActuatorCommand,
    EncoderTelemetry,
    SafetyInput,
    SupervisorCommand,
)
from robot_supervisor_interfaces.srv import SetFaultInjection


SUPPORTED_FAULTS = {
    'software_estop',
    'heartbeat_loss',
    'encoder_suppressed',
    'encoder_duplicate',
    'encoder_out_of_order',
    'encoder_failed',
    'encoder_invalid',
    'encoder_nonfinite',
    'relay_feedback_failed',
    'undervoltage',
    'safety_suppressed',
    'safety_duplicate',
    'safety_out_of_order',
}


def _volatile_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _actuator_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class SyntheticSimulatorNode(Node):
    """No-GPIO synthetic interface fixture for multi-node ROS tests."""

    def __init__(self) -> None:
        super().__init__('simulator_node')
        self.declare_parameter('synthetic_configuration', False)
        self.declare_parameter('tick_period_s', 0.01)
        self.declare_parameter('topic_prefix', '/robot_supervisor')
        self.declare_parameter('nominal_supply_voltage_v', 6.0)
        self.declare_parameter('undervoltage_voltage_v', 4.4)
        self.declare_parameter('auto_publish_command', True)
        self.declare_parameter('auto_arm_request', False)
        self.declare_parameter('auto_run_request', False)
        self.declare_parameter('auto_requested_motor_voltage_v', 0.0)
        if not bool(
            self.get_parameter('synthetic_configuration').value
        ):
            raise ValueError(
                'robot_supervisor_sim requires synthetic_configuration=true'
            )
        self._period = float(self.get_parameter('tick_period_s').value)
        self._nominal_supply = float(
            self.get_parameter('nominal_supply_voltage_v').value
        )
        self._undervoltage = float(
            self.get_parameter('undervoltage_voltage_v').value
        )
        if (
            not math.isfinite(self._period)
            or self._period <= 0.0
            or not math.isfinite(self._nominal_supply)
            or not math.isfinite(self._undervoltage)
        ):
            raise ValueError('synthetic simulator numeric parameters invalid')

        prefix = str(self.get_parameter('topic_prefix').value).strip()
        if not prefix.startswith('/'):
            prefix = '/' + prefix
        prefix = prefix.rstrip('/')
        qos = _volatile_qos()
        self._encoder_pub = self.create_publisher(
            EncoderTelemetry, prefix + '/encoder', qos
        )
        self._safety_pub = self.create_publisher(
            SafetyInput, prefix + '/safety_input', qos
        )
        self._command_pub = self.create_publisher(
            SupervisorCommand, prefix + '/command', qos
        )
        self.create_subscription(
            ActuatorCommand,
            prefix + '/actuator_command',
            self._on_actuator,
            _actuator_qos(),
        )
        self._fault_service = self.create_service(
            SetFaultInjection,
            prefix + '/set_fault_injection',
            self._on_set_fault,
        )

        self._encoder_sequence = 0
        self._safety_sequence = 0
        self._command_sequence = 0
        self._position_rad = 0.0
        self._actuator_relay_enable = False
        self._actuator_motor_voltage_v = 0.0
        self._active_faults: dict[str, int | None] = {}
        self._timer = self.create_timer(self._period, self._on_tick)
        self.get_logger().warning(
            'Synthetic simulator only: no GPIO, electrical relay, physical '
            'supply, or physical E-stop is represented'
        )

    def _active(self, fault: str) -> bool:
        return fault in self._active_faults

    def _on_set_fault(
        self,
        request: SetFaultInjection.Request,
        response: SetFaultInjection.Response,
    ) -> SetFaultInjection.Response:
        fault = request.fault.strip()
        if fault not in SUPPORTED_FAULTS:
            response.accepted = False
            response.reason = (
                'unsupported synthetic fault; supported='
                + ','.join(sorted(SUPPORTED_FAULTS))
            )
            return response
        if request.active:
            self._active_faults[fault] = (
                None
                if request.duration_samples == 0
                else int(request.duration_samples)
            )
            response.reason = 'synthetic fault enabled'
        else:
            self._active_faults.pop(fault, None)
            response.reason = 'synthetic fault cleared'
        response.accepted = True
        return response

    def _on_actuator(self, message: ActuatorCommand) -> None:
        self._actuator_relay_enable = bool(message.relay_enable)
        self._actuator_motor_voltage_v = (
            float(message.motor_voltage_v)
            if message.relay_enable
            else 0.0
        )

    def _publish_encoder(self) -> None:
        if self._active('encoder_suppressed'):
            return
        message = EncoderTelemetry()
        message.stamp = self.get_clock().now().to_msg()
        if self._active('encoder_duplicate'):
            message.sequence = self._encoder_sequence
        elif self._active('encoder_out_of_order'):
            message.sequence = max(0, self._encoder_sequence - 1)
        else:
            self._encoder_sequence += 1
            message.sequence = self._encoder_sequence
        message.position_rad = (
            math.nan
            if self._active('encoder_nonfinite')
            else self._position_rad
        )
        message.healthy = not self._active('encoder_failed')
        message.valid = not self._active('encoder_invalid')
        self._encoder_pub.publish(message)

    def _publish_safety(self) -> None:
        if self._active('safety_suppressed'):
            return
        message = SafetyInput()
        message.stamp = self.get_clock().now().to_msg()
        if self._active('safety_duplicate'):
            message.sequence = self._safety_sequence
        elif self._active('safety_out_of_order'):
            message.sequence = max(0, self._safety_sequence - 1)
        else:
            self._safety_sequence += 1
            message.sequence = self._safety_sequence
        message.supply_voltage_v = (
            self._undervoltage
            if self._active('undervoltage')
            else self._nominal_supply
        )
        message.software_estop_active = self._active('software_estop')
        message.watchdog_heartbeat = not self._active('heartbeat_loss')
        message.relay_feedback_enabled = (
            self._actuator_relay_enable
            and not self._active('relay_feedback_failed')
        )
        message.relay_feedback_healthy = not self._active(
            'relay_feedback_failed'
        )
        self._safety_pub.publish(message)

    def _publish_command(self) -> None:
        if not bool(
            self.get_parameter('auto_publish_command').value
        ):
            return
        self._command_sequence += 1
        message = SupervisorCommand()
        message.stamp = self.get_clock().now().to_msg()
        message.sequence = self._command_sequence
        message.arm_request = bool(
            self.get_parameter('auto_arm_request').value
        )
        message.run_request = bool(
            self.get_parameter('auto_run_request').value
        )
        message.shutdown_request = False
        message.requested_motor_voltage_v = float(
            self.get_parameter(
                'auto_requested_motor_voltage_v'
            ).value
        )
        self._command_pub.publish(message)

    def _advance_fault_durations(self) -> None:
        expired: list[str] = []
        for fault, remaining in self._active_faults.items():
            if remaining is None:
                continue
            next_remaining = remaining - 1
            if next_remaining <= 0:
                expired.append(fault)
            else:
                self._active_faults[fault] = next_remaining
        for fault in expired:
            self._active_faults.pop(fault, None)

    def _on_tick(self) -> None:
        self._position_rad += (
            self._actuator_motor_voltage_v * self._period * 0.01
        )
        self._publish_encoder()
        self._publish_safety()
        self._publish_command()
        self._advance_fault_durations()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SyntheticSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
