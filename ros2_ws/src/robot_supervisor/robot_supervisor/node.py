"""ROS 2 node adapting typed messages to the authoritative SIM-010 bench."""

from __future__ import annotations

import math
from pathlib import Path
import signal
from typing import Any

import models
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.signals import SignalHandlerOptions
from robot_supervisor.core import (
    actuator_mapping,
    CommandFrame,
    EncoderFrame,
    fault_telemetry_mapping,
    safe_actuator_mapping,
    safety_status_mapping,
    SafetyFrame,
    supervisor_telemetry_mapping,
    SupervisorAdapter,
)
from robot_supervisor_interfaces.msg import (
    ActuatorCommand,
    EncoderTelemetry,
    FaultTelemetry,
    SafetyInput,
    SafetyStatus,
    SupervisorCommand,
    SupervisorTelemetry,
)
from robot_supervisor_interfaces.srv import ResetFault


def _input_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _safe_output_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class SupervisorNode(Node):
    """Periodic ROS middleware shell; all state transitions remain SIM-010."""

    def __init__(self) -> None:
        super().__init__('supervisor_node')
        self.declare_parameter('synthetic_configuration', False)
        self.declare_parameter('tick_period_s', 0.02)
        self.declare_parameter('command_stale_samples', 3)
        self.declare_parameter('topic_prefix', '/robot_supervisor')
        self.declare_parameter('sim_config_path', '')
        self.declare_parameter('motor_config_path', '')
        self.declare_parameter('synthetic_exit_after_ticks', 0)

        if not bool(
            self.get_parameter('synthetic_configuration').value
        ):
            raise ValueError(
                'ROS2-010 requires synthetic_configuration=true'
            )
        tick_period = float(self.get_parameter('tick_period_s').value)
        if not math.isfinite(tick_period) or tick_period <= 0.0:
            raise ValueError('tick_period_s must be finite and above zero')
        command_stale_samples = int(
            self.get_parameter('command_stale_samples').value
        )
        exit_after_ticks = int(
            self.get_parameter('synthetic_exit_after_ticks').value
        )
        if exit_after_ticks < 0:
            raise ValueError('synthetic_exit_after_ticks cannot be negative')
        self._synthetic_exit_after_ticks = exit_after_ticks
        self._tick_count = 0
        self._synthetic_exit_requested = False
        self._safe_shutdown_published = False

        model_parameters = (
            Path(models.__file__).resolve().parent / 'parameters'
        )
        sim_path_value = str(
            self.get_parameter('sim_config_path').value
        ).strip()
        motor_path_value = str(
            self.get_parameter('motor_config_path').value
        ).strip()
        sim_path = (
            Path(sim_path_value)
            if sim_path_value
            else model_parameters / 'synthetic_sim_010.json'
        )
        motor_path = (
            Path(motor_path_value)
            if motor_path_value
            else model_parameters / 'synthetic_motor.json'
        )
        self._adapter = SupervisorAdapter.from_files(
            sim_path,
            motor_path,
            command_stale_samples=command_stale_samples,
        )

        prefix = str(self.get_parameter('topic_prefix').value).strip()
        if not prefix.startswith('/'):
            prefix = '/' + prefix
        prefix = prefix.rstrip('/')
        input_qos = _input_qos()
        output_qos = _safe_output_qos()

        self._actuator_pub = self.create_publisher(
            ActuatorCommand, prefix + '/actuator_command', output_qos
        )
        self._safety_status_pub = self.create_publisher(
            SafetyStatus, prefix + '/safety_status', output_qos
        )
        self._supervisor_pub = self.create_publisher(
            SupervisorTelemetry, prefix + '/telemetry', output_qos
        )
        self._fault_pub = self.create_publisher(
            FaultTelemetry, prefix + '/faults', output_qos
        )
        self.create_subscription(
            SupervisorCommand,
            prefix + '/command',
            self._on_command,
            input_qos,
        )
        self.create_subscription(
            EncoderTelemetry,
            prefix + '/encoder',
            self._on_encoder,
            input_qos,
        )
        self.create_subscription(
            SafetyInput,
            prefix + '/safety_input',
            self._on_safety,
            input_qos,
        )
        self._reset_service = self.create_service(
            ResetFault, prefix + '/reset_fault', self._on_reset
        )

        self._publish_safe_output('node_startup')
        self._timer = self.create_timer(tick_period, self._on_tick)
        self.get_logger().warning(
            'ROS2-010 is an explicitly synthetic middleware fixture; '
            'its software E-stop topic is not a physical safety function'
        )

    @property
    def adapter(self) -> SupervisorAdapter:
        return self._adapter

    def _on_command(self, message: SupervisorCommand) -> None:
        disposition = self._adapter.ingest_command(
            CommandFrame(
                sequence=message.sequence,
                arm_request=message.arm_request,
                run_request=message.run_request,
                shutdown_request=message.shutdown_request,
                requested_motor_voltage_v=(
                    message.requested_motor_voltage_v
                ),
            )
        )
        if disposition != 'ACCEPTED':
            self.get_logger().warning(
                f'command message disposition: {disposition}'
            )

    def _on_encoder(self, message: EncoderTelemetry) -> None:
        disposition = self._adapter.ingest_encoder(
            EncoderFrame(
                sequence=message.sequence,
                position_rad=message.position_rad,
                healthy=message.healthy,
                valid=message.valid,
            )
        )
        if disposition != 'ACCEPTED':
            self.get_logger().warning(
                f'encoder message disposition: {disposition}'
            )

    def _on_safety(self, message: SafetyInput) -> None:
        disposition = self._adapter.ingest_safety(
            SafetyFrame(
                sequence=message.sequence,
                supply_voltage_v=message.supply_voltage_v,
                software_estop_active=message.software_estop_active,
                watchdog_heartbeat=message.watchdog_heartbeat,
                relay_feedback_enabled=message.relay_feedback_enabled,
                relay_feedback_healthy=message.relay_feedback_healthy,
            )
        )
        if disposition != 'ACCEPTED':
            self.get_logger().warning(
                f'safety message disposition: {disposition}'
            )

    def _stamp_and_publish(
        self, publisher: Any, message: Any, values: dict[str, Any]
    ) -> None:
        message.stamp = self.get_clock().now().to_msg()
        for name, value in values.items():
            setattr(message, name, value)
        publisher.publish(message)

    def _publish_result(self, result: Any, *, reset_reason: str = '') -> None:
        self._stamp_and_publish(
            self._actuator_pub,
            ActuatorCommand(),
            actuator_mapping(result.record),
        )
        self._stamp_and_publish(
            self._safety_status_pub,
            SafetyStatus(),
            safety_status_mapping(result.record),
        )
        self._stamp_and_publish(
            self._supervisor_pub,
            SupervisorTelemetry(),
            supervisor_telemetry_mapping(result),
        )
        self._stamp_and_publish(
            self._fault_pub,
            FaultTelemetry(),
            fault_telemetry_mapping(result, reset_reason=reset_reason),
        )

    def _publish_safe_output(self, reason: str) -> None:
        terminal_reasons = {'node_shutdown', 'synthetic_test_exit'}
        if self._safe_shutdown_published and reason in terminal_reasons:
            return
        values = safe_actuator_mapping(
            self._adapter.bench.sample_index,
            state=self._adapter.bench.state.value,
            reason=reason,
        )
        self._stamp_and_publish(
            self._actuator_pub, ActuatorCommand(), values
        )
        if reason in terminal_reasons:
            self._safe_shutdown_published = True

    def _on_tick(self) -> None:
        if self._synthetic_exit_requested:
            rclpy.shutdown()
            return
        if not self._adapter.initial_telemetry_received:
            self._publish_safe_output('awaiting_initial_telemetry')
            return
        result = self._adapter.tick()
        self._publish_result(result)
        self._tick_count += 1
        if (
            self._synthetic_exit_after_ticks
            and self._tick_count >= self._synthetic_exit_after_ticks
        ):
            self._publish_safe_output('synthetic_test_exit')
            self.get_logger().warning(
                'exiting after configured synthetic test ticks'
            )
            self._synthetic_exit_requested = True

    def _on_reset(
        self,
        request: ResetFault.Request,
        response: ResetFault.Response,
    ) -> ResetFault.Response:
        del request
        result, accepted, reason = self._adapter.reset_tick()
        self._publish_result(result, reset_reason=reason)
        response.accepted = accepted
        response.reason = reason
        response.sample_index = result.record.sample_index
        response.state = result.record.state.value
        return response

    def publish_shutdown_safe_output(self) -> None:
        self._publish_safe_output('node_shutdown')


def main(args: list[str] | None = None) -> None:
    rclpy.init(
        args=args, signal_handler_options=SignalHandlerOptions.NO
    )
    node: SupervisorNode | None = None

    def safe_signal_handler(signum: int, frame: Any) -> None:
        del signum, frame
        if node is not None and rclpy.ok():
            try:
                node.publish_shutdown_safe_output()
            except Exception:
                # The DDS context can become invalid between the ok() check
                # and publish when launch is concurrently stopping the node.
                pass
        rclpy.try_shutdown()

    signal.signal(signal.SIGINT, safe_signal_handler)
    signal.signal(signal.SIGTERM, safe_signal_handler)
    try:
        node = SupervisorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                try:
                    node.publish_shutdown_safe_output()
                    rclpy.spin_once(node, timeout_sec=0.05)
                except Exception:
                    # Shutdown must remain fail-safe even if the middleware
                    # context has already been torn down externally.
                    pass
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
