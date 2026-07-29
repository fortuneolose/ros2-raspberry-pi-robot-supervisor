"""ROS-independent tests for the ROS2-010 transport/SIM-010 adapter."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(
    0, str(ROOT / "ros2_ws" / "src" / "robot_supervisor")
)

from models.sil import FaultCode, SupervisorState  # noqa: E402
from robot_supervisor.core import (  # noqa: E402
    ACCEPTED,
    DUPLICATE,
    OUT_OF_ORDER,
    CommandFrame,
    EncoderFrame,
    SafetyFrame,
    SupervisorAdapter,
    actuator_mapping,
    fault_telemetry_mapping,
    safe_actuator_mapping,
    safety_status_mapping,
    supervisor_telemetry_mapping,
)

SIM_CONFIG = (
    ROOT / "models" / "parameters" / "synthetic_sim_010.json"
)
MOTOR_CONFIG = (
    ROOT / "models" / "parameters" / "synthetic_motor.json"
)


class TestRos2010Core(unittest.TestCase):
    """Independent mapping, transport, fault, and recovery checks."""

    def setUp(self) -> None:
        self.adapter = self.new_adapter()
        self.command_sequence = 0
        self.encoder_sequence = 0
        self.safety_sequence = 0

    @staticmethod
    def new_adapter() -> SupervisorAdapter:
        return SupervisorAdapter.from_files(SIM_CONFIG, MOTOR_CONFIG)

    def _previous_relay(self) -> bool:
        records = self.adapter.bench.telemetry.records
        return records[-1].relay_enable_command if records else False

    def step(
        self,
        *,
        arm: Any = False,
        run: Any = False,
        shutdown: Any = False,
        command_v: Any = 0.0,
        encoder: bool = True,
        encoder_sequence: int | None = None,
        encoder_healthy: Any = True,
        encoder_valid: Any = True,
        encoder_position: Any = 0.0,
        safety: bool = True,
        heartbeat: Any = True,
        estop: Any = False,
        supply_v: Any = 6.0,
        relay_feedback_enabled: bool | None = None,
        relay_feedback_healthy: Any = True,
    ):
        self.command_sequence += 1
        self.adapter.ingest_command(
            CommandFrame(
                sequence=self.command_sequence,
                arm_request=arm,
                run_request=run,
                shutdown_request=shutdown,
                requested_motor_voltage_v=command_v,
            )
        )
        if encoder:
            if encoder_sequence is None:
                self.encoder_sequence += 1
                sequence = self.encoder_sequence
            else:
                sequence = encoder_sequence
            self.adapter.ingest_encoder(
                EncoderFrame(
                    sequence=sequence,
                    position_rad=encoder_position,
                    healthy=encoder_healthy,
                    valid=encoder_valid,
                )
            )
        if safety:
            self.safety_sequence += 1
            feedback = (
                self._previous_relay()
                if relay_feedback_enabled is None
                else relay_feedback_enabled
            )
            self.adapter.ingest_safety(
                SafetyFrame(
                    sequence=self.safety_sequence,
                    supply_voltage_v=supply_v,
                    software_estop_active=estop,
                    watchdog_heartbeat=heartbeat,
                    relay_feedback_enabled=feedback,
                    relay_feedback_healthy=relay_feedback_healthy,
                )
            )
        return self.adapter.tick()

    def bring_to_running(self) -> None:
        for _ in range(3):
            result = self.step()
        self.assertEqual(result.record.state, SupervisorState.READY)
        self.step()
        result = self.step(arm=True, run=True, command_v=2.0)
        self.assertEqual(result.record.state, SupervisorState.RUNNING)

    def assert_safe_fault(self, result: Any, fault: FaultCode) -> None:
        self.assertEqual(
            result.record.state, SupervisorState.FAULT_LATCHED
        )
        self.assertIn(fault, result.record.latched_faults)
        self.assertFalse(result.record.relay_enable_command)
        self.assertEqual(result.record.motor_command_v, 0.0)
        self.assertTrue(result.record.safe_output)

    def test_interface_to_sim010_mapping(self) -> None:
        self.adapter.ingest_command(
            CommandFrame(1, True, True, False, 1.25)
        )
        self.adapter.ingest_encoder(
            EncoderFrame(1, 0.125, True, True)
        )
        self.adapter.ingest_safety(
            SafetyFrame(1, 5.5, False, True, False, True)
        )
        result = self.adapter.tick()
        self.assertEqual(result.command_disposition, ACCEPTED)
        self.assertEqual(result.encoder_disposition, ACCEPTED)
        self.assertEqual(result.safety_disposition, ACCEPTED)
        self.assertEqual(result.inputs.supply_voltage_v, 5.5)
        self.assertEqual(result.inputs.requested_motor_voltage_v, 1.25)
        self.assertTrue(result.inputs.arm_request)
        self.assertTrue(result.inputs.run_request)
        self.assertFalse(result.inputs.encoder_stale)

    def test_sim010_output_to_message_mapping(self) -> None:
        result = self.step()
        actuator = actuator_mapping(result.record)
        status = safety_status_mapping(result.record)
        telemetry = supervisor_telemetry_mapping(result)
        fault = fault_telemetry_mapping(result)
        self.assertEqual(actuator["motor_voltage_v"], 0.0)
        self.assertFalse(actuator["relay_enable"])
        self.assertTrue(status["safe_output"])
        self.assertEqual(telemetry["state"], "SAFE_STARTUP")
        self.assertEqual(telemetry["encoder_disposition"], ACCEPTED)
        self.assertEqual(fault["latched_faults"], [])

    def test_normal_startup_ready_and_running(self) -> None:
        self.bring_to_running()
        result = self.step(arm=True, run=True, command_v=2.0)
        self.assertTrue(result.record.relay_enable_command)
        self.assertEqual(result.record.motor_command_v, 2.0)

    def test_software_estop_faults_same_tick(self) -> None:
        self.bring_to_running()
        result = self.step(
            arm=True, run=True, command_v=2.0, estop=True
        )
        self.assert_safe_fault(result, FaultCode.EMERGENCY_STOP)

    def test_watchdog_exact_n_minus_one_and_n_boundary(self) -> None:
        self.bring_to_running()
        threshold = (
            self.adapter.config.synthetic_watchdog_missed_heartbeat_samples
        )
        for _ in range(threshold - 1):
            result = self.step(
                arm=True, run=True, command_v=2.0, heartbeat=False
            )
            self.assertNotIn(
                FaultCode.WATCHDOG_TIMEOUT, result.record.detected_faults
            )
        result = self.step(
            arm=True, run=True, command_v=2.0, heartbeat=False
        )
        self.assert_safe_fault(result, FaultCode.WATCHDOG_TIMEOUT)

    def test_missing_encoder_faults_at_stale_boundary(self) -> None:
        self.bring_to_running()
        threshold = self.adapter.config.synthetic_encoder_stale_samples
        for _ in range(threshold - 1):
            result = self.step(
                arm=True,
                run=True,
                command_v=2.0,
                encoder=False,
            )
            self.assertNotIn(
                FaultCode.ENCODER_STALE, result.record.detected_faults
            )
        result = self.step(
            arm=True, run=True, command_v=2.0, encoder=False
        )
        self.assert_safe_fault(result, FaultCode.ENCODER_STALE)

    def test_duplicate_encoder_is_ignored_and_ages_to_fault(self) -> None:
        self.bring_to_running()
        duplicate = self.encoder_sequence
        threshold = self.adapter.config.synthetic_encoder_stale_samples
        for index in range(threshold):
            self.command_sequence += 1
            self.adapter.ingest_command(
                CommandFrame(
                    self.command_sequence, True, True, False, 2.0
                )
            )
            disposition = self.adapter.ingest_encoder(
                EncoderFrame(duplicate, 0.0, True, True)
            )
            self.safety_sequence += 1
            self.adapter.ingest_safety(
                SafetyFrame(
                    self.safety_sequence,
                    6.0,
                    False,
                    True,
                    self._previous_relay(),
                    True,
                )
            )
            result = self.adapter.tick()
            self.assertEqual(disposition, DUPLICATE)
            if index < threshold - 1:
                self.assertNotIn(
                    FaultCode.ENCODER_STALE,
                    result.record.detected_faults,
                )
        self.assert_safe_fault(result, FaultCode.ENCODER_STALE)

    def test_out_of_order_encoder_faults_immediately(self) -> None:
        self.bring_to_running()
        self.command_sequence += 1
        self.adapter.ingest_command(
            CommandFrame(
                self.command_sequence, True, True, False, 2.0
            )
        )
        disposition = self.adapter.ingest_encoder(
            EncoderFrame(self.encoder_sequence - 1, 0.0, True, True)
        )
        self.safety_sequence += 1
        self.adapter.ingest_safety(
            SafetyFrame(
                self.safety_sequence,
                6.0,
                False,
                True,
                self._previous_relay(),
                True,
            )
        )
        result = self.adapter.tick()
        self.assertEqual(disposition, OUT_OF_ORDER)
        self.assert_safe_fault(result, FaultCode.ENCODER_INVALID)

    def test_encoder_failure_and_invalidity(self) -> None:
        for kwargs, expected in (
            ({"encoder_healthy": False}, FaultCode.ENCODER_FAILURE),
            ({"encoder_valid": False}, FaultCode.ENCODER_INVALID),
            ({"encoder_position": math.nan}, FaultCode.ENCODER_INVALID),
        ):
            with self.subTest(expected=expected):
                self.setUp()
                self.bring_to_running()
                result = self.step(
                    arm=True, run=True, command_v=2.0, **kwargs
                )
                self.assert_safe_fault(result, expected)

    def test_relay_feedback_failure_exact_boundary(self) -> None:
        self.bring_to_running()
        threshold = (
            self.adapter.config.synthetic_relay_feedback_mismatch_samples
        )
        for index in range(threshold):
            result = self.step(
                arm=True,
                run=True,
                command_v=2.0,
                relay_feedback_enabled=False,
                relay_feedback_healthy=False,
            )
            if index < threshold - 1:
                self.assertNotIn(
                    FaultCode.RELAY_FEEDBACK_FAILURE,
                    result.record.detected_faults,
                )
                self.assertEqual(result.record.motor_command_v, 0.0)
        self.assert_safe_fault(
            result, FaultCode.RELAY_FEEDBACK_FAILURE
        )

    def test_undervoltage_fault_and_exact_threshold_health(self) -> None:
        self.bring_to_running()
        threshold = self.adapter.config.synthetic_undervoltage_threshold_v
        healthy = self.step(
            arm=True, run=True, command_v=2.0, supply_v=threshold
        )
        self.assertNotIn(
            FaultCode.UNDERVOLTAGE, healthy.record.detected_faults
        )
        faulted = self.step(
            arm=True,
            run=True,
            command_v=2.0,
            supply_v=threshold - 0.01,
        )
        self.assert_safe_fault(faulted, FaultCode.UNDERVOLTAGE)

    def test_finite_voltage_saturation_does_not_fault(self) -> None:
        self.bring_to_running()
        limit = self.adapter.config.synthetic_command_voltage_abs_max_v
        result = self.step(
            arm=True, run=True, command_v=limit + 1.0
        )
        self.assertEqual(result.record.motor_command_v, limit)
        self.assertTrue(result.record.command_saturated)
        self.assertFalse(result.record.detected_faults)
        self.assertEqual(result.record.state, SupervisorState.RUNNING)

    def test_malformed_and_nonfinite_command_faults(self) -> None:
        for malformed in (math.nan, math.inf, "bad", None, True):
            with self.subTest(malformed=malformed):
                self.setUp()
                self.bring_to_running()
                result = self.step(
                    arm=True, run=True, command_v=malformed
                )
                self.assert_safe_fault(
                    result, FaultCode.INVALID_COMMAND
                )

    def test_fault_latches_after_source_clears(self) -> None:
        self.bring_to_running()
        self.step(arm=True, run=True, command_v=2.0, estop=True)
        result = self.step(arm=False, run=False)
        self.assert_safe_fault(result, FaultCode.EMERGENCY_STOP)
        self.assertFalse(result.record.detected_faults)

    def test_unsafe_reset_rejected(self) -> None:
        self.bring_to_running()
        self.step(
            arm=True, run=True, command_v=2.0, estop=True
        )
        self.step(arm=True, run=True, command_v=2.0)
        result, accepted, reason = self.adapter.reset_tick()
        self.assertFalse(accepted)
        self.assertTrue(result.record.reset_rejected)
        self.assertIn("arm_or_run_active", reason)
        self.assertTrue(result.record.safe_output)

    def test_controlled_recovery_rejects_held_requests(self) -> None:
        self.bring_to_running()
        self.step(
            arm=True, run=True, command_v=2.0, estop=True
        )
        self.step(arm=False, run=False)
        reset, accepted, _ = self.adapter.reset_tick()
        self.assertTrue(accepted)
        self.assertTrue(reset.record.reset_accepted)
        hold = self.adapter.config.synthetic_safe_shutdown_hold_samples
        for _ in range(hold):
            result = self.step(arm=True, run=True, command_v=2.0)
        self.assertEqual(result.record.state, SupervisorState.READY)
        held = self.step(arm=True, run=True, command_v=2.0)
        self.assertEqual(held.record.state, SupervisorState.READY)
        self.step(arm=False, run=False)
        running = self.step(arm=True, run=True, command_v=2.0)
        self.assertEqual(running.record.state, SupervisorState.RUNNING)

    def test_supervisor_restart_returns_to_safe_startup(self) -> None:
        self.bring_to_running()
        restarted = self.new_adapter()
        result = restarted.tick()
        self.assertEqual(result.record.state, SupervisorState.SAFE_STARTUP)
        self.assertTrue(result.record.safe_output)
        self.assertFalse(result.record.encoder_liveness_established)

    def test_telemetry_disappearance_is_fail_safe(self) -> None:
        self.bring_to_running()
        count = max(
            self.adapter.config.synthetic_encoder_stale_samples,
            self.adapter.config.synthetic_watchdog_missed_heartbeat_samples,
        )
        for _ in range(count):
            self.command_sequence += 1
            self.adapter.ingest_command(
                CommandFrame(
                    self.command_sequence, True, True, False, 2.0
                )
            )
            result = self.adapter.tick()
        self.assertEqual(
            result.record.state, SupervisorState.FAULT_LATCHED
        )
        self.assertTrue(result.record.safe_output)
        self.assertTrue(
            {
                FaultCode.ENCODER_STALE,
                FaultCode.WATCHDOG_TIMEOUT,
            }
            & set(result.record.latched_faults)
        )

    def test_startup_and_shutdown_safe_mapping(self) -> None:
        startup = safe_actuator_mapping(
            0, state="SAFE_STARTUP", reason="node_startup"
        )
        shutdown = safe_actuator_mapping(
            12, state="RUNNING", reason="node_shutdown"
        )
        for mapping in (startup, shutdown):
            self.assertFalse(mapping["relay_enable"])
            self.assertEqual(mapping["motor_voltage_v"], 0.0)
        self.assertEqual(shutdown["reason"], "node_shutdown")


if __name__ == "__main__":
    unittest.main()
