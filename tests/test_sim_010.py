"""SIM-010 deterministic synthetic supervisor software-in-the-loop tests."""

from __future__ import annotations

import copy
import json
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np

from models.sil import (
    FaultCode,
    SupervisorInputs,
    SupervisorState,
    SupervisorTestBench,
    SyntheticSilConfig,
)
from models.validate_sim import (
    bring_to_running,
    build_motor_model,
    build_validation_evidence,
    nominal_inputs,
    write_trace_csv,
)

ROOT = Path(__file__).resolve().parents[1]
SIM_CONFIG_FILE = (
    ROOT / "models" / "parameters" / "synthetic_sim_010.json"
)
MOTOR_FILE = ROOT / "models" / "parameters" / "synthetic_motor.json"
REPORT_FILE = (
    ROOT
    / "data"
    / "processed"
    / "sim_010_synthetic_validation_report.json"
)
TRACE_FILE = (
    ROOT
    / "data"
    / "processed"
    / "sim_010_synthetic_scenario_trace.csv"
)


class TestSim010SyntheticSupervisor(unittest.TestCase):
    """Independent safety, boundary, recovery, and evidence checks."""

    def setUp(self) -> None:
        self.sim_payload = json.loads(
            SIM_CONFIG_FILE.read_text(encoding="utf-8")
        )
        self.motor_payload = json.loads(
            MOTOR_FILE.read_text(encoding="utf-8")
        )
        self.config = SyntheticSilConfig.from_mapping(self.sim_payload)
        self.motor_model = build_motor_model(self.motor_payload)

    def new_bench(self) -> SupervisorTestBench:
        return SupervisorTestBench(self.config, self.motor_model)

    def running_bench(self) -> SupervisorTestBench:
        bench = self.new_bench()
        bring_to_running(bench, self.config)
        self.assertEqual(bench.state, SupervisorState.RUNNING)
        return bench

    def run_input(self, **overrides: Any) -> SupervisorInputs:
        values: dict[str, Any] = {
            "arm_request": True,
            "run_request": True,
            "requested_motor_voltage_v": (
                self.config.synthetic_normal_run_command_voltage_v
            ),
        }
        values.update(overrides)
        return nominal_inputs(self.config, **values)

    def faulted_bench(
        self,
        *,
        fault_inputs: SupervisorInputs | None = None,
    ) -> SupervisorTestBench:
        bench = self.running_bench()
        record = bench.step(
            fault_inputs
            if fault_inputs is not None
            else self.run_input(estop_active=True)
        )
        self.assertEqual(record.state, SupervisorState.FAULT_LATCHED)
        self.assertTrue(record.safe_output)
        return bench

    def assert_safe_fault(
        self,
        record: Any,
        expected_fault: FaultCode,
    ) -> None:
        self.assertEqual(record.state, SupervisorState.FAULT_LATCHED)
        self.assertIn(expected_fault, record.detected_faults)
        self.assertIn(expected_fault, record.latched_faults)
        self.assertFalse(record.relay_enable_command)
        self.assertEqual(record.motor_command_v, 0.0)
        self.assertTrue(record.safe_output)

    def test_normal_startup_operation_and_shutdown(self) -> None:
        bench = self.new_bench()
        bring_to_running(bench, self.config)
        running = bench.step(self.run_input())
        self.assertTrue(running.relay_enable_command)
        self.assertNotEqual(running.motor_command_v, 0.0)
        shutdown = bench.step(
            nominal_inputs(self.config, shutdown_request=True)
        )
        self.assertEqual(shutdown.state, SupervisorState.SAFE_SHUTDOWN)
        self.assertTrue(shutdown.safe_output)

    def test_primary_faults_force_safe_outputs_in_same_sample(self) -> None:
        cases = (
            (
                FaultCode.EMERGENCY_STOP,
                {"estop_active": True},
            ),
            (
                FaultCode.ENCODER_FAILURE,
                {"encoder_failed": True},
            ),
            (
                FaultCode.ENCODER_INVALID,
                {"encoder_invalid": True},
            ),
            (
                FaultCode.UNDERVOLTAGE,
                {
                    "supply_voltage_v": (
                        self.config.synthetic_undervoltage_threshold_v - 0.1
                    )
                },
            ),
        )
        for expected_fault, overrides in cases:
            with self.subTest(fault=expected_fault.value):
                bench = self.running_bench()
                record = bench.step(self.run_input(**overrides))
                self.assert_safe_fault(record, expected_fault)

    def test_watchdog_faults_at_exact_missed_sample_threshold(self) -> None:
        bench = self.running_bench()
        threshold = (
            self.config.synthetic_watchdog_missed_heartbeat_samples
        )
        for missed_count in range(1, threshold):
            record = bench.step(
                self.run_input(watchdog_heartbeat=False)
            )
            self.assertEqual(record.watchdog_missed_samples, missed_count)
            self.assertNotIn(
                FaultCode.WATCHDOG_TIMEOUT,
                record.detected_faults,
            )
            self.assertEqual(record.state, SupervisorState.RUNNING)
        boundary = bench.step(
            self.run_input(watchdog_heartbeat=False)
        )
        self.assertEqual(boundary.watchdog_missed_samples, threshold)
        self.assert_safe_fault(boundary, FaultCode.WATCHDOG_TIMEOUT)

    def test_watchdog_heartbeat_resets_counter(self) -> None:
        bench = self.running_bench()
        bench.step(self.run_input(watchdog_heartbeat=False))
        record = bench.step(self.run_input(watchdog_heartbeat=True))
        self.assertEqual(record.watchdog_missed_samples, 0)
        self.assertEqual(record.state, SupervisorState.RUNNING)

    def test_encoder_first_sample_is_only_a_baseline(self) -> None:
        bench = self.new_bench()
        record = bench.step(nominal_inputs(self.config))
        self.assertEqual(record.encoder_sequence, 1)
        self.assertFalse(record.encoder_liveness_established)
        self.assertEqual(record.state, SupervisorState.SAFE_STARTUP)

    def test_encoder_first_genuine_transition_establishes_liveness(self) -> None:
        bench = self.new_bench()
        bench.step(nominal_inputs(self.config))
        record = bench.step(nominal_inputs(self.config))
        self.assertEqual(record.encoder_sequence, 2)
        self.assertTrue(record.encoder_liveness_established)
        self.assertEqual(record.encoder_stale_age_samples, 0)
        self.assertEqual(record.state, SupervisorState.SAFE_STARTUP)

    def test_startup_requires_liveness_and_configured_healthy_samples(self) -> None:
        bench = self.new_bench()
        first = bench.step(nominal_inputs(self.config))
        self.assertEqual(first.state, SupervisorState.SAFE_STARTUP)
        for count in range(
            1,
            self.config.synthetic_safe_startup_healthy_samples + 1,
        ):
            record = bench.step(nominal_inputs(self.config))
            expected = (
                SupervisorState.READY
                if count
                == self.config.synthetic_safe_startup_healthy_samples
                else SupervisorState.SAFE_STARTUP
            )
            self.assertEqual(record.state, expected)

    def test_recovery_cannot_bypass_unestablished_encoder_liveness(
        self,
    ) -> None:
        bench = self.new_bench()
        fault = bench.step(
            nominal_inputs(
                self.config,
                estop_active=True,
                encoder_stale=True,
            )
        )
        self.assertEqual(fault.state, SupervisorState.FAULT_LATCHED)
        self.assertFalse(fault.encoder_liveness_established)
        reset = bench.step(
            nominal_inputs(
                self.config,
                reset_request=True,
                encoder_stale=True,
            )
        )
        self.assertTrue(reset.reset_accepted)
        self.assertFalse(reset.encoder_liveness_established)
        while bench.state == SupervisorState.SAFE_SHUTDOWN:
            record = bench.step(
                nominal_inputs(self.config, encoder_stale=True)
            )
        self.assertEqual(record.state, SupervisorState.FAULT_LATCHED)
        self.assertIn(FaultCode.ENCODER_STALE, record.detected_faults)
        self.assertNotEqual(record.state, SupervisorState.READY)

    def test_no_encoder_advancement_faults_at_exact_startup_threshold(
        self,
    ) -> None:
        bench = self.new_bench()
        baseline = bench.step(
            nominal_inputs(self.config, encoder_stale=True)
        )
        self.assertEqual(baseline.encoder_sequence, 0)
        self.assertEqual(baseline.encoder_stale_age_samples, 0)
        self.assertFalse(baseline.encoder_liveness_established)
        threshold = self.config.synthetic_encoder_stale_samples
        for stale_age in range(1, threshold):
            record = bench.step(
                nominal_inputs(self.config, encoder_stale=True)
            )
            self.assertEqual(record.encoder_stale_age_samples, stale_age)
            self.assertNotIn(FaultCode.ENCODER_STALE, record.detected_faults)
            self.assertEqual(record.state, SupervisorState.SAFE_STARTUP)
        boundary = bench.step(
            nominal_inputs(self.config, encoder_stale=True)
        )
        self.assertEqual(
            boundary.encoder_stale_age_samples,
            threshold,
        )
        self.assert_safe_fault(boundary, FaultCode.ENCODER_STALE)

    def test_repeated_encoder_sequence_after_liveness_uses_same_boundary(
        self,
    ) -> None:
        bench = self.running_bench()
        threshold = self.config.synthetic_encoder_stale_samples
        for stale_age in range(1, threshold):
            record = bench.step(self.run_input(encoder_stale=True))
            self.assertTrue(record.encoder_liveness_established)
            self.assertEqual(record.encoder_stale_age_samples, stale_age)
            self.assertNotIn(FaultCode.ENCODER_STALE, record.detected_faults)
        boundary = bench.step(self.run_input(encoder_stale=True))
        self.assert_safe_fault(boundary, FaultCode.ENCODER_STALE)

    def test_encoder_failed_and_invalid_telemetry_are_distinct_faults(
        self,
    ) -> None:
        for field, expected in (
            ("encoder_failed", FaultCode.ENCODER_FAILURE),
            ("encoder_invalid", FaultCode.ENCODER_INVALID),
        ):
            with self.subTest(field=field):
                bench = self.running_bench()
                record = bench.step(self.run_input(**{field: True}))
                self.assert_safe_fault(record, expected)

    def test_relay_mismatch_faults_at_exact_threshold(self) -> None:
        bench = self.running_bench()
        threshold = (
            self.config.synthetic_relay_feedback_mismatch_samples
        )
        for mismatch_count in range(1, threshold):
            record = bench.step(
                self.run_input(relay_feedback_failed=True)
            )
            self.assertEqual(
                record.relay_feedback_mismatch_samples,
                mismatch_count,
            )
            self.assertEqual(record.state, SupervisorState.RUNNING)
            self.assertNotIn(
                FaultCode.RELAY_FEEDBACK_FAILURE,
                record.detected_faults,
            )
            self.assertEqual(record.motor_command_v, 0.0)
            self.assertTrue(record.relay_enable_command)
            self.assertFalse(record.relay_enable_feedback)
        boundary = bench.step(
            self.run_input(relay_feedback_failed=True)
        )
        self.assertEqual(
            boundary.relay_feedback_mismatch_samples,
            threshold,
        )
        self.assert_safe_fault(
            boundary,
            FaultCode.RELAY_FEEDBACK_FAILURE,
        )

    def test_persistent_relay_failure_source_rejects_reset(self) -> None:
        bench = self.running_bench()
        for _ in range(
            self.config.synthetic_relay_feedback_mismatch_samples
        ):
            bench.step(self.run_input(relay_feedback_failed=True))
        settle = bench.step(
            nominal_inputs(
                self.config,
                relay_feedback_failed=True,
            )
        )
        self.assertEqual(settle.relay_feedback_mismatch_samples, 0)
        self.assertIn(
            FaultCode.RELAY_FEEDBACK_FAILURE,
            settle.active_raw_fault_sources,
        )
        rejected = bench.step(
            nominal_inputs(
                self.config,
                reset_request=True,
                relay_feedback_failed=True,
            )
        )
        self.assertTrue(rejected.reset_rejected)
        self.assertFalse(rejected.reset_accepted)
        self.assertEqual(rejected.state, SupervisorState.FAULT_LATCHED)
        self.assertTrue(rejected.safe_output)

    def test_cleared_relay_failure_source_allows_safe_reset(self) -> None:
        bench = self.running_bench()
        for _ in range(
            self.config.synthetic_relay_feedback_mismatch_samples
        ):
            bench.step(self.run_input(relay_feedback_failed=True))
        accepted = bench.step(
            nominal_inputs(self.config, reset_request=True)
        )
        self.assertTrue(accepted.reset_accepted)
        self.assertFalse(accepted.reset_rejected)
        self.assertEqual(accepted.state, SupervisorState.SAFE_SHUTDOWN)
        self.assertTrue(accepted.safe_output)

    def test_reset_rejects_each_arm_run_combination_except_both_low(
        self,
    ) -> None:
        for arm, run in ((True, False), (False, True), (True, True)):
            with self.subTest(arm=arm, run=run):
                bench = self.faulted_bench()
                record = bench.step(
                    nominal_inputs(
                        self.config,
                        reset_request=True,
                        arm_request=arm,
                        run_request=run,
                    )
                )
                self.assertTrue(record.reset_rejected)
                self.assertFalse(record.reset_accepted)
                self.assertEqual(
                    record.state,
                    SupervisorState.FAULT_LATCHED,
                )
                self.assertTrue(record.safe_output)

    def test_source_clearance_alone_cannot_restart(self) -> None:
        bench = self.faulted_bench()
        for _ in range(3):
            record = bench.step(self.run_input())
            self.assertEqual(record.state, SupervisorState.FAULT_LATCHED)
            self.assertTrue(record.safe_output)
        self.assertIn(FaultCode.EMERGENCY_STOP, bench.latched_faults)

    def test_arm_run_held_from_safe_shutdown_into_ready_is_not_consumed(
        self,
    ) -> None:
        bench = self.faulted_bench()
        accepted = bench.step(
            nominal_inputs(self.config, reset_request=True)
        )
        self.assertTrue(accepted.reset_accepted)
        for _ in range(
            self.config.synthetic_safe_shutdown_hold_samples
        ):
            held = bench.step(self.run_input())
            self.assertTrue(held.safe_output)
        self.assertEqual(bench.state, SupervisorState.READY)
        still_held = bench.step(self.run_input())
        self.assertEqual(still_held.state, SupervisorState.READY)
        self.assertTrue(still_held.safe_output)

    def test_request_before_ready_disarmed_sample_is_ignored(self) -> None:
        bench = self.faulted_bench()
        bench.step(nominal_inputs(self.config, reset_request=True))
        for _ in range(
            self.config.synthetic_safe_shutdown_hold_samples
        ):
            bench.step(nominal_inputs(self.config))
        premature = bench.step(self.run_input())
        self.assertEqual(premature.state, SupervisorState.READY)
        self.assertTrue(premature.safe_output)

    def test_recovery_requires_ready_disarmed_sample_then_new_request(
        self,
    ) -> None:
        bench = self.faulted_bench()
        accepted = bench.step(
            nominal_inputs(self.config, reset_request=True)
        )
        self.assertTrue(accepted.reset_accepted)
        for _ in range(
            self.config.synthetic_safe_shutdown_hold_samples
        ):
            bench.step(self.run_input())
        ignored = bench.step(self.run_input())
        self.assertEqual(ignored.state, SupervisorState.READY)
        disarmed = bench.step(nominal_inputs(self.config))
        self.assertEqual(disarmed.state, SupervisorState.READY)
        self.assertTrue(disarmed.safe_output)
        rearmed = bench.step(self.run_input())
        self.assertEqual(rearmed.state, SupervisorState.RUNNING)
        self.assertTrue(rearmed.relay_enable_command)
        self.assertNotEqual(rearmed.motor_command_v, 0.0)

    def test_supply_exact_threshold_is_healthy(self) -> None:
        bench = self.running_bench()
        record = bench.step(
            self.run_input(
                supply_voltage_v=(
                    self.config.synthetic_undervoltage_threshold_v
                )
            )
        )
        self.assertTrue(record.supply_healthy)
        self.assertNotIn(FaultCode.UNDERVOLTAGE, record.detected_faults)
        self.assertEqual(record.state, SupervisorState.RUNNING)

    def test_supply_below_threshold_faults(self) -> None:
        bench = self.running_bench()
        record = bench.step(
            self.run_input(
                supply_voltage_v=(
                    self.config.synthetic_undervoltage_threshold_v
                    - 1e-12
                )
            )
        )
        self.assert_safe_fault(record, FaultCode.UNDERVOLTAGE)

    def test_malformed_supply_values_fault_without_exception(self) -> None:
        values = (
            "4.5",
            None,
            True,
            float("nan"),
            float("inf"),
            float("-inf"),
            -1e-12,
            10**10000,
        )
        for value in values:
            with self.subTest(value=value):
                bench = self.running_bench()
                record = bench.step(
                    self.run_input(supply_voltage_v=value)
                )
                self.assert_safe_fault(
                    record,
                    FaultCode.INVALID_SUPPLY_VOLTAGE,
                )
                self.assertIn(
                    "supply_voltage_v",
                    record.invalid_input_fields,
                )

    def test_command_inside_and_at_limits_passes_unchanged(self) -> None:
        limit = self.config.synthetic_command_voltage_abs_max_v
        for command in (0.0, limit - 1e-12, limit, -limit):
            with self.subTest(command=command):
                bench = self.running_bench()
                record = bench.step(
                    self.run_input(requested_motor_voltage_v=command)
                )
                self.assertEqual(record.motor_command_v, command)
                self.assertFalse(record.command_saturated)
                self.assertFalse(record.detected_faults)
                self.assertEqual(record.state, SupervisorState.RUNNING)

    def test_finite_overlimit_commands_clip_without_fault(self) -> None:
        limit = self.config.synthetic_command_voltage_abs_max_v
        for command, expected in (
            (limit + 1e-12, limit),
            (-limit - 1e-12, -limit),
        ):
            with self.subTest(command=command):
                bench = self.running_bench()
                record = bench.step(
                    self.run_input(requested_motor_voltage_v=command)
                )
                self.assertEqual(record.motor_command_v, expected)
                self.assertTrue(record.command_saturated)
                self.assertFalse(record.detected_faults)
                self.assertFalse(record.latched_faults)
                self.assertEqual(record.state, SupervisorState.RUNNING)

    def test_malformed_commands_fault_without_exception(self) -> None:
        values = (
            "2.0",
            None,
            True,
            float("nan"),
            float("inf"),
            float("-inf"),
            10**10000,
        )
        for value in values:
            with self.subTest(value=value):
                bench = self.running_bench()
                record = bench.step(
                    self.run_input(requested_motor_voltage_v=value)
                )
                self.assert_safe_fault(record, FaultCode.INVALID_COMMAND)
                self.assertIn(
                    "requested_motor_voltage_v",
                    record.invalid_input_fields,
                )

    def test_malformed_boolean_inputs_fault_without_exception(self) -> None:
        fields = (
            "arm_request",
            "run_request",
            "shutdown_request",
            "reset_request",
            "watchdog_heartbeat",
            "estop_active",
            "encoder_stale",
            "encoder_failed",
            "encoder_invalid",
            "relay_feedback_failed",
        )
        invalid_values = ("false", None, 0, 1.0)
        for field in fields:
            for value in invalid_values:
                with self.subTest(field=field, value=value):
                    bench = self.running_bench()
                    record = bench.step(
                        self.run_input(**{field: value})
                    )
                    self.assert_safe_fault(
                        record,
                        FaultCode.INVALID_RUNTIME_INPUT,
                    )
                    self.assertIn(field, record.invalid_input_fields)

    def test_malformed_sample_object_faults_without_exception(self) -> None:
        bench = self.running_bench()
        record = bench.step(None)  # type: ignore[arg-type]
        self.assert_safe_fault(record, FaultCode.INVALID_RUNTIME_INPUT)
        self.assertEqual(record.invalid_input_fields, ("inputs",))

    def test_config_rejects_boolean_and_non_numeric_numbers(self) -> None:
        locations = (
            (
                "synthetic_plant",
                "opposing_load_torque_nm",
            ),
            (
                "synthetic_operating_values",
                "nominal_supply_voltage_v",
            ),
            (
                "synthetic_operating_values",
                "normal_run_command_voltage_v",
            ),
            (
                "synthetic_operating_limits",
                "command_voltage_abs_max_v",
            ),
            (
                "synthetic_operating_limits",
                "undervoltage_threshold_v",
            ),
        )
        for section, field in locations:
            for value in (
                True,
                "1.0",
                None,
                float("nan"),
                float("inf"),
                float("-inf"),
            ):
                with self.subTest(section=section, field=field, value=value):
                    payload = copy.deepcopy(self.sim_payload)
                    payload[section][field] = value
                    with self.assertRaises(ValueError):
                        SyntheticSilConfig.from_mapping(payload)

    def test_config_rejects_invalid_integer_thresholds(self) -> None:
        fields = (
            "safe_startup_healthy_samples",
            "safe_shutdown_hold_samples",
            "watchdog_missed_heartbeat_samples",
            "encoder_stale_samples",
            "relay_feedback_mismatch_samples",
        )
        for field in fields:
            for value in (True, 1.0, "1", None, 0, -1):
                with self.subTest(field=field, value=value):
                    payload = copy.deepcopy(self.sim_payload)
                    payload["synthetic_fault_thresholds"][field] = value
                    with self.assertRaises(ValueError):
                        SyntheticSilConfig.from_mapping(payload)
        for value in (True, 4096.0, "4096", None, 3, 0, -1):
            with self.subTest(encoder_counts=value):
                payload = copy.deepcopy(self.sim_payload)
                payload["synthetic_interfaces"][
                    "encoder_counts_per_revolution"
                ] = value
                with self.assertRaises(ValueError):
                    SyntheticSilConfig.from_mapping(payload)

    def test_config_rejects_malformed_shape_ranges_and_channels(self) -> None:
        invalid_payloads: list[dict[str, Any]] = []
        missing = copy.deepcopy(self.sim_payload)
        del missing["synthetic_plant"]
        invalid_payloads.append(missing)
        malformed = copy.deepcopy(self.sim_payload)
        malformed["synthetic_interfaces"] = None
        invalid_payloads.append(malformed)
        negative_load = copy.deepcopy(self.sim_payload)
        negative_load["synthetic_plant"]["opposing_load_torque_nm"] = -1e-12
        invalid_payloads.append(negative_load)
        bad_range = copy.deepcopy(self.sim_payload)
        bad_range["synthetic_operating_limits"][
            "undervoltage_threshold_v"
        ] = bad_range["synthetic_operating_values"][
            "nominal_supply_voltage_v"
        ]
        invalid_payloads.append(bad_range)
        bad_channel = copy.deepcopy(self.sim_payload)
        bad_channel["synthetic_interfaces"][
            "fault_channel"
        ] = "unordered"
        invalid_payloads.append(bad_channel)
        for payload in invalid_payloads:
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    SyntheticSilConfig.from_mapping(payload)

    def test_direct_config_validation_rejects_boolean_numeric_field(
        self,
    ) -> None:
        with self.assertRaises(ValueError):
            replace(
                self.config,
                synthetic_nominal_supply_voltage_v=True,
            ).validate()

    def test_motor_config_rejects_malformed_numeric_values(self) -> None:
        for value in (
            True,
            "0.001",
            None,
            0.0,
            -1e-12,
            float("nan"),
            float("inf"),
            float("-inf"),
        ):
            with self.subTest(value=value):
                payload = copy.deepcopy(self.motor_payload)
                payload["sample_period_s"] = value
                with self.assertRaises(ValueError):
                    build_motor_model(payload)
        for value in (
            True,
            "1.0",
            None,
            0.0,
            -1.0,
            float("nan"),
            float("inf"),
            float("-inf"),
        ):
            with self.subTest(parameter=value):
                payload = copy.deepcopy(self.motor_payload)
                payload["parameters"]["armature_resistance_ohm"] = value
                with self.assertRaises(ValueError):
                    build_motor_model(payload)

    def test_bench_rejects_malformed_discrete_motor_model(self) -> None:
        invalid_models = (
            replace(self.motor_model, sample_period_s=True),
            replace(self.motor_model, sample_period_s=float("nan")),
            replace(
                self.motor_model,
                a=np.full((3, 3), float("inf")),
            ),
            replace(
                self.motor_model,
                b=np.zeros((3, 1)),
            ),
        )
        for motor_model in invalid_models:
            with self.subTest(motor_model=motor_model):
                with self.assertRaises(ValueError):
                    SupervisorTestBench(self.config, motor_model)

    def test_scenario_suite_passes_and_is_exactly_repeatable(self) -> None:
        report, traces = build_validation_evidence(
            self.sim_payload,
            self.motor_payload,
        )
        replay_report, replay_traces = build_validation_evidence(
            copy.deepcopy(self.sim_payload),
            copy.deepcopy(self.motor_payload),
        )
        self.assertEqual(report, replay_report)
        self.assertEqual(traces, replay_traces)
        self.assertEqual(report["result"], "PASS")
        self.assertTrue(report["checks"]["exact_replay_matches"])

    def test_committed_evidence_matches_fresh_regeneration(self) -> None:
        report, traces = build_validation_evidence(
            self.sim_payload,
            self.motor_payload,
        )
        expected_report = json.dumps(
            report,
            indent=2,
            sort_keys=True,
        ) + "\n"
        self.assertEqual(
            REPORT_FILE.read_text(encoding="utf-8"),
            expected_report,
        )
        with tempfile.TemporaryDirectory() as directory:
            trace_path = Path(directory) / "trace.csv"
            write_trace_csv(trace_path, traces)
            self.assertEqual(
                TRACE_FILE.read_text(encoding="utf-8"),
                trace_path.read_text(encoding="utf-8"),
            )

    def test_every_detected_or_latched_fault_record_is_safe(self) -> None:
        report, traces = build_validation_evidence(
            self.sim_payload,
            self.motor_payload,
        )
        fault_records = [
            record
            for record in traces
            if record["detected_faults"]
            or record["state"] == SupervisorState.FAULT_LATCHED.value
        ]
        self.assertTrue(fault_records)
        self.assertTrue(
            report["checks"][
                "all_detected_or_latched_fault_records_safe"
            ]
        )
        for record in fault_records:
            self.assertFalse(record["relay_enable_command"])
            self.assertEqual(record["motor_command_v"], 0.0)
            self.assertTrue(record["safe_output"])

    def test_fault_and_scenario_ordering_is_stable(self) -> None:
        bench = self.running_bench()
        record = bench.step(
            self.run_input(
                supply_voltage_v=None,
                requested_motor_voltage_v=None,
                estop_active="true",
            )
        )
        self.assertEqual(
            record.detected_faults,
            tuple(
                code
                for code in FaultCode
                if code
                in {
                    FaultCode.EMERGENCY_STOP,
                    FaultCode.INVALID_SUPPLY_VOLTAGE,
                    FaultCode.INVALID_COMMAND,
                    FaultCode.INVALID_RUNTIME_INPUT,
                }
            ),
        )
        report, _ = build_validation_evidence(
            self.sim_payload,
            self.motor_payload,
        )
        scenario_ids = [
            scenario["scenario_id"]
            for scenario in report["scenarios"]
        ]
        self.assertEqual(scenario_ids, list(dict.fromkeys(scenario_ids)))

    def test_pre_fixed_point_holds_remain_unchanged(self) -> None:
        report, _ = build_validation_evidence(
            self.sim_payload,
            self.motor_payload,
        )
        self.assertEqual(report["coefficient_freeze_readiness"], "HOLD")
        self.assertEqual(
            report["fixed_point_conversion_readiness"],
            "HOLD",
        )
        with self.assertRaises(ValueError):
            replace(
                self.config,
                coefficient_freeze_readiness="READY",
            ).validate()
        with self.assertRaises(ValueError):
            replace(
                self.config,
                fixed_point_conversion_readiness="READY",
            ).validate()


if __name__ == "__main__":
    unittest.main()
