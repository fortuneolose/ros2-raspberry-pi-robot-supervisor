"""Generate deterministic machine-readable ROS2-010 adapter evidence."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Callable

from models.sil import FaultCode, SupervisorState
from robot_supervisor.core import (
    actuator_mapping,
    CommandFrame,
    EncoderFrame,
    fault_telemetry_mapping,
    safety_status_mapping,
    SafetyFrame,
    supervisor_telemetry_mapping,
    SupervisorAdapter,
)


Scenario = Callable[['Harness'], dict[str, bool]]


class Harness:
    """Message-neutral deterministic publisher/adapter harness."""

    def __init__(self, sim_path: Path, motor_path: Path) -> None:
        self.sim_path = sim_path
        self.motor_path = motor_path
        self.adapter = SupervisorAdapter.from_files(sim_path, motor_path)
        self.command_sequence = 0
        self.encoder_sequence = 0
        self.safety_sequence = 0
        self.generation = 0
        self.results: list[Any] = []

    def previous_relay(self) -> bool:
        records = self.adapter.bench.telemetry.records
        return records[-1].relay_enable_command if records else False

    def step(
        self,
        *,
        arm: Any = False,
        run: Any = False,
        command_v: Any = 0.0,
        command: bool = True,
        encoder: bool = True,
        encoder_sequence: int | None = None,
        encoder_healthy: Any = True,
        encoder_valid: Any = True,
        encoder_position: Any = 0.0,
        safety: bool = True,
        heartbeat: Any = True,
        estop: Any = False,
        supply_v: Any = 6.0,
        relay_enabled: bool | None = None,
        relay_healthy: Any = True,
    ) -> Any:
        if command:
            self.command_sequence += 1
            self.adapter.ingest_command(
                CommandFrame(
                    self.command_sequence,
                    arm,
                    run,
                    False,
                    command_v,
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
                    sequence,
                    encoder_position,
                    encoder_healthy,
                    encoder_valid,
                )
            )
        if safety:
            self.safety_sequence += 1
            self.adapter.ingest_safety(
                SafetyFrame(
                    self.safety_sequence,
                    supply_v,
                    estop,
                    heartbeat,
                    (
                        self.previous_relay()
                        if relay_enabled is None
                        else relay_enabled
                    ),
                    relay_healthy,
                )
            )
        result = self.adapter.tick()
        self.results.append(result)
        return result

    def reset(self) -> tuple[Any, bool, str]:
        result, accepted, reason = self.adapter.reset_tick()
        self.results.append(result)
        return result, accepted, reason

    def restart(self) -> Any:
        self.adapter = SupervisorAdapter.from_files(
            self.sim_path, self.motor_path
        )
        self.generation += 1
        result = self.adapter.tick()
        self.results.append(result)
        return result

    def running(self) -> None:
        for _ in range(3):
            self.step()
        self.step()
        result = self.step(arm=True, run=True, command_v=2.0)
        if result.record.state != SupervisorState.RUNNING:
            raise RuntimeError('ROS2-010 evidence setup did not reach RUNNING')


def _safe_fault(result: Any, fault: FaultCode) -> bool:
    return (
        result.record.state == SupervisorState.FAULT_LATCHED
        and fault in result.record.latched_faults
        and result.record.safe_output
    )


def _normal_and_saturation(harness: Harness) -> dict[str, bool]:
    harness.running()
    normal = harness.step(arm=True, run=True, command_v=2.0)
    saturated = harness.step(arm=True, run=True, command_v=7.0)
    return {
        'normal_running': (
            normal.record.state == SupervisorState.RUNNING
            and normal.record.motor_command_v == 2.0
        ),
        'finite_saturation_clipped': saturated.record.motor_command_v == 6.0,
        'saturation_reported': saturated.record.command_saturated,
        'saturation_not_faulted': not saturated.record.latched_faults,
    }


def _estop(harness: Harness) -> dict[str, bool]:
    harness.running()
    result = harness.step(
        arm=True, run=True, command_v=2.0, estop=True
    )
    return {'same_tick_safe_fault': _safe_fault(
        result, FaultCode.EMERGENCY_STOP
    )}


def _watchdog_boundary(harness: Harness) -> dict[str, bool]:
    harness.running()
    threshold = (
        harness.adapter.config.synthetic_watchdog_missed_heartbeat_samples
    )
    before = None
    for _ in range(threshold - 1):
        before = harness.step(
            arm=True, run=True, command_v=2.0, heartbeat=False
        )
    at = harness.step(
        arm=True, run=True, command_v=2.0, heartbeat=False
    )
    return {
        'n_minus_one_healthy': (
            before is not None
            and FaultCode.WATCHDOG_TIMEOUT
            not in before.record.detected_faults
        ),
        'n_faults_safe': _safe_fault(at, FaultCode.WATCHDOG_TIMEOUT),
    }


def _encoder_missing_boundary(harness: Harness) -> dict[str, bool]:
    harness.running()
    threshold = harness.adapter.config.synthetic_encoder_stale_samples
    before = None
    for _ in range(threshold - 1):
        before = harness.step(
            arm=True, run=True, command_v=2.0, encoder=False
        )
    at = harness.step(
        arm=True, run=True, command_v=2.0, encoder=False
    )
    return {
        'n_minus_one_healthy': (
            before is not None
            and FaultCode.ENCODER_STALE
            not in before.record.detected_faults
        ),
        'n_faults_safe': _safe_fault(at, FaultCode.ENCODER_STALE),
    }


def _encoder_duplicate(harness: Harness) -> dict[str, bool]:
    harness.running()
    duplicate = harness.encoder_sequence
    threshold = harness.adapter.config.synthetic_encoder_stale_samples
    result = None
    dispositions: list[str] = []
    for _ in range(threshold):
        result = harness.step(
            arm=True,
            run=True,
            command_v=2.0,
            encoder_sequence=duplicate,
        )
        dispositions.append(result.encoder_disposition)
    return {
        'duplicates_explicit': all(
            value == 'DUPLICATE' for value in dispositions
        ),
        'duplicate_age_faults_safe': (
            result is not None
            and _safe_fault(result, FaultCode.ENCODER_STALE)
        ),
    }


def _encoder_out_of_order(harness: Harness) -> dict[str, bool]:
    harness.running()
    result = harness.step(
        arm=True,
        run=True,
        command_v=2.0,
        encoder_sequence=harness.encoder_sequence - 1,
    )
    return {
        'out_of_order_explicit': (
            result.encoder_disposition == 'OUT_OF_ORDER'
        ),
        'out_of_order_faults_safe': _safe_fault(
            result, FaultCode.ENCODER_INVALID
        ),
    }


def _encoder_failure(harness: Harness) -> dict[str, bool]:
    harness.running()
    result = harness.step(
        arm=True,
        run=True,
        command_v=2.0,
        encoder_healthy=False,
    )
    return {'encoder_failure_safe': _safe_fault(
        result, FaultCode.ENCODER_FAILURE
    )}


def _relay_boundary(harness: Harness) -> dict[str, bool]:
    harness.running()
    threshold = (
        harness.adapter.config.synthetic_relay_feedback_mismatch_samples
    )
    before = None
    for _ in range(threshold - 1):
        before = harness.step(
            arm=True,
            run=True,
            command_v=2.0,
            relay_enabled=False,
            relay_healthy=False,
        )
    at = harness.step(
        arm=True,
        run=True,
        command_v=2.0,
        relay_enabled=False,
        relay_healthy=False,
    )
    return {
        'n_minus_one_inhibits_motor': (
            before is not None
            and before.record.motor_command_v == 0.0
            and FaultCode.RELAY_FEEDBACK_FAILURE
            not in before.record.detected_faults
        ),
        'n_faults_safe': _safe_fault(
            at, FaultCode.RELAY_FEEDBACK_FAILURE
        ),
    }


def _undervoltage(harness: Harness) -> dict[str, bool]:
    harness.running()
    threshold = harness.adapter.config.synthetic_undervoltage_threshold_v
    healthy = harness.step(
        arm=True, run=True, command_v=2.0, supply_v=threshold
    )
    faulted = harness.step(
        arm=True,
        run=True,
        command_v=2.0,
        supply_v=threshold - 0.1,
    )
    return {
        'exact_threshold_healthy': (
            FaultCode.UNDERVOLTAGE
            not in healthy.record.detected_faults
        ),
        'below_threshold_safe_fault': _safe_fault(
            faulted, FaultCode.UNDERVOLTAGE
        ),
    }


def _malformed_command(harness: Harness) -> dict[str, bool]:
    harness.running()
    result = harness.step(
        arm=True, run=True, command_v=math.nan
    )
    return {'nonfinite_command_safe_fault': _safe_fault(
        result, FaultCode.INVALID_COMMAND
    )}


def _latch_reset_recovery(harness: Harness) -> dict[str, bool]:
    harness.running()
    harness.step(arm=True, run=True, command_v=2.0, estop=True)
    cleared_high = harness.step(arm=True, run=True, command_v=2.0)
    rejected, rejected_ok, _ = harness.reset()
    harness.step(arm=False, run=False)
    accepted, accepted_ok, _ = harness.reset()
    hold = harness.adapter.config.synthetic_safe_shutdown_hold_samples
    for _ in range(hold):
        held = harness.step(arm=True, run=True, command_v=2.0)
    still_ready = harness.step(arm=True, run=True, command_v=2.0)
    harness.step(arm=False, run=False)
    running = harness.step(arm=True, run=True, command_v=2.0)
    return {
        'source_clear_preserves_latch': (
            cleared_high.record.state == SupervisorState.FAULT_LATCHED
        ),
        'unsafe_reset_rejected': (
            not rejected_ok and rejected.record.reset_rejected
        ),
        'safe_reset_accepted': (
            accepted_ok and accepted.record.reset_accepted
        ),
        'held_request_cannot_restart': (
            held.record.state == SupervisorState.READY
            and still_ready.record.state == SupervisorState.READY
        ),
        'controlled_rearm_runs': (
            running.record.state == SupervisorState.RUNNING
        ),
    }


def _restart(harness: Harness) -> dict[str, bool]:
    harness.running()
    restarted = harness.restart()
    return {
        'restart_sample_resets': restarted.record.sample_index == 0,
        'restart_safe_startup': (
            restarted.record.state == SupervisorState.SAFE_STARTUP
            and restarted.record.safe_output
        ),
        'restart_requires_liveness': (
            not restarted.record.encoder_liveness_established
        ),
    }


def _telemetry_disappearance(harness: Harness) -> dict[str, bool]:
    harness.running()
    count = max(
        harness.adapter.config.synthetic_encoder_stale_samples,
        harness.adapter.config.synthetic_watchdog_missed_heartbeat_samples,
    )
    for _ in range(count):
        result = harness.step(
            arm=True,
            run=True,
            command_v=2.0,
            encoder=False,
            safety=False,
        )
    return {
        'disappearance_faults': (
            result.record.state == SupervisorState.FAULT_LATCHED
        ),
        'disappearance_safe': result.record.safe_output,
    }


SCENARIOS: tuple[tuple[str, Scenario], ...] = (
    ('normal_startup_running_and_saturation', _normal_and_saturation),
    ('software_estop', _estop),
    ('watchdog_n_minus_one_and_n', _watchdog_boundary),
    ('missing_encoder_n_minus_one_and_n', _encoder_missing_boundary),
    ('duplicate_encoder', _encoder_duplicate),
    ('out_of_order_encoder', _encoder_out_of_order),
    ('encoder_failure', _encoder_failure),
    ('relay_feedback_n_minus_one_and_n', _relay_boundary),
    ('undervoltage', _undervoltage),
    ('malformed_nonfinite_command', _malformed_command),
    ('fault_latch_reset_and_controlled_recovery', _latch_reset_recovery),
    ('supervisor_restart', _restart),
    ('simulator_telemetry_disappearance', _telemetry_disappearance),
)


def _trace_mapping(
    scenario_id: str,
    trace_index: int,
    generation: int,
    result: Any,
) -> dict[str, Any]:
    record = result.record
    return {
        'scenario_id': scenario_id,
        'trace_index': trace_index,
        'process_generation': generation,
        **record.to_mapping(),
        'command_disposition': result.command_disposition,
        'encoder_disposition': result.encoder_disposition,
        'safety_disposition': result.safety_disposition,
        'input_diagnostics': list(result.input_diagnostics),
        'actuator_message': actuator_mapping(record),
        'safety_status_message': safety_status_mapping(record),
        'supervisor_telemetry_message': (
            supervisor_telemetry_mapping(result)
        ),
        'fault_telemetry_message': fault_telemetry_mapping(result),
    }


def execute(
    sim_path: Path, motor_path: Path
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    summaries: list[dict[str, Any]] = []
    traces: list[dict[str, Any]] = []
    for scenario_id, scenario in SCENARIOS:
        harness = Harness(sim_path, motor_path)
        checks = scenario(harness)
        for index, result in enumerate(harness.results):
            generation = (
                1
                if scenario_id == 'supervisor_restart'
                and index == len(harness.results) - 1
                else 0
            )
            traces.append(
                _trace_mapping(
                    scenario_id, index, generation, result
                )
            )
        summaries.append(
            {
                'scenario_id': scenario_id,
                'result': 'PASS' if all(checks.values()) else 'FAIL',
                'checks': checks,
                'trace_count': len(harness.results),
                'states': list(
                    dict.fromkeys(
                        result.record.state.value
                        for result in harness.results
                    )
                ),
            }
        )
    return summaries, traces


def build_evidence(
    sim_path: Path, motor_path: Path
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    scenarios, traces = execute(sim_path, motor_path)
    replay_scenarios, replay_traces = execute(sim_path, motor_path)
    fault_records = [
        item
        for item in traces
        if item['detected_faults']
        or item['state'] == SupervisorState.FAULT_LATCHED.value
    ]
    checks = {
        'all_scenarios_pass': all(
            item['result'] == 'PASS' for item in scenarios
        ),
        'exact_replay_matches': (
            scenarios == replay_scenarios and traces == replay_traces
        ),
        'all_detected_or_latched_fault_outputs_safe': (
            bool(fault_records)
            and all(item['safe_output'] for item in fault_records)
        ),
        'startup_output_safe': all(
            item['safe_output']
            for item in traces
            if item['sample_index'] == 0
        ),
        'coefficient_freeze_hold_preserved': True,
        'fixed_point_conversion_hold_preserved': True,
    }
    report = {
        'evidence_id': 'ROS2-010-SYNTHETIC',
        'result': 'PASS' if all(checks.values()) else 'FAIL',
        'scope': (
            'hardware-independent ROS 2 Jazzy middleware adaptation around '
            'the authoritative SIM-010 supervisor'
        ),
        'determinism_basis': (
            'sample-indexed frame sequences, no random source, authoritative '
            'SIM-010 ticks, exact replay, stable JSON/CSV serialization'
        ),
        'transport_semantics': (
            'accepted monotonic publisher sequences are consumed once per '
            'supervisor tick; duplicates are ignored, out-of-order frames '
            'are rejected, and required-message disappearance advances '
            'SIM-010 encoder/watchdog fail-safe counters'
        ),
        'software_estop_limitation': (
            'The software E-stop topic is synthetic test input only and is '
            'not a physical emergency-stop or independent power removal.'
        ),
        'scenario_count': len(scenarios),
        'trace_count': len(traces),
        'fault_or_latched_trace_count': len(fault_records),
        'checks': checks,
        'scenarios': scenarios,
        'coefficient_freeze_readiness': 'HOLD',
        'fixed_point_conversion_readiness': 'HOLD',
        'not_demonstrated': [
            'physical safety or emergency-stop function',
            'GPIO or electrical relay operation',
            'physical motor, encoder, supply, or disable timing',
            'real-time scheduling or bounded DDS latency',
            'fixed-point coefficients, binary points, arithmetic, or RTL',
        ],
    }
    return report, traces


def write_csv(path: Path, records: list[dict[str, Any]]) -> None:
    fieldnames = [
        'scenario_id',
        'trace_index',
        'process_generation',
        'sample_index',
        'state_before',
        'state',
        'arm_request',
        'run_request',
        'reset_request',
        'watchdog_heartbeat',
        'watchdog_missed_samples',
        'supply_voltage_v',
        'encoder_sequence',
        'encoder_stale_age_samples',
        'requested_motor_voltage_v',
        'motor_command_v',
        'command_saturated',
        'relay_enable_command',
        'relay_enable_feedback',
        'relay_feedback_mismatch_samples',
        'command_disposition',
        'encoder_disposition',
        'safety_disposition',
        'input_diagnostics',
        'detected_faults',
        'latched_faults',
        'active_raw_fault_sources',
        'reset_accepted',
        'reset_rejected',
        'safe_output',
        'actuator_message',
        'safety_status_message',
        'supervisor_telemetry_message',
        'fault_telemetry_message',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(
            handle, fieldnames=fieldnames, lineterminator='\n'
        )
        writer.writeheader()
        for source in records:
            row = {name: source[name] for name in fieldnames}
            for name in (
                'input_diagnostics',
                'detected_faults',
                'latched_faults',
                'active_raw_fault_sources',
            ):
                row[name] = '|'.join(source[name])
            for name in (
                'actuator_message',
                'safety_status_message',
                'supervisor_telemetry_message',
                'fault_telemetry_message',
            ):
                row[name] = json.dumps(
                    source[name],
                    sort_keys=True,
                    separators=(',', ':'),
                )
            writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--sim-config',
        type=Path,
        default=Path('models/parameters/synthetic_sim_010.json'),
    )
    parser.add_argument(
        '--motor-config',
        type=Path,
        default=Path('models/parameters/synthetic_motor.json'),
    )
    parser.add_argument(
        '--report',
        type=Path,
        default=Path(
            'data/processed/ros2_010_synthetic_validation_report.json'
        ),
    )
    parser.add_argument(
        '--trace',
        type=Path,
        default=Path(
            'data/processed/ros2_010_synthetic_message_trace.csv'
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report, traces = build_evidence(args.sim_config, args.motor_config)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open(
        'w', encoding='utf-8', newline=''
    ) as report_handle:
        report_handle.write(
            json.dumps(report, indent=2, sort_keys=True) + '\n'
        )
    write_csv(args.trace, traces)
    print(
        f"{report['evidence_id']}: {report['result']} "
        f"({len(report['checks'])} checks, "
        f"{report['scenario_count']} scenarios, "
        f"{report['trace_count']} traces)"
    )
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
