"""Generate deterministic SIM-010 synthetic software-in-the-loop evidence."""

from __future__ import annotations

import argparse
import csv
import json
from collections.abc import Mapping
from numbers import Real
from pathlib import Path
from typing import Any, Callable

import numpy as np

from models.dc_motor import (
    MotorParameters,
    continuous_dc_motor_model,
    discretize_zero_order_hold,
)
from models.sil import (
    FaultCode,
    SupervisorInputs,
    SupervisorState,
    SupervisorTestBench,
    SyntheticSilConfig,
    TelemetryRecord,
)

ScenarioRunner = Callable[
    [SyntheticSilConfig, Any],
    tuple[SupervisorTestBench, dict[str, bool]],
]


def _finite_positive_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be numeric")
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise ValueError(f"{name} must be representable") from error
    if not np.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and above zero")
    return result


def load_json(path: Path) -> dict[str, Any]:
    """Load one JSON object."""

    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def build_motor_model(motor_payload: dict[str, Any]) -> Any:
    """Build the existing discrete synthetic motor model."""

    if not isinstance(motor_payload, Mapping):
        raise ValueError("synthetic motor configuration must be an object")
    required_text = ("parameter_set_id", "status", "provenance")
    for field_name in required_text:
        value = motor_payload.get(field_name)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(
                f"synthetic motor {field_name} must be a nonempty string"
            )
    if motor_payload["status"] != "synthetic_software_fixture":
        raise ValueError("synthetic motor status is invalid")
    sample_period = _finite_positive_number(
        motor_payload.get("sample_period_s"),
        "synthetic motor sample_period_s",
    )
    parameter_payload = motor_payload.get("parameters")
    if not isinstance(parameter_payload, Mapping):
        raise ValueError("synthetic motor parameters must be an object")
    required_parameters = (
        "armature_resistance_ohm",
        "armature_inductance_h",
        "torque_constant_nm_per_a",
        "back_emf_constant_v_per_rad_s",
        "rotor_inertia_kg_m2",
        "viscous_friction_nm_s_per_rad",
    )
    validated_parameters: dict[str, float] = {}
    for field_name in required_parameters:
        validated_parameters[field_name] = _finite_positive_number(
            parameter_payload.get(field_name),
            f"synthetic motor {field_name}",
        )
    parameters = MotorParameters.from_mapping(validated_parameters)
    return discretize_zero_order_hold(
        continuous_dc_motor_model(parameters),
        sample_period,
    )


def nominal_inputs(
    config: SyntheticSilConfig,
    **overrides: Any,
) -> SupervisorInputs:
    """Return one healthy synthetic input sample with explicit overrides."""

    values: dict[str, Any] = {
        "supply_voltage_v": (
            config.synthetic_nominal_supply_voltage_v
        ),
        "requested_motor_voltage_v": 0.0,
        "arm_request": False,
        "run_request": False,
        "shutdown_request": False,
        "reset_request": False,
        "watchdog_heartbeat": True,
        "estop_active": False,
        "encoder_stale": False,
        "encoder_failed": False,
        "encoder_invalid": False,
        "relay_feedback_failed": False,
    }
    values.update(overrides)
    return SupervisorInputs(**values)


def bring_to_running(
    bench: SupervisorTestBench,
    config: SyntheticSilConfig,
) -> None:
    """Complete safe startup and make one explicit run transition."""

    maximum_startup_samples = (
        config.synthetic_safe_startup_healthy_samples + 2
    )
    for _ in range(maximum_startup_samples):
        bench.step(nominal_inputs(config))
        if bench.state == SupervisorState.READY:
            break
    if bench.state != SupervisorState.READY:
        raise RuntimeError("SIM-010 scenario setup did not reach READY")
    bench.step(nominal_inputs(config))
    bench.step(
        nominal_inputs(
            config,
            arm_request=True,
            run_request=True,
            requested_motor_voltage_v=(
                config.synthetic_normal_run_command_voltage_v
            ),
        )
    )
    if bench.state != SupervisorState.RUNNING:
        raise RuntimeError("SIM-010 scenario setup did not reach RUNNING")


def _new_bench(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> SupervisorTestBench:
    return SupervisorTestBench(config, motor_model)


def _fault_safe(records: list[TelemetryRecord]) -> bool:
    relevant = [
        record
        for record in records
        if record.detected_faults
        or record.state == SupervisorState.FAULT_LATCHED
    ]
    return bool(relevant) and all(record.safe_output for record in relevant)


def _has_latched_event(
    bench: SupervisorTestBench,
    fault: FaultCode,
) -> bool:
    return any(
        event.code == fault.value and event.action == "LATCHED"
        for event in bench.faults.events
    )


def _normal_startup_and_operation(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> tuple[SupervisorTestBench, dict[str, bool]]:
    bench = _new_bench(config, motor_model)
    bring_to_running(bench, config)
    for _ in range(3):
        bench.step(
            nominal_inputs(
                config,
                arm_request=True,
                run_request=True,
                requested_motor_voltage_v=(
                    config.synthetic_normal_run_command_voltage_v
                ),
            )
        )
    bench.step(
        nominal_inputs(
            config,
            shutdown_request=True,
        )
    )
    for _ in range(config.synthetic_safe_shutdown_hold_samples):
        bench.step(nominal_inputs(config))
    states = {record.state for record in bench.telemetry.records}
    checks = {
        "safe_startup_observed": (
            SupervisorState.SAFE_STARTUP in states
        ),
        "ready_observed": SupervisorState.READY in states,
        "running_observed": SupervisorState.RUNNING in states,
        "safe_shutdown_observed": (
            SupervisorState.SAFE_SHUTDOWN in states
        ),
        "running_command_nonzero": any(
            record.state == SupervisorState.RUNNING
            and record.motor_command_v != 0.0
            and record.relay_enable_command
            for record in bench.telemetry.records
        ),
        "startup_and_shutdown_outputs_safe": all(
            record.safe_output
            for record in bench.telemetry.records
            if record.state
            in {
                SupervisorState.SAFE_STARTUP,
                SupervisorState.SAFE_SHUTDOWN,
            }
        ),
        "no_fault_events": not bench.faults.events,
        "final_state_ready": bench.state == SupervisorState.READY,
    }
    return bench, checks


def _run_fault_injection(
    config: SyntheticSilConfig,
    motor_model: Any,
    fault: FaultCode,
    injection_samples: list[dict[str, Any]],
) -> tuple[SupervisorTestBench, dict[str, bool]]:
    bench = _new_bench(config, motor_model)
    bring_to_running(bench, config)
    for overrides in injection_samples:
        input_values = {
            "arm_request": True,
            "run_request": True,
            "requested_motor_voltage_v": (
                config.synthetic_normal_run_command_voltage_v
            ),
            **overrides,
        }
        bench.step(nominal_inputs(config, **input_values))
    checks = {
        "expected_fault_latched": _has_latched_event(bench, fault),
        "fault_latched_state_reached": (
            bench.state == SupervisorState.FAULT_LATCHED
        ),
        "fault_records_force_or_preserve_safe_output": _fault_safe(
            bench.telemetry.records
        ),
        "final_relay_command_disabled": (
            not bench.telemetry.records[-1].relay_enable_command
        ),
        "final_motor_command_zero": (
            bench.telemetry.records[-1].motor_command_v == 0.0
        ),
    }
    return bench, checks


def _estop_activation(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> tuple[SupervisorTestBench, dict[str, bool]]:
    return _run_fault_injection(
        config,
        motor_model,
        FaultCode.EMERGENCY_STOP,
        [{"estop_active": True}],
    )


def _watchdog_timeout(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> tuple[SupervisorTestBench, dict[str, bool]]:
    return _run_fault_injection(
        config,
        motor_model,
        FaultCode.WATCHDOG_TIMEOUT,
        [
            {"watchdog_heartbeat": False}
            for _ in range(
                config.synthetic_watchdog_missed_heartbeat_samples
            )
        ],
    )


def _stale_encoder_telemetry(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> tuple[SupervisorTestBench, dict[str, bool]]:
    return _run_fault_injection(
        config,
        motor_model,
        FaultCode.ENCODER_STALE,
        [
            {"encoder_stale": True}
            for _ in range(config.synthetic_encoder_stale_samples)
        ],
    )


def _encoder_failure(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> tuple[SupervisorTestBench, dict[str, bool]]:
    return _run_fault_injection(
        config,
        motor_model,
        FaultCode.ENCODER_FAILURE,
        [{"encoder_failed": True}],
    )


def _relay_feedback_failure(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> tuple[SupervisorTestBench, dict[str, bool]]:
    return _run_fault_injection(
        config,
        motor_model,
        FaultCode.RELAY_FEEDBACK_FAILURE,
        [
            {"relay_feedback_failed": True}
            for _ in range(
                config.synthetic_relay_feedback_mismatch_samples
            )
        ],
    )


def _undervoltage(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> tuple[SupervisorTestBench, dict[str, bool]]:
    return _run_fault_injection(
        config,
        motor_model,
        FaultCode.UNDERVOLTAGE,
        [
            {
                "supply_voltage_v": (
                    config.synthetic_undervoltage_threshold_v - 0.1
                )
            }
        ],
    )


def _command_voltage_saturation(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> tuple[SupervisorTestBench, dict[str, bool]]:
    bench = _new_bench(config, motor_model)
    bring_to_running(bench, config)
    limit = config.synthetic_command_voltage_abs_max_v
    record = bench.step(
        nominal_inputs(
            config,
            arm_request=True,
            run_request=True,
            requested_motor_voltage_v=limit + 0.1,
        )
    )
    checks = {
        "finite_overlimit_command_clipped": record.motor_command_v == limit,
        "saturation_reported": record.command_saturated,
        "ordinary_saturation_not_faulted": (
            not record.detected_faults
            and not record.latched_faults
            and not bench.faults.events
        ),
        "running_state_preserved": record.state == SupervisorState.RUNNING,
        "relay_remains_enabled": (
            record.relay_enable_command
            and record.relay_enable_feedback
        ),
    }
    return bench, checks


def _fault_latching(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> tuple[SupervisorTestBench, dict[str, bool]]:
    bench, checks = _run_fault_injection(
        config,
        motor_model,
        FaultCode.UNDERVOLTAGE,
        [
            {
                "supply_voltage_v": (
                    config.synthetic_undervoltage_threshold_v - 0.1
                )
            }
        ],
    )
    for _ in range(3):
        bench.step(
            nominal_inputs(
                config,
                arm_request=True,
                run_request=True,
                requested_motor_voltage_v=(
                    config.synthetic_normal_run_command_voltage_v
                ),
            )
        )
    checks.update(
        {
            "source_clear_does_not_clear_latch": (
                bench.state == SupervisorState.FAULT_LATCHED
                and FaultCode.UNDERVOLTAGE in bench.latched_faults
            ),
            "latched_interval_preserves_safe_output": all(
                record.safe_output
                for record in bench.telemetry.records
                if record.state == SupervisorState.FAULT_LATCHED
            ),
        }
    )
    return bench, checks


def _rejected_unsafe_restart(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> tuple[SupervisorTestBench, dict[str, bool]]:
    bench, checks = _run_fault_injection(
        config,
        motor_model,
        FaultCode.EMERGENCY_STOP,
        [{"estop_active": True}],
    )
    record = bench.step(
        nominal_inputs(
            config,
            arm_request=True,
            run_request=True,
            reset_request=True,
            requested_motor_voltage_v=(
                config.synthetic_normal_run_command_voltage_v
            ),
        )
    )
    checks.update(
        {
            "unsafe_reset_rejected": record.reset_rejected,
            "unsafe_reset_not_accepted": not record.reset_accepted,
            "restart_did_not_reach_running": (
                bench.state == SupervisorState.FAULT_LATCHED
            ),
            "rejected_restart_output_safe": record.safe_output,
        }
    )
    return bench, checks


def _successful_controlled_recovery(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> tuple[SupervisorTestBench, dict[str, bool]]:
    bench, checks = _run_fault_injection(
        config,
        motor_model,
        FaultCode.EMERGENCY_STOP,
        [{"estop_active": True}],
    )
    reset_record = bench.step(
        nominal_inputs(
            config,
            reset_request=True,
        )
    )
    for _ in range(config.synthetic_safe_shutdown_hold_samples):
        bench.step(nominal_inputs(config))
    ready_before_rearm = bench.state == SupervisorState.READY
    disarmed_ready_record = bench.step(nominal_inputs(config))
    rearm_record = bench.step(
        nominal_inputs(
            config,
            arm_request=True,
            run_request=True,
            requested_motor_voltage_v=(
                config.synthetic_normal_run_command_voltage_v
            ),
        )
    )
    checks.update(
        {
            "explicit_reset_accepted": reset_record.reset_accepted,
            "reset_enters_safe_shutdown": (
                reset_record.state == SupervisorState.SAFE_SHUTDOWN
                and reset_record.safe_output
            ),
            "ready_reached_before_rearm": ready_before_rearm,
            "complete_disarmed_ready_sample_observed": (
                disarmed_ready_record.state == SupervisorState.READY
                and disarmed_ready_record.safe_output
            ),
            "latched_faults_cleared": not bench.latched_faults,
            "new_arm_and_run_required": (
                rearm_record.state == SupervisorState.RUNNING
                and rearm_record.motor_command_v != 0.0
                and rearm_record.relay_enable_command
            ),
        }
    )
    return bench, checks


_SCENARIOS: tuple[tuple[str, str, ScenarioRunner], ...] = (
    (
        "normal_startup_and_operation",
        "Safe startup, ready, running, safe shutdown, and return to ready.",
        _normal_startup_and_operation,
    ),
    (
        "estop_activation",
        "Synthetic emergency-stop assertion while running.",
        _estop_activation,
    ),
    (
        "watchdog_timeout",
        "Synthetic consecutive missed heartbeats at the configured threshold.",
        _watchdog_timeout,
    ),
    (
        "stale_encoder_telemetry",
        "Synthetic encoder sequence held stale at the configured threshold.",
        _stale_encoder_telemetry,
    ),
    (
        "encoder_failure",
        "Synthetic encoder health failure while running.",
        _encoder_failure,
    ),
    (
        "relay_feedback_failure",
        "Synthetic relay feedback mismatch at the configured threshold.",
        _relay_feedback_failure,
    ),
    (
        "undervoltage",
        "Synthetic supply sample below the configured undervoltage threshold.",
        _undervoltage,
    ),
    (
        "command_voltage_saturation",
        "Finite synthetic voltage clips at the configured absolute limit "
        "without latching a fault.",
        _command_voltage_saturation,
    ),
    (
        "fault_latching",
        "Fault source clears but the latch and safe outputs persist.",
        _fault_latching,
    ),
    (
        "rejected_unsafe_restart",
        "Reset is rejected while arm and run remain asserted.",
        _rejected_unsafe_restart,
    ),
    (
        "successful_controlled_recovery",
        "Source clear, explicit reset, safe shutdown, ready, and new re-arm.",
        _successful_controlled_recovery,
    ),
)


def execute_scenarios(
    config: SyntheticSilConfig,
    motor_model: Any,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Execute every required deterministic scenario."""

    scenario_records: list[dict[str, Any]] = []
    trace_records: list[dict[str, Any]] = []
    for scenario_id, description, runner in _SCENARIOS:
        bench, checks = runner(config, motor_model)
        mappings = [record.to_mapping() for record in bench.telemetry.records]
        for mapping in mappings:
            trace_records.append({"scenario_id": scenario_id, **mapping})
        scenario_records.append(
            {
                "scenario_id": scenario_id,
                "description": description,
                "result": "PASS" if all(checks.values()) else "FAIL",
                "checks": checks,
                "sample_count": len(mappings),
                "states_visited": list(
                    dict.fromkeys(mapping["state"] for mapping in mappings)
                ),
                "final_state": bench.state.value,
                "latched_faults_at_end": [
                    fault.value
                    for fault in sorted(
                        bench.latched_faults,
                        key=lambda item: item.value,
                    )
                ],
                "fault_events": [
                    event.to_mapping() for event in bench.faults.events
                ],
                "fault_or_latched_record_count": sum(
                    bool(mapping["detected_faults"])
                    or mapping["state"]
                    == SupervisorState.FAULT_LATCHED.value
                    for mapping in mappings
                ),
                "all_fault_or_latched_records_safe": all(
                    mapping["safe_output"]
                    for mapping in mappings
                    if mapping["detected_faults"]
                    or mapping["state"]
                    == SupervisorState.FAULT_LATCHED.value
                ),
            }
        )
    return scenario_records, trace_records


def build_validation_evidence(
    sim_config_payload: dict[str, Any],
    motor_payload: dict[str, Any],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Run SIM-010 twice and build deterministic machine-readable evidence."""

    config = SyntheticSilConfig.from_mapping(sim_config_payload)
    motor_model = build_motor_model(motor_payload)
    if (
        motor_payload["parameter_set_id"]
        != config.synthetic_motor_parameter_set_id
    ):
        raise ValueError("SIM-010 synthetic motor parameter-set ID mismatch")
    scenarios, traces = execute_scenarios(config, motor_model)
    replay_scenarios, replay_traces = execute_scenarios(config, motor_model)
    required_ids = [item[0] for item in _SCENARIOS]
    observed_ids = [item["scenario_id"] for item in scenarios]
    fault_trace_records = [
        record
        for record in traces
        if record["detected_faults"]
        or record["state"] == SupervisorState.FAULT_LATCHED.value
    ]
    checks = {
        "all_required_scenarios_executed": observed_ids == required_ids,
        "scenario_identifiers_unique": (
            len(observed_ids) == len(set(observed_ids))
        ),
        "all_scenarios_pass": all(
            item["result"] == "PASS" for item in scenarios
        ),
        "exact_replay_matches": (
            scenarios == replay_scenarios and traces == replay_traces
        ),
        "all_detected_or_latched_fault_records_safe": (
            bool(fault_trace_records)
            and all(record["safe_output"] for record in fault_trace_records)
        ),
        "coefficient_freeze_hold_preserved": (
            config.coefficient_freeze_readiness == "HOLD"
        ),
        "fixed_point_conversion_hold_preserved": (
            config.fixed_point_conversion_readiness == "HOLD"
        ),
    }
    report = {
        "evidence_id": config.test_bench_id,
        "result": "PASS" if all(checks.values()) else "FAIL",
        "scope": (
            "deterministic hardware-independent synthetic supervisor "
            "software-in-the-loop validation"
        ),
        "status": config.status,
        "provenance": config.provenance,
        "determinism_basis": (
            "sample-indexed execution, no wall clock, no random source, "
            "ordered in-memory channels, finite telemetry normalized to 12 "
            "significant decimal digits for serialization, exact full-suite "
            "replay, and CI regeneration diff enforcement"
        ),
        "encoder_stale_definition": (
            "The first valid sequence establishes age zero. Each subsequent "
            "valid repeated sequence increments the consecutive no-advance "
            "interval age; N-1 is healthy and configured age N faults. A "
            "valid sequence transition resets age to zero and establishes "
            "startup liveness."
        ),
        "recovery_interlock": (
            "Reset requires all active raw sources clear and arm/run both "
            "low. Requests in SAFE_SHUTDOWN or held into READY are ignored; "
            "one complete disarmed READY sample must precede a later new "
            "sample with arm/run both high."
        ),
        "command_saturation_policy": (
            "Finite commands inside or exactly at the synthetic absolute "
            "limit pass unchanged. Finite over-limit commands clip and set "
            "saturation telemetry without latching. Malformed or non-finite "
            "commands latch INVALID_COMMAND and force safe outputs."
        ),
        "synthetic_configuration": {
            "plant": sim_config_payload["synthetic_plant"],
            "operating_values": sim_config_payload[
                "synthetic_operating_values"
            ],
            "operating_limits": sim_config_payload[
                "synthetic_operating_limits"
            ],
            "fault_thresholds": sim_config_payload[
                "synthetic_fault_thresholds"
            ],
            "interfaces": sim_config_payload["synthetic_interfaces"],
            "motor_sample_period_s": motor_payload["sample_period_s"],
            "motor_parameters": motor_payload["parameters"],
        },
        "simulated_interfaces": [
            "dc_motor",
            "encoder",
            "h_bridge_command",
            "relay_enable_and_feedback",
            "emergency_stop",
            "watchdog",
            "supply_voltage_monitor",
            "telemetry_channel",
            "fault_channel",
        ],
        "supervisor_states": [state.value for state in SupervisorState],
        "scenario_count": len(scenarios),
        "telemetry_record_count": len(traces),
        "fault_or_latched_record_count": len(fault_trace_records),
        "checks": checks,
        "scenarios": scenarios,
        "coefficient_freeze_readiness": (
            config.coefficient_freeze_readiness
        ),
        "fixed_point_conversion_readiness": (
            config.fixed_point_conversion_readiness
        ),
        "acceptance_limitation": (
            "SIM-010 validates deterministic software behaviour only. All "
            "plant values, operating values, limits, and fault thresholds "
            "are synthetic. It is not physical, ROS 2 runtime, GPIO, timing, "
            "relay, H-bridge, encoder, supply, E-stop, or watchdog validation."
        ),
        "not_demonstrated": [
            "physical plant or component behaviour",
            "physical fault-detection thresholds or disable latency",
            "ROS 2 executor, process, transport, or scheduling behaviour",
            "GPIO or other electrical interfaces",
            "hardware-in-the-loop operation",
            "fixed-point coefficients, formats, or arithmetic",
            "FPGA RTL",
        ],
    }
    return report, traces


def write_trace_csv(path: Path, records: list[dict[str, Any]]) -> None:
    """Write the deterministic sample trace with stable unit-bearing fields."""

    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "scenario_id",
        "sample_index",
        "state_before",
        "state",
        "arm_request",
        "run_request",
        "shutdown_request",
        "reset_request",
        "estop_active",
        "watchdog_heartbeat",
        "watchdog_missed_samples",
        "supply_voltage_v",
        "supply_healthy",
        "encoder_position_rad",
        "encoder_sequence",
        "encoder_healthy",
        "encoder_valid",
        "encoder_liveness_established",
        "encoder_stale_age_samples",
        "requested_motor_voltage_v",
        "motor_command_v",
        "command_saturated",
        "relay_enable_command",
        "relay_enable_feedback",
        "relay_feedback_mismatch_samples",
        "active_raw_fault_sources",
        "invalid_input_fields",
        "detected_faults",
        "latched_faults",
        "reset_accepted",
        "reset_rejected",
        "motor_position_rad",
        "motor_speed_rad_s",
        "motor_current_a",
        "safe_output",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
            lineterminator="\n",
        )
        writer.writeheader()
        for record in records:
            csv_record = dict(record)
            csv_record["detected_faults"] = "|".join(
                record["detected_faults"]
            )
            csv_record["latched_faults"] = "|".join(
                record["latched_faults"]
            )
            csv_record["active_raw_fault_sources"] = "|".join(
                record["active_raw_fault_sources"]
            )
            csv_record["invalid_input_fields"] = "|".join(
                record["invalid_input_fields"]
            )
            writer.writerow(csv_record)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sim-config",
        type=Path,
        default=Path("models/parameters/synthetic_sim_010.json"),
    )
    parser.add_argument(
        "--motor-parameters",
        type=Path,
        default=Path("models/parameters/synthetic_motor.json"),
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path(
            "data/processed/sim_010_synthetic_validation_report.json"
        ),
    )
    parser.add_argument(
        "--trace",
        type=Path,
        default=Path(
            "data/processed/sim_010_synthetic_scenario_trace.csv"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report, traces = build_validation_evidence(
        load_json(args.sim_config),
        load_json(args.motor_parameters),
    )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_trace_csv(args.trace, traces)
    print(
        f"{report['evidence_id']}: {report['result']} "
        f"({sum(report['checks'].values())}/{len(report['checks'])} checks, "
        f"{report['scenario_count']} scenarios, "
        f"{report['telemetry_record_count']} samples)"
    )
    print(f"report: {args.report}")
    print(f"trace:  {args.trace}")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
