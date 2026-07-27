"""Generate reproducible MODEL-020 synthetic robustness evidence."""

from __future__ import annotations

import argparse
import csv
import json
import platform
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scipy

from models.control import (
    continuous_poles_to_discrete,
    design_observer_controller,
    simulate_observer_feedback,
)
from models.dc_motor import (
    MotorParameters,
    continuous_dc_motor_model,
    discretize_zero_order_hold,
)
from models.robustness import (
    RobustnessResult,
    build_scenarios_from_mapping,
    run_robustness_scenario,
)


def load_json(path: Path) -> dict[str, Any]:
    """Load and validate one JSON object."""

    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def build_report(
    motor_file: Path,
    controller_file: Path,
    robustness_file: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Run every configured scenario and return report plus tabular records."""

    motor_payload = load_json(motor_file)
    controller_payload = load_json(controller_file)
    robustness_payload = load_json(robustness_file)
    nominal_parameters = MotorParameters.from_mapping(motor_payload["parameters"])
    sample_period_s = float(motor_payload["sample_period_s"])
    nominal_model = discretize_zero_order_hold(
        continuous_dc_motor_model(nominal_parameters),
        sample_period_s,
    )
    controller_poles = continuous_poles_to_discrete(
        controller_payload["controller"]["continuous_poles_rad_s"],
        sample_period_s,
    )
    observer_poles = continuous_poles_to_discrete(
        controller_payload["observer"]["continuous_poles_rad_s"],
        sample_period_s,
    )
    design = design_observer_controller(
        nominal_model,
        controller_poles,
        observer_poles,
    )

    simulation = robustness_payload["simulation"]
    sample_count = int(
        round(float(simulation["duration_s"]) / sample_period_s)
    )
    if sample_count < 2:
        raise ValueError("robustness simulation must contain at least two samples")
    reference_rad = float(simulation["reference_position_rad"])
    references = np.full(sample_count, reference_rad, dtype=np.float64)
    loads = np.zeros(sample_count, dtype=np.float64)
    random_seed = int(robustness_payload["random_seed"])
    scenarios = build_scenarios_from_mapping(robustness_payload)
    envelope = robustness_payload["development_integrity_envelope"]

    records: list[dict[str, Any]] = []
    results: dict[str, RobustnessResult] = {}
    repeatable = True
    for scenario in scenarios:
        result, metrics, spectral_radius = run_robustness_scenario(
            nominal_parameters,
            nominal_model,
            design,
            scenario,
            references,
            loads,
            random_seed,
        )
        replay, replay_metrics, replay_radius = run_robustness_scenario(
            nominal_parameters,
            nominal_model,
            design,
            scenario,
            references,
            loads,
            random_seed,
        )
        repeatable &= (
            np.array_equal(result.true_states, replay.true_states)
            and np.array_equal(
                result.measured_positions_rad,
                replay.measured_positions_rad,
            )
            and metrics == replay_metrics
            and spectral_radius == replay_radius
        )
        results[scenario.scenario_id] = result
        parameter_name = ""
        parameter_scale: float | None = None
        if len(scenario.parameter_multipliers) == 1:
            parameter_name, parameter_scale = next(
                iter(scenario.parameter_multipliers.items())
            )
        inside_envelope = bool(
            metrics["all_values_finite"]
            and metrics["peak_absolute_position_rad"]
            <= float(envelope["peak_absolute_position_rad_max"])
            and metrics["peak_absolute_speed_rad_s"]
            <= float(envelope["peak_absolute_speed_rad_s_max"])
            and metrics["peak_absolute_current_a"]
            <= float(envelope["peak_absolute_current_a_max"])
        )
        voltage_respected = bool(
            metrics["maximum_absolute_applied_voltage_v"]
            <= scenario.voltage_limit_v + 1e-12
        )
        records.append(
            {
                "scenario_id": scenario.scenario_id,
                "category": scenario.category,
                "description": scenario.description,
                "parameter_name": parameter_name,
                "parameter_scale": parameter_scale,
                "parameter_multipliers": dict(
                    sorted(scenario.parameter_multipliers.items())
                ),
                "measurement_noise_std_rad": scenario.measurement_noise_std_rad,
                "encoder_counts_per_revolution": (
                    scenario.encoder_counts_per_revolution
                ),
                "control_delay_samples": scenario.control_delay_samples,
                "voltage_limit_v": scenario.voltage_limit_v,
                "zero_delay_linearized_spectral_radius": spectral_radius,
                "zero_delay_linearization_stable": bool(
                    spectral_radius
                    <= float(envelope["zero_delay_spectral_radius_max"])
                ),
                "voltage_limit_respected": voltage_respected,
                "inside_development_integrity_envelope": inside_envelope,
                **metrics,
            }
        )

    nominal_result = results["nominal"]
    baseline = simulate_observer_feedback(
        nominal_model,
        design,
        references,
        loads,
        float(simulation["nominal_voltage_limit_v"]),
    )
    nominal_regression = bool(
        np.array_equal(nominal_result.true_states, baseline.true_states)
        and np.array_equal(
            nominal_result.estimated_states,
            baseline.estimated_states,
        )
        and np.array_equal(
            nominal_result.applied_voltages_v,
            baseline.applied_voltages_v,
        )
    )
    checks = {
        "scenario_identifiers_unique": (
            len(records) == len({record["scenario_id"] for record in records})
        ),
        "configured_scenario_count_executed": len(records) == len(scenarios),
        "seeded_results_exactly_repeatable": repeatable,
        "nominal_model_010_regression_exact": nominal_regression,
        "all_scenarios_finite": all(
            record["all_values_finite"] for record in records
        ),
        "all_voltage_limits_respected": all(
            record["voltage_limit_respected"] for record in records
        ),
        "all_zero_delay_linearizations_stable": all(
            record["zero_delay_linearization_stable"] for record in records
        ),
        "all_responses_inside_development_integrity_envelope": all(
            record["inside_development_integrity_envelope"]
            for record in records
        ),
    }
    worst_tracking = max(
        records,
        key=lambda record: record["rms_tracking_error_rad"],
    )
    worst_estimation = max(
        records,
        key=lambda record: record["rms_position_estimation_error_rad"],
    )
    smallest_stability_margin = max(
        records,
        key=lambda record: record["zero_delay_linearized_spectral_radius"],
    )
    report = {
        "evidence_id": robustness_payload["analysis_id"],
        "scope": (
            "deterministic synthetic floating-point robustness and "
            "uncertainty development analysis"
        ),
        "result": "PASS" if all(checks.values()) else "FAIL",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "software_environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "matplotlib": matplotlib.__version__,
        },
        "status": robustness_payload["status"],
        "provenance": robustness_payload["provenance"],
        "random_seed": random_seed,
        "parameter_set_id": motor_payload["parameter_set_id"],
        "controller_design_id": controller_payload["design_id"],
        "sample_period_s": sample_period_s,
        "simulation": simulation,
        "development_integrity_envelope": envelope,
        "scenario_count": len(records),
        "scenario_categories": {
            category: sum(
                record["category"] == category for record in records
            )
            for category in sorted({record["category"] for record in records})
        },
        "checks": checks,
        "worst_case_descriptors": {
            "rms_tracking_error": {
                "scenario_id": worst_tracking["scenario_id"],
                "value_rad": worst_tracking["rms_tracking_error_rad"],
            },
            "rms_position_estimation_error": {
                "scenario_id": worst_estimation["scenario_id"],
                "value_rad": worst_estimation[
                    "rms_position_estimation_error_rad"
                ],
            },
            "largest_zero_delay_linearized_spectral_radius": {
                "scenario_id": smallest_stability_margin["scenario_id"],
                "value": smallest_stability_margin[
                    "zero_delay_linearized_spectral_radius"
                ],
            },
        },
        "scenarios": records,
        "interpretation": [
            (
                "Passing checks establish repeatable software behavior only "
                "inside the explicitly synthetic development envelope."
            ),
            (
                "Scenario performance metrics are descriptive; they are not "
                "final physical acceptance thresholds."
            ),
            (
                "The zero-delay spectral radius excludes nonlinear and delayed "
                "effects, which are exercised separately in time-domain runs."
            ),
        ],
        "not_demonstrated": [
            "identified parameter distributions or physical tolerances",
            "hardware controller or observer acceptance",
            "backlash, Coulomb friction, thermal drift, or PWM ripple",
            "persistent-load rejection",
            "fixed-point coefficient, state, or arithmetic conversion",
            "quantized closed-loop stability",
            "Raspberry Pi or ROS 2 timing",
            "FPGA RTL equivalence",
            "hardware-in-the-loop or safe physical operation",
        ],
        "acceptance_limitation": (
            "MODEL-020-SYNTHETIC is not final robustness acceptance evidence. "
            "The uncertainty ranges, sensor model, delays, supply limits, "
            "integrity envelope, plant, and controller are synthetic fixtures."
        ),
    }
    return report, records


def write_results_csv(path: Path, records: list[dict[str, Any]]) -> None:
    """Write one unit-bearing summary row per scenario."""

    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "scenario_id",
        "category",
        "parameter_name",
        "parameter_scale",
        "measurement_noise_std_rad",
        "encoder_counts_per_revolution",
        "control_delay_samples",
        "voltage_limit_v",
        "zero_delay_linearized_spectral_radius",
        "zero_delay_linearization_stable",
        "rms_tracking_error_rad",
        "tail_mean_absolute_tracking_error_rad",
        "peak_absolute_position_rad",
        "peak_absolute_speed_rad_s",
        "peak_absolute_current_a",
        "rms_position_estimation_error_rad",
        "maximum_absolute_applied_voltage_v",
        "saturated_sample_count",
        "all_values_finite",
        "voltage_limit_respected",
        "inside_development_integrity_envelope",
        "description",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
            lineterminator="\n",
            extrasaction="ignore",
        )
        writer.writeheader()
        writer.writerows(records)


def write_parameter_sweep_plot(
    path: Path,
    records: list[dict[str, Any]],
) -> None:
    """Plot one-at-a-time tracking error and linearized spectral radius."""

    path.parent.mkdir(parents=True, exist_ok=True)
    nominal = next(record for record in records if record["scenario_id"] == "nominal")
    parameter_records = [
        record for record in records if record["category"] == "plant_parameter"
    ]
    parameter_names = sorted(
        {record["parameter_name"] for record in parameter_records}
    )
    short_names = {
        "armature_resistance_ohm": "R",
        "armature_inductance_h": "L",
        "torque_constant_nm_per_a": "Kt",
        "back_emf_constant_v_per_rad_s": "Ke",
        "rotor_inertia_kg_m2": "J",
        "viscous_friction_nm_s_per_rad": "b",
    }
    figure, axes = plt.subplots(1, 2, figsize=(11.0, 4.5))
    for parameter_name in parameter_names:
        selected = sorted(
            (
                record
                for record in parameter_records
                if record["parameter_name"] == parameter_name
            ),
            key=lambda record: record["parameter_scale"],
        )
        scales = [selected[0]["parameter_scale"], 1.0, selected[1]["parameter_scale"]]
        tracking = [
            selected[0]["tail_mean_absolute_tracking_error_rad"],
            nominal["tail_mean_absolute_tracking_error_rad"],
            selected[1]["tail_mean_absolute_tracking_error_rad"],
        ]
        radii = [
            selected[0]["zero_delay_linearized_spectral_radius"],
            nominal["zero_delay_linearized_spectral_radius"],
            selected[1]["zero_delay_linearized_spectral_radius"],
        ]
        label = short_names.get(parameter_name, parameter_name)
        axes[0].plot(scales, tracking, marker="o", label=label)
        axes[1].plot(scales, radii, marker="o", label=label)

    axes[0].set_yscale("log")
    axes[0].set_title("Tail mean absolute tracking error")
    axes[0].set_xlabel("Synthetic parameter multiplier")
    axes[0].set_ylabel("Error (rad, log scale)")
    axes[1].set_title("Zero-delay augmented spectral radius")
    axes[1].set_xlabel("Synthetic parameter multiplier")
    axes[1].set_ylabel("Spectral radius")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(ncol=2)
    figure.suptitle(
        "MODEL-020-SYNTHETIC one-at-a-time plant-parameter sweep"
    )
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def write_nonideality_plot(
    path: Path,
    records: list[dict[str, Any]],
) -> None:
    """Plot tracking and estimation descriptors for non-parameter cases."""

    path.parent.mkdir(parents=True, exist_ok=True)
    selected = [
        record
        for record in records
        if record["category"] not in {"plant_parameter"}
    ]
    labels = [record["scenario_id"] for record in selected]
    positions = np.arange(len(selected))
    figure, axes = plt.subplots(2, 1, figsize=(11.5, 8.0), sharex=True)
    display_floor = 1e-12
    axes[0].bar(
        positions,
        [
            max(
                record["tail_mean_absolute_tracking_error_rad"],
                display_floor,
            )
            for record in selected
        ],
        color="#4472C4",
    )
    axes[0].set_yscale("log")
    axes[0].set_ylabel("Tail mean absolute error (rad, log scale)")
    axes[0].set_title("Tracking residue after the transient")
    axes[1].bar(
        positions,
        [
            max(
                record["rms_position_estimation_error_rad"],
                display_floor,
            )
            for record in selected
        ],
        color="#ED7D31",
    )
    axes[1].set_yscale("log")
    axes[1].set_ylabel("RMS estimation error (rad, log scale)")
    axes[1].set_title("Observer mismatch")
    axes[1].set_xticks(positions, labels, rotation=30, ha="right")
    for axis in axes:
        axis.grid(True, axis="y", alpha=0.3)
    figure.suptitle(
        "MODEL-020-SYNTHETIC sensor, timing, actuator, and combined cases"
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    figure.savefig(path, dpi=160)
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    """Parse command-line paths."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--motor-parameters",
        type=Path,
        default=Path("models/parameters/synthetic_motor.json"),
    )
    parser.add_argument(
        "--controller-parameters",
        type=Path,
        default=Path("models/parameters/synthetic_controller.json"),
    )
    parser.add_argument(
        "--robustness-parameters",
        type=Path,
        default=Path("models/parameters/synthetic_robustness.json"),
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("data/processed/model_020_synthetic_robustness_report.json"),
    )
    parser.add_argument(
        "--results",
        type=Path,
        default=Path("data/processed/model_020_synthetic_robustness_results.csv"),
    )
    parser.add_argument(
        "--parameter-plot",
        type=Path,
        default=Path("docs/media/model_020_parameter_sweep.png"),
    )
    parser.add_argument(
        "--nonideality-plot",
        type=Path,
        default=Path("docs/media/model_020_nonideality_summary.png"),
    )
    return parser.parse_args()


def main() -> int:
    """Run the analysis, write evidence, and return failure to the shell."""

    args = parse_args()
    report, records = build_report(
        args.motor_parameters,
        args.controller_parameters,
        args.robustness_parameters,
    )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_results_csv(args.results, records)
    write_parameter_sweep_plot(args.parameter_plot, records)
    write_nonideality_plot(args.nonideality_plot, records)
    print(
        f"{report['evidence_id']}: {report['result']} "
        f"({sum(report['checks'].values())}/{len(report['checks'])} checks, "
        f"{report['scenario_count']} scenarios)"
    )
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
