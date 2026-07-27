"""Generate reproducible controller/observer evidence for the synthetic model."""

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
    ClosedLoopResult,
    ObserverControllerDesign,
    ObserverResult,
    continuous_poles_to_discrete,
    design_observer_controller,
    simulate_observer_feedback,
    simulate_state_observer,
    step_response_metrics,
    suffix_entry_time,
)
from models.dc_motor import (
    MotorParameters,
    continuous_dc_motor_model,
    discretize_zero_order_hold,
)


def load_json(path: Path) -> dict[str, Any]:
    """Load one JSON object."""

    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def complex_pole_records(values: np.ndarray) -> list[dict[str, float]]:
    """Convert complex poles into stable JSON records."""

    return [
        {
            "real": float(np.real(value)),
            "imaginary": float(np.imag(value)),
            "magnitude": float(np.abs(value)),
        }
        for value in np.sort_complex(values)
    ]


def maximum_pole_error(
    requested: np.ndarray,
    achieved: np.ndarray,
) -> float:
    """Return the maximum absolute pole-placement error."""

    return float(
        np.max(
            np.abs(
                np.sort_complex(requested)
                - np.sort_complex(achieved)
            )
        )
    )


def build_report(
    parameter_file: Path,
    controller_file: Path,
) -> tuple[
    dict[str, Any],
    ObserverControllerDesign,
    ClosedLoopResult,
    ClosedLoopResult,
    ObserverResult,
]:
    """Design, simulate, evaluate, and return the synthetic baseline."""

    motor_payload = load_json(parameter_file)
    controller_payload = load_json(controller_file)
    parameters = MotorParameters.from_mapping(motor_payload["parameters"])
    sample_period_s = float(motor_payload["sample_period_s"])
    model = discretize_zero_order_hold(
        continuous_dc_motor_model(parameters),
        sample_period_s,
    )

    requested_controller_poles = continuous_poles_to_discrete(
        controller_payload["controller"]["continuous_poles_rad_s"],
        sample_period_s,
    )
    requested_observer_poles = continuous_poles_to_discrete(
        controller_payload["observer"]["continuous_poles_rad_s"],
        sample_period_s,
    )
    design = design_observer_controller(
        model,
        requested_controller_poles,
        requested_observer_poles,
    )

    simulation = controller_payload["simulation"]
    limits = controller_payload["development_limits"]
    duration_s = float(simulation["duration_s"])
    sample_count = int(round(duration_s / sample_period_s))
    reference_rad = float(simulation["reference_position_rad"])
    voltage_limit_v = float(simulation["voltage_limit_v"])
    references = np.full(sample_count, reference_rad, dtype=np.float64)
    zero_load = np.zeros(sample_count, dtype=np.float64)
    nominal = simulate_observer_feedback(
        model,
        design,
        references,
        zero_load,
        voltage_limit_v,
    )
    nominal_metrics = step_response_metrics(
        nominal.times_s[:-1],
        nominal.true_states[:-1, 0],
        reference_rad,
    )

    load_torques = np.zeros(sample_count, dtype=np.float64)
    disturbance_start_index = int(
        round(float(simulation["load_pulse_start_s"]) / sample_period_s)
    )
    disturbance_end_index = int(
        round(float(simulation["load_pulse_end_s"]) / sample_period_s)
    )
    if not 0 <= disturbance_start_index < disturbance_end_index < sample_count:
        raise ValueError("load-pulse interval must lie inside the simulation")
    load_torques[disturbance_start_index:disturbance_end_index] = float(
        simulation["load_pulse_nm"]
    )
    disturbed = simulate_observer_feedback(
        model,
        design,
        references,
        load_torques,
        voltage_limit_v,
    )
    disturbed_tracking_error = (
        reference_rad - disturbed.true_states[:-1, 0]
    )
    disturbance_recovery_time_s = suffix_entry_time(
        disturbed.times_s[:-1],
        disturbed_tracking_error,
        0.02 * abs(reference_rad),
        start_index=disturbance_end_index,
    )

    observer_sample_count = int(
        round(
            float(simulation["observer_duration_s"])
            / sample_period_s
        )
    )
    observer_inputs = np.zeros(
        (observer_sample_count, model.b.shape[1]),
        dtype=np.float64,
    )
    observer = simulate_state_observer(
        model,
        design.observer_gain,
        observer_inputs,
        simulation["observer_initial_state"],
        simulation["observer_initial_estimate"],
    )
    observer_state_error = (
        observer.true_states - observer.estimated_states
    )
    observer_component_limits = np.array(
        [
            limits["observer_component_limits"]["position_error_rad"],
            limits["observer_component_limits"]["speed_error_rad_s"],
            limits["observer_component_limits"]["current_error_a"],
        ],
        dtype=np.float64,
    )
    normalized_observer_error = np.max(
        np.abs(observer_state_error) / observer_component_limits,
        axis=1,
    )
    observer_convergence_time_s = suffix_entry_time(
        observer.times_s,
        normalized_observer_error,
        1.0,
    )

    controller_pole_error = maximum_pole_error(
        design.requested_controller_poles,
        design.achieved_controller_poles,
    )
    observer_pole_error = maximum_pole_error(
        design.requested_observer_poles,
        design.achieved_observer_poles,
    )
    maximum_applied_voltage_v = float(
        np.max(np.abs(nominal.applied_voltages_v))
    )
    checks = {
        "controller_poles_stable": bool(
            np.max(np.abs(design.achieved_controller_poles)) < 1.0
        ),
        "observer_poles_stable": bool(
            np.max(np.abs(design.achieved_observer_poles)) < 1.0
        ),
        "controller_pole_placement_error_below_1e_9": (
            controller_pole_error <= 1e-9
        ),
        "observer_pole_placement_error_below_1e_9": (
            observer_pole_error <= 1e-9
        ),
        "rise_time_within_development_limit": (
            nominal_metrics["rise_time_10_90_s"]
            <= float(limits["rise_time_10_90_s_max"])
        ),
        "settling_time_within_development_limit": (
            nominal_metrics["settling_time_2_percent_s"]
            <= float(limits["settling_time_2_percent_s_max"])
        ),
        "overshoot_within_development_limit": (
            nominal_metrics["overshoot_percent"]
            <= float(limits["overshoot_percent_max"])
        ),
        "steady_state_error_within_development_limit": (
            nominal_metrics["steady_state_error_rad"]
            <= float(limits["steady_state_error_rad_max"])
        ),
        "nominal_voltage_within_limit": (
            maximum_applied_voltage_v <= voltage_limit_v
        ),
        "nominal_response_not_saturated": (
            int(np.count_nonzero(nominal.saturated)) == 0
        ),
        "observer_convergence_within_development_limit": (
            observer_convergence_time_s
            <= float(limits["observer_convergence_time_s_max"])
        ),
        "observer_final_error_within_development_limit": (
            float(normalized_observer_error[-1]) <= 1.0
        ),
        "disturbance_recovery_within_development_limit": (
            disturbance_recovery_time_s
            <= float(limits["disturbance_recovery_time_s_max"])
        ),
    }

    report = {
        "evidence_id": controller_payload["design_id"],
        "result": "PASS" if all(checks.values()) else "FAIL",
        "scope": "synthetic floating-point controller and observer development check",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "software_environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "matplotlib": matplotlib.__version__,
        },
        "acceptance_limitation": (
            "This is not final MODEL-010 acceptance evidence. The plant, pole "
            "targets, voltage limit, reference, load pulse, and thresholds are "
            "synthetic development fixtures and must be revisited after motor "
            "selection and plant identification."
        ),
        "parameter_set_id": motor_payload["parameter_set_id"],
        "parameter_status": motor_payload["status"],
        "design_status": controller_payload["status"],
        "design_provenance": controller_payload["provenance"],
        "sample_period_s": sample_period_s,
        "controller": {
            "method": controller_payload["controller"]["method"],
            "continuous_pole_targets_rad_s": controller_payload["controller"][
                "continuous_poles_rad_s"
            ],
            "requested_discrete_poles": complex_pole_records(
                design.requested_controller_poles
            ),
            "achieved_discrete_poles": complex_pole_records(
                design.achieved_controller_poles
            ),
            "maximum_pole_error": controller_pole_error,
            "state_feedback_gain": design.state_feedback_gain.tolist(),
            "reference_gain": design.reference_gain,
        },
        "observer": {
            "method": controller_payload["observer"]["method"],
            "continuous_pole_targets_rad_s": controller_payload["observer"][
                "continuous_poles_rad_s"
            ],
            "requested_discrete_poles": complex_pole_records(
                design.requested_observer_poles
            ),
            "achieved_discrete_poles": complex_pole_records(
                design.achieved_observer_poles
            ),
            "maximum_pole_error": observer_pole_error,
            "gain": design.observer_gain.tolist(),
            "component_limits": limits["observer_component_limits"],
            "initial_normalized_max_error": float(
                normalized_observer_error[0]
            ),
            "peak_normalized_max_error": float(
                np.max(normalized_observer_error)
            ),
            "final_normalized_max_error": float(
                normalized_observer_error[-1]
            ),
            "final_component_errors": {
                "position_error_rad": float(observer_state_error[-1, 0]),
                "speed_error_rad_s": float(observer_state_error[-1, 1]),
                "current_error_a": float(observer_state_error[-1, 2]),
            },
            "convergence_time_s": observer_convergence_time_s,
            "convergence_definition": (
                "All absolute component errors remain at or below their "
                "dimensioned limits."
            ),
        },
        "nominal_step": {
            "duration_s": duration_s,
            "reference_position_rad": reference_rad,
            "voltage_limit_v": voltage_limit_v,
            "maximum_applied_voltage_v": maximum_applied_voltage_v,
            "saturated_sample_count": int(
                np.count_nonzero(nominal.saturated)
            ),
            **nominal_metrics,
        },
        "load_pulse": {
            "start_s": float(simulation["load_pulse_start_s"]),
            "end_s": float(simulation["load_pulse_end_s"]),
            "magnitude_nm": float(simulation["load_pulse_nm"]),
            "peak_absolute_tracking_error_rad": float(
                np.max(
                    np.abs(
                        disturbed_tracking_error[
                            disturbance_start_index:
                        ]
                    )
                )
            ),
            "recovery_time_s": disturbance_recovery_time_s,
            "final_tracking_error_rad": float(
                disturbed_tracking_error[-1]
            ),
        },
        "development_limits": limits,
        "checks": checks,
        "not_demonstrated": [
            "identified physical-motor parameters",
            "final gain or performance acceptance",
            "integral disturbance rejection",
            "measurement-noise robustness",
            "plant-parameter uncertainty",
            "actuator dead zone or PWM nonlinearity",
            "fixed-point arithmetic",
            "Raspberry Pi scheduling",
            "FPGA RTL",
            "hardware-in-the-loop operation",
        ],
    }
    return report, design, nominal, disturbed, observer


def write_response_csv(
    path: Path,
    result: ClosedLoopResult,
    component_limits: np.ndarray,
) -> None:
    """Write the disturbance scenario using explicit engineering units."""

    path.parent.mkdir(parents=True, exist_ok=True)
    state_error = result.true_states - result.estimated_states
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(
            [
                "time_s",
                "reference_position_rad",
                "load_torque_nm",
                "true_position_rad",
                "true_speed_rad_s",
                "true_current_a",
                "estimated_position_rad",
                "estimated_speed_rad_s",
                "estimated_current_a",
                "position_estimation_error_rad",
                "speed_estimation_error_rad_s",
                "current_estimation_error_a",
                "normalized_estimation_error_max",
                "requested_voltage_v",
                "applied_voltage_v",
                "saturated",
            ]
        )
        for index in range(result.references_rad.size):
            writer.writerow(
                [
                    result.times_s[index],
                    result.references_rad[index],
                    result.load_torques_nm[index],
                    *result.true_states[index],
                    *result.estimated_states[index],
                    *state_error[index],
                    np.max(
                        np.abs(state_error[index])
                        / component_limits
                    ),
                    result.requested_voltages_v[index],
                    result.applied_voltages_v[index],
                    int(result.saturated[index]),
                ]
            )


def write_closed_loop_plot(
    path: Path,
    result: ClosedLoopResult,
    load_start_s: float,
    load_end_s: float,
) -> None:
    """Plot the synthetic closed-loop response and load pulse."""

    path.parent.mkdir(parents=True, exist_ok=True)
    times = result.times_s[:-1]
    figure, axes = plt.subplots(4, 1, figsize=(9.0, 9.5), sharex=True)
    series = (
        (
            "Position",
            result.true_states[:-1, 0],
            result.estimated_states[:-1, 0],
            "rad",
        ),
        (
            "Speed",
            result.true_states[:-1, 1],
            result.estimated_states[:-1, 1],
            "rad/s",
        ),
        (
            "Armature current",
            result.true_states[:-1, 2],
            result.estimated_states[:-1, 2],
            "A",
        ),
    )
    for axis, (title, actual, estimated, unit) in zip(axes[:3], series):
        axis.plot(times, actual, label="plant", linewidth=1.8)
        axis.plot(
            times,
            estimated,
            label="observer",
            linestyle="--",
            linewidth=1.2,
        )
        axis.axvspan(load_start_s, load_end_s, color="#f6b26b", alpha=0.25)
        axis.set_ylabel(unit)
        axis.set_title(title, loc="left", fontsize=10, fontweight="bold")
        axis.grid(True, alpha=0.25)
    axes[0].plot(
        times,
        result.references_rad,
        label="reference",
        color="#333333",
        linestyle=":",
        linewidth=1.3,
    )
    axes[0].legend(loc="best", ncol=3)

    axes[3].plot(
        times,
        result.applied_voltages_v,
        label="applied voltage",
        color="#2e74b5",
        linewidth=1.6,
    )
    axes[3].axvspan(
        load_start_s,
        load_end_s,
        color="#f6b26b",
        alpha=0.25,
        label="load pulse",
    )
    axes[3].set_ylabel("V")
    axes[3].set_xlabel("Time (s)")
    axes[3].set_title(
        "Control voltage and disturbance interval",
        loc="left",
        fontsize=10,
        fontweight="bold",
    )
    axes[3].grid(True, alpha=0.25)
    axes[3].legend(loc="best")

    figure.suptitle(
        "MODEL-010-SYNTHETIC: observer-based position control",
        fontsize=13,
        fontweight="bold",
    )
    figure.text(
        0.5,
        0.01,
        "Synthetic plant and provisional gains — not physical acceptance evidence",
        ha="center",
        fontsize=9,
        color="#7a4b00",
    )
    figure.tight_layout(rect=(0, 0.035, 1, 0.965))
    figure.savefig(path, dpi=160)
    plt.close(figure)


def write_observer_plot(
    path: Path,
    result: ObserverResult,
    component_limits: np.ndarray,
) -> None:
    """Plot observer-error convergence from the configured initial mismatch."""

    path.parent.mkdir(parents=True, exist_ok=True)
    state_error = result.true_states - result.estimated_states
    normalized_component_error = state_error / component_limits
    normalized_max_error = np.max(
        np.abs(normalized_component_error),
        axis=1,
    )
    figure, axes = plt.subplots(2, 1, figsize=(8.5, 6.5), sharex=True)
    axes[0].semilogy(
        result.times_s,
        np.maximum(normalized_max_error, np.finfo(float).tiny),
        color="#c0392b",
        linewidth=1.7,
        label="maximum normalized component error",
    )
    axes[0].axhline(
        1.0,
        color="#333333",
        linestyle=":",
        label="all component limits satisfied",
    )
    axes[0].set_ylabel("normalized error")
    axes[0].set_title(
        "Observer convergence",
        loc="left",
        fontsize=10,
        fontweight="bold",
    )
    axes[0].grid(True, which="both", alpha=0.25)
    axes[0].legend(loc="best")

    labels = ("position error", "speed error", "current error")
    for index, label in enumerate(labels):
        axes[1].plot(
            result.times_s,
            normalized_component_error[:, index],
            linewidth=1.3,
            label=label,
        )
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("error / component limit")
    axes[1].set_title(
        "Component errors",
        loc="left",
        fontsize=10,
        fontweight="bold",
    )
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="best")
    figure.suptitle(
        "MODEL-010-SYNTHETIC: Luenberger observer development check",
        fontsize=12,
        fontweight="bold",
    )
    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(path, dpi=160)
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
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
        "--report",
        type=Path,
        default=Path(
            "data/processed/model_010_synthetic_report.json"
        ),
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path(
            "data/processed/model_010_synthetic_response.csv"
        ),
    )
    parser.add_argument(
        "--closed-loop-plot",
        type=Path,
        default=Path(
            "docs/media/model_010_synthetic_closed_loop.png"
        ),
    )
    parser.add_argument(
        "--observer-plot",
        type=Path,
        default=Path(
            "docs/media/model_010_synthetic_observer.png"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report, _design, _nominal, disturbed, observer = build_report(
        args.motor_parameters,
        args.controller_parameters,
    )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    component_limits = np.array(
        [
            report["observer"]["component_limits"]["position_error_rad"],
            report["observer"]["component_limits"]["speed_error_rad_s"],
            report["observer"]["component_limits"]["current_error_a"],
        ],
        dtype=np.float64,
    )
    write_response_csv(args.csv, disturbed, component_limits)
    write_closed_loop_plot(
        args.closed_loop_plot,
        disturbed,
        report["load_pulse"]["start_s"],
        report["load_pulse"]["end_s"],
    )
    write_observer_plot(
        args.observer_plot,
        observer,
        component_limits,
    )

    print(
        f"{report['evidence_id']}: {report['result']} "
        f"(synthetic development baseline only)"
    )
    print(f"report:       {args.report}")
    print(f"data:         {args.csv}")
    print(f"closed loop:  {args.closed_loop_plot}")
    print(f"observer:     {args.observer_plot}")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
