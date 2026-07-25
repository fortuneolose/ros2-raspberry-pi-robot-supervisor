"""Generate reproducible development evidence for the synthetic plant model."""

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

from models.dc_motor import (
    MotorParameters,
    continuous_dc_motor_model,
    discretize_zero_order_hold,
    evaluate_structure,
    simulate_discrete,
)


def load_parameter_file(path: Path) -> tuple[MotorParameters, dict[str, Any]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return MotorParameters.from_mapping(payload["parameters"]), payload


def build_report(
    parameter_file: Path,
    duration_s: float,
    step_voltage_v: float,
) -> tuple[dict[str, Any], np.ndarray, np.ndarray, np.ndarray]:
    parameters, metadata = load_parameter_file(parameter_file)
    sample_period_s = float(metadata["sample_period_s"])
    continuous = continuous_dc_motor_model(parameters)
    discrete = discretize_zero_order_hold(continuous, sample_period_s)

    continuous_structure = evaluate_structure(continuous)
    discrete_structure = evaluate_structure(discrete)
    state_count = continuous.a.shape[0]
    passed = all(
        (
            continuous_structure["voltage_input_controllability_rank"] == state_count,
            continuous_structure["encoder_position_observability_rank"] == state_count,
            discrete_structure["voltage_input_controllability_rank"] == state_count,
            discrete_structure["encoder_position_observability_rank"] == state_count,
        )
    )

    steps = int(round(duration_s / sample_period_s))
    inputs = np.zeros((steps, 2), dtype=np.float64)
    inputs[:, 0] = step_voltage_v
    states, _outputs = simulate_discrete(discrete, inputs)
    times = np.arange(steps + 1, dtype=np.float64) * sample_period_s
    export_inputs = np.vstack((inputs, inputs[-1]))

    report = {
        "evidence_id": "MODEL-001-SYNTHETIC",
        "result": "PASS" if passed else "FAIL",
        "scope": "software-development structural check",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "software_environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "matplotlib": matplotlib.__version__,
        },
        "acceptance_limitation": (
            "This is not final MODEL-001 acceptance evidence. The parameter set "
            "is synthetic and must be replaced after motor selection and plant "
            "identification."
        ),
        "parameter_set_id": metadata["parameter_set_id"],
        "parameter_status": metadata["status"],
        "parameter_provenance": metadata["provenance"],
        "sample_period_s": sample_period_s,
        "model_assumptions": [
            "linear time-invariant electrical and mechanical dynamics",
            "rigid shaft with parameters referred to the modelled shaft",
            "ideal applied armature voltage",
            "linear viscous friction",
            "ideal encoder-position measurement",
            "no saturation, dead zone, stiction, backlash, quantisation, noise, or thermal variation",
        ],
        "states": [
            {"name": name, "unit": unit}
            for name, unit in zip(discrete.state_names, discrete.state_units)
        ],
        "inputs": [
            {"name": name, "unit": unit}
            for name, unit in zip(discrete.input_names, discrete.input_units)
        ],
        "outputs": [
            {"name": name, "unit": unit}
            for name, unit in zip(discrete.output_names, discrete.output_units)
        ],
        "continuous_structure": continuous_structure,
        "discrete_structure": discrete_structure,
        "continuous_matrices": {
            "A": continuous.a.tolist(),
            "B": continuous.b.tolist(),
            "C": continuous.c.tolist(),
            "D": continuous.d.tolist(),
        },
        "discrete_matrices": {
            "A": discrete.a.tolist(),
            "B": discrete.b.tolist(),
            "C": discrete.c.tolist(),
            "D": discrete.d.tolist(),
        },
        "development_simulation": {
            "duration_s": duration_s,
            "step_voltage_v": step_voltage_v,
            "opposing_load_torque_nm": 0.0,
            "final_state": {
                name: float(value)
                for name, value in zip(discrete.state_names, states[-1])
            },
        },
        "not_demonstrated": [
            "identified physical-motor parameters",
            "controller or observer design",
            "closed-loop performance",
            "fixed-point arithmetic",
            "Raspberry Pi timing",
            "FPGA RTL",
            "hardware-in-the-loop operation",
        ],
    }
    return report, times, export_inputs, states


def write_csv(
    path: Path,
    times: np.ndarray,
    inputs: np.ndarray,
    states: np.ndarray,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(
            [
                "time_s",
                "armature_voltage_v",
                "opposing_load_torque_nm",
                "shaft_position_rad",
                "shaft_speed_rad_s",
                "armature_current_a",
            ]
        )
        for time_s, input_vector, state in zip(times, inputs, states):
            writer.writerow([time_s, *input_vector, *state])


def write_plot(path: Path, times: np.ndarray, states: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(3, 1, figsize=(8.0, 7.5), sharex=True)
    series = (
        ("Shaft position", states[:, 0], "rad"),
        ("Shaft speed", states[:, 1], "rad/s"),
        ("Armature current", states[:, 2], "A"),
    )
    for axis, (title, values, unit) in zip(axes, series):
        axis.plot(times, values, color="#2e74b5", linewidth=1.8)
        axis.set_ylabel(unit)
        axis.set_title(title, loc="left", fontsize=10, fontweight="bold")
        axis.grid(True, alpha=0.25)
    axes[-1].set_xlabel("Time (s)")
    figure.suptitle(
        "MODEL-001-SYNTHETIC: 1 V open-loop development response",
        fontsize=12,
        fontweight="bold",
    )
    figure.text(
        0.5,
        0.01,
        "Synthetic parameters — not physical plant-identification evidence",
        ha="center",
        fontsize=9,
        color="#7a4b00",
    )
    figure.tight_layout(rect=(0, 0.04, 1, 0.96))
    figure.savefig(path, dpi=160)
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--parameters",
        type=Path,
        default=Path("models/parameters/synthetic_motor.json"),
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("data/processed/model_001_synthetic_report.json"),
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path("data/processed/model_001_synthetic_step.csv"),
    )
    parser.add_argument(
        "--plot",
        type=Path,
        default=Path("docs/media/model_001_synthetic_step.png"),
    )
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--step-voltage-v", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.duration_s <= 0.0:
        raise ValueError("--duration-s must be greater than zero")
    if not np.isfinite(args.step_voltage_v):
        raise ValueError("--step-voltage-v must be finite")

    report, times, inputs, states = build_report(
        args.parameters, args.duration_s, args.step_voltage_v
    )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_csv(args.csv, times, inputs, states)
    write_plot(args.plot, times, states)

    print(
        f"{report['evidence_id']}: {report['result']} "
        f"(synthetic development baseline only)"
    )
    print(f"report: {args.report}")
    print(f"data:   {args.csv}")
    print(f"plot:   {args.plot}")
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
