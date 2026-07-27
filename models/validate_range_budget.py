"""Generate the synthetic pre-fixed-point provenance and range-budget audit."""

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
from models.range_budget import (
    ObservedRange,
    coefficient_dynamic_range,
    coefficient_records,
    finalize_range_records,
    observe_closed_loop_result,
    observe_observer_result,
)
from models.robustness import (
    build_scenarios_from_mapping,
    run_robustness_scenario,
)
from models.validate_controller import build_report as build_controller_report


def load_json(path: Path) -> dict[str, Any]:
    """Load one JSON object."""

    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def build_audit(
    motor_file: Path,
    controller_file: Path,
    robustness_file: Path,
    range_config_file: Path,
) -> tuple[dict[str, Any], list[dict[str, Any]], list[dict[str, Any]]]:
    """Run provenance, range, and readiness checks."""

    motor_payload = load_json(motor_file)
    controller_payload = load_json(controller_file)
    robustness_payload = load_json(robustness_file)
    range_config = load_json(range_config_file)

    nominal_parameters = MotorParameters.from_mapping(
        motor_payload["parameters"]
    )
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
    repeated_design = design_observer_controller(
        nominal_model,
        controller_poles,
        observer_poles,
    )

    (
        _,
        controller_report_design,
        nominal,
        disturbed,
        observer_initialization,
    ) = build_controller_report(motor_file, controller_file)

    observations: dict[str, ObservedRange] = {}
    observe_closed_loop_result(
        observations,
        "model010_nominal",
        nominal,
        nominal_model,
        design,
    )
    observe_closed_loop_result(
        observations,
        "model010_load_pulse",
        disturbed,
        nominal_model,
        design,
    )
    observe_observer_result(
        observations,
        "model010_observer_initialization",
        observer_initialization,
        nominal_model,
        design,
    )

    probe = range_config["saturation_probe"]
    probe_count = int(
        round(float(probe["duration_s"]) / sample_period_s)
    )
    if probe_count < 1:
        raise ValueError("saturation probe must contain at least one sample")
    saturation_probe = simulate_observer_feedback(
        nominal_model,
        design,
        np.full(
            probe_count,
            float(probe["reference_position_rad"]),
            dtype=np.float64,
        ),
        np.zeros(probe_count, dtype=np.float64),
        float(probe["voltage_limit_v"]),
    )
    observe_closed_loop_result(
        observations,
        "model010_saturation_probe",
        saturation_probe,
        nominal_model,
        design,
    )

    robustness_simulation = robustness_payload["simulation"]
    robustness_sample_count = int(
        round(
            float(robustness_simulation["duration_s"])
            / sample_period_s
        )
    )
    references = np.full(
        robustness_sample_count,
        float(robustness_simulation["reference_position_rad"]),
        dtype=np.float64,
    )
    loads = np.zeros(robustness_sample_count, dtype=np.float64)
    robustness_scenarios = build_scenarios_from_mapping(robustness_payload)
    for scenario in robustness_scenarios:
        result, _, _ = run_robustness_scenario(
            nominal_parameters,
            nominal_model,
            design,
            scenario,
            references,
            loads,
            int(robustness_payload["random_seed"]),
        )
        observe_closed_loop_result(
            observations,
            f"model020_{scenario.scenario_id}",
            result,
            nominal_model,
            design,
        )

    range_records = finalize_range_records(
        observations,
        float(range_config["observed_range_guard_factor"]),
        {
            str(name): float(value)
            for name, value in range_config[
                "configured_hard_bounds"
            ].items()
        },
    )
    coefficients = coefficient_records(nominal_model, design)
    coefficient_span = coefficient_dynamic_range(coefficients)
    required_signals = set(range_config["required_signal_ids"])
    actual_signals = {
        record["signal_id"] for record in range_records
    }
    readiness = {
        str(name): bool(value)
        for name, value in range_config["readiness_requirements"].items()
    }

    uniform_coefficient_width = 18
    maximum_coefficient_integer_bits = max(
        1
        if abs(float(record["value"])) < 1.0
        else int(np.floor(np.log2(abs(float(record["value"]))))) + 2
        for record in coefficients
        if float(record["value"]) != 0.0
    )
    uniform_fractional_bits = (
        uniform_coefficient_width - maximum_coefficient_integer_bits
    )
    uniform_lsb = 2.0 ** (-uniform_fractional_bits)
    coefficients_below_half_lsb = [
        record["coefficient_id"]
        for record in coefficients
        if 0.0 < abs(float(record["value"])) < 0.5 * uniform_lsb
    ]

    source_audit = [
        {
            "item": "continuous_motor_parameters",
            "source_file": str(motor_file),
            "classification": motor_payload["status"],
            "derivation_reproducible": True,
            "physical_source_present": False,
            "decision": (
                "Replace with datasheet-backed and experimentally identified "
                "values plus uncertainty before coefficient freeze."
            ),
        },
        {
            "item": "sample_period",
            "source_file": str(motor_file),
            "classification": "preliminary_design_target",
            "derivation_reproducible": True,
            "physical_source_present": False,
            "decision": (
                "Justify from identified bandwidth and measured platform timing."
            ),
        },
        {
            "item": "controller_pole_targets",
            "source_file": str(controller_file),
            "classification": controller_payload["status"],
            "derivation_reproducible": True,
            "physical_source_present": False,
            "decision": (
                "Re-select against approved performance and actuator limits."
            ),
        },
        {
            "item": "observer_pole_targets",
            "source_file": str(controller_file),
            "classification": controller_payload["status"],
            "derivation_reproducible": True,
            "physical_source_present": False,
            "decision": (
                "Re-select after encoder noise and estimator range measurements."
            ),
        },
        {
            "item": "A_Bv_C_discrete_model_coefficients",
            "source_file": "models/dc_motor.py",
            "classification": "derived_from_synthetic_inputs",
            "derivation_reproducible": True,
            "physical_source_present": False,
            "decision": (
                "Derivation is traceable; numeric values are not hardware-approved."
            ),
        },
        {
            "item": "K_state_feedback_gain",
            "source_file": "models/control.py",
            "classification": "derived_from_synthetic_inputs",
            "derivation_reproducible": True,
            "physical_source_present": False,
            "decision": (
                "Keep as regression fixture; redesign after plant identification."
            ),
        },
        {
            "item": "N_reference_gain",
            "source_file": "models/control.py",
            "classification": "derived_from_synthetic_inputs",
            "derivation_reproducible": True,
            "physical_source_present": False,
            "decision": (
                "Recompute for the accepted plant and reference architecture."
            ),
        },
        {
            "item": "L_observer_gain",
            "source_file": "models/control.py",
            "classification": "derived_from_synthetic_inputs",
            "derivation_reproducible": True,
            "physical_source_present": False,
            "decision": (
                "Keep as regression fixture; redesign after noise/range review."
            ),
        },
        {
            "item": "uncertainty_and_nonideality_ranges",
            "source_file": str(robustness_file),
            "classification": robustness_payload["status"],
            "derivation_reproducible": True,
            "physical_source_present": False,
            "decision": (
                "Replace synthetic factors with measured or source-backed bounds."
            ),
        },
    ]

    coefficients_repeatable = bool(
        np.array_equal(
            design.state_feedback_gain,
            repeated_design.state_feedback_gain,
        )
        and design.reference_gain == repeated_design.reference_gain
        and np.array_equal(
            design.observer_gain,
            repeated_design.observer_gain,
        )
        and np.array_equal(
            design.state_feedback_gain,
            controller_report_design.state_feedback_gain,
        )
        and design.reference_gain == controller_report_design.reference_gain
        and np.array_equal(
            design.observer_gain,
            controller_report_design.observer_gain,
        )
    )
    checks = {
        "coefficient_derivation_exactly_repeatable": coefficients_repeatable,
        "all_source_classifications_explicit": all(
            record["classification"] for record in source_audit
        ),
        "synthetic_sources_correctly_flagged_nonphysical": not any(
            record["physical_source_present"] for record in source_audit
        ),
        "all_required_signal_ranges_present": (
            required_signals <= actual_signals
        ),
        "all_observed_ranges_finite": all(
            np.isfinite(record["observed_min"])
            and np.isfinite(record["observed_max"])
            and np.isfinite(record["observed_peak_abs"])
            for record in range_records
        ),
        "all_budget_bounds_enclose_observations": all(
            record["budget_abs_bound"] + 1e-12
            >= record["observed_peak_abs"]
            for record in range_records
        ),
        "all_integer_bit_counts_enclose_budget_bounds": all(
            record["budget_abs_bound"]
            < 2.0
            ** (
                record[
                    "minimum_signed_integer_bits_including_sign"
                ]
                - 1
            )
            for record in range_records
        ),
        "fractional_bits_deliberately_unassigned": all(
            record["fractional_bits_status"]
            == "TBD_PENDING_QUANTIZATION_ERROR_STUDY"
            for record in range_records
        ),
        "fixed_point_readiness_hold_enforced": (
            not all(readiness.values())
        ),
    }
    audit_result = "PASS" if all(checks.values()) else "FAIL"
    report = {
        "audit_id": range_config["audit_id"],
        "result": audit_result,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": (
            "synthetic coefficient-provenance review and floating-point "
            "numeric-range budget before fixed-point conversion"
        ),
        "status": range_config["status"],
        "provenance": range_config["provenance"],
        "software_environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "matplotlib": matplotlib.__version__,
        },
        "source_audit": source_audit,
        "coefficient_records": coefficients,
        "coefficient_dynamic_range": coefficient_span,
        "uniform_18_bit_coefficient_hypothesis": {
            "total_bits": uniform_coefficient_width,
            "integer_bits_including_sign_needed_for_largest_coefficient": (
                maximum_coefficient_integer_bits
            ),
            "fractional_bits_remaining_if_uniformly_scaled": (
                uniform_fractional_bits
            ),
            "resulting_lsb": uniform_lsb,
            "coefficients_below_half_lsb": coefficients_below_half_lsb,
            "verdict": (
                "NOT_SUPPORTED_WITH_ONE_GLOBAL_BINARY_POINT"
            ),
            "required_follow_up": (
                "Evaluate state normalization and block-, row-, or "
                "coefficient-specific scaling during quantization studies."
            ),
        },
        "case_count": 4 + len(robustness_scenarios),
        "case_coverage": [
            "MODEL-010 nominal step",
            "MODEL-010 load pulse",
            "MODEL-010 observer initialisation",
            "MODEL-010 synthetic saturation probe",
            "all 20 MODEL-020 robustness scenarios",
        ],
        "observed_range_guard_factor": float(
            range_config["observed_range_guard_factor"]
        ),
        "range_records": range_records,
        "checks": checks,
        "readiness_requirements": readiness,
        "coefficient_freeze_readiness": "HOLD",
        "fixed_point_conversion_readiness": "HOLD",
        "pr4_recommendation": "KEEP_DRAFT_PENDING_HUMAN_REVIEW",
        "blockers": [
            name for name, complete in readiness.items() if not complete
        ],
        "interpretation": [
            (
                "The derivation chain is reproducible, but reproducibility does "
                "not establish physical coefficient provenance."
            ),
            (
                "The guarded ranges contain only configured synthetic cases; "
                "they are not worst-case physical guarantees."
            ),
            (
                "Integer range estimates use unity engineering-unit scaling. "
                "Fractional precision and binary points remain deliberately TBD."
            ),
            (
                "The current single 18-bit coefficient-width hypothesis cannot "
                "use one shared binary point across the observed coefficient span."
            ),
        ],
        "acceptance_limitation": (
            "Audit execution may pass while readiness remains HOLD. No "
            "fixed-point format or hardware coefficient set is approved."
        ),
    }
    return report, range_records, coefficients


def write_range_csv(path: Path, records: list[dict[str, Any]]) -> None:
    """Write the full range budget."""

    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "signal_id",
        "group",
        "unit",
        "observed_min",
        "observed_max",
        "observed_peak_abs",
        "peak_source_case",
        "budget_abs_bound",
        "budget_basis",
        "guard_factor",
        "minimum_signed_integer_bits_including_sign",
        "fractional_bits_status",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(records)


def write_coefficient_csv(path: Path, records: list[dict[str, Any]]) -> None:
    """Write coefficient derivation records."""

    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "coefficient_id",
        "block",
        "value",
        "unit",
        "derivation",
        "source_class",
    ]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=fieldnames,
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(records)


def write_range_plot(
    path: Path,
    records: list[dict[str, Any]],
    selected_signal_ids: list[str],
) -> None:
    """Compare observed peaks with provisional budget bounds."""

    lookup = {record["signal_id"]: record for record in records}
    selected = [lookup[signal_id] for signal_id in selected_signal_ids]
    positions = np.arange(len(selected))
    observed = np.asarray(
        [record["observed_peak_abs"] for record in selected],
        dtype=np.float64,
    )
    budget = np.asarray(
        [record["budget_abs_bound"] for record in selected],
        dtype=np.float64,
    )
    labels = [
        f"{record['signal_id']} [{record['unit']}]"
        for record in selected
    ]

    path.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(11.5, 8.5))
    height = 0.36
    axis.barh(
        positions - height / 2.0,
        observed,
        height=height,
        label="observed floating-point peak",
        color="#4472C4",
    )
    axis.barh(
        positions + height / 2.0,
        budget,
        height=height,
        label="provisional budget bound",
        color="#ED7D31",
    )
    axis.set_xscale("log")
    axis.set_yticks(positions, labels)
    axis.invert_yaxis()
    axis.set_xlabel("Absolute magnitude in the signal's engineering unit")
    axis.set_title(
        "MODEL-020-PREFLIGHT-SYNTHETIC observed ranges and guarded bounds"
    )
    axis.grid(True, axis="x", alpha=0.3)
    axis.legend()
    figure.tight_layout()
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
        "--range-config",
        type=Path,
        default=Path("models/parameters/synthetic_range_budget.json"),
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path(
            "data/processed/model_020_synthetic_preflight_report.json"
        ),
    )
    parser.add_argument(
        "--range-csv",
        type=Path,
        default=Path(
            "data/processed/model_020_synthetic_numeric_range_budget.csv"
        ),
    )
    parser.add_argument(
        "--coefficient-csv",
        type=Path,
        default=Path(
            "data/processed/model_020_synthetic_coefficient_provenance.csv"
        ),
    )
    parser.add_argument(
        "--plot",
        type=Path,
        default=Path("docs/media/model_020_synthetic_range_budget.png"),
    )
    return parser.parse_args()


def main() -> int:
    """Run the audit and write machine-readable evidence."""

    args = parse_args()
    range_config = load_json(args.range_config)
    report, ranges, coefficients = build_audit(
        args.motor_parameters,
        args.controller_parameters,
        args.robustness_parameters,
        args.range_config,
    )
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_range_csv(args.range_csv, ranges)
    write_coefficient_csv(args.coefficient_csv, coefficients)
    write_range_plot(
        args.plot,
        ranges,
        list(range_config["plot_signal_ids"]),
    )
    print(
        f"{report['audit_id']}: {report['result']} "
        f"({sum(report['checks'].values())}/{len(report['checks'])} checks); "
        f"fixed-point readiness: {report['fixed_point_conversion_readiness']}"
    )
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
