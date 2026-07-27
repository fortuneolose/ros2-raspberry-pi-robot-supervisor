"""Pre-fixed-point provenance and numeric-range analysis utilities.

The functions in this module report floating-point observations and guarded
engineering-unit bounds. They do not assign word lengths, binary points,
rounding modes, or saturation arithmetic.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from models.control import (
    ClosedLoopResult,
    ObserverControllerDesign,
    ObserverResult,
)
from models.dc_motor import StateSpaceModel
from models.robustness import RobustnessResult


@dataclass
class ObservedRange:
    """Mutable extrema gathered across named deterministic cases."""

    signal_id: str
    group: str
    unit: str
    observed_min: float = np.inf
    observed_max: float = -np.inf
    observed_peak_abs: float = 0.0
    peak_source_case: str = ""

    def update(self, case_id: str, values: Any) -> None:
        """Include one finite, non-empty array in the accumulated range."""

        array = np.asarray(values, dtype=np.float64).reshape(-1)
        if array.size == 0 or not np.all(np.isfinite(array)):
            raise ValueError(
                f"{self.signal_id} values must be non-empty and finite"
            )
        self.observed_min = min(self.observed_min, float(np.min(array)))
        self.observed_max = max(self.observed_max, float(np.max(array)))
        peak = float(np.max(np.abs(array)))
        if peak > self.observed_peak_abs or not self.peak_source_case:
            self.observed_peak_abs = peak
            self.peak_source_case = case_id


def minimum_signed_integer_bits_including_sign(abs_bound: float) -> int:
    """Return integer bits needed for a signed fixed-point range.

    This counts the sign bit but no fractional bits. The positive endpoint of
    a two's-complement fixed-point format is strictly below ``2**(I - 1)``, so
    an exact power-of-two bound requires the next integer bit.
    """

    if not np.isfinite(abs_bound) or abs_bound < 0.0:
        raise ValueError("abs_bound must be finite and nonnegative")
    if abs_bound < 1.0:
        return 1
    return int(np.floor(np.log2(abs_bound))) + 2


def observe(
    observations: dict[str, ObservedRange],
    signal_id: str,
    group: str,
    unit: str,
    case_id: str,
    values: Any,
) -> None:
    """Create or update one consistently defined signal range."""

    if signal_id not in observations:
        observations[signal_id] = ObservedRange(signal_id, group, unit)
    record = observations[signal_id]
    if record.group != group or record.unit != unit:
        raise ValueError(f"inconsistent metadata for {signal_id}")
    record.update(case_id, values)


def _observe_controller_intermediates(
    observations: dict[str, ObservedRange],
    case_id: str,
    references: np.ndarray,
    estimated_states: np.ndarray,
    measured_positions: np.ndarray,
    commanded_voltages: np.ndarray,
    model: StateSpaceModel,
    design: ObserverControllerDesign,
) -> None:
    """Record controller and observer products before their final sums."""

    estimates = estimated_states[:-1]
    innovation = measured_positions - (
        model.c @ estimates.T
    ).reshape(-1)
    observe(
        observations,
        "innovation_rad",
        "sensor",
        "rad",
        case_id,
        innovation,
    )

    state_names = ("position", "speed", "current")
    state_units = ("rad", "rad/s", "A")
    feedback_products = (
        estimates * design.state_feedback_gain.reshape(1, -1)
    )
    for index, (state_name, unit) in enumerate(zip(state_names, state_units)):
        observe(
            observations,
            f"feedback_{state_name}_product_v",
            "controller_intermediate",
            "V",
            case_id,
            feedback_products[:, index],
        )
    feedback_accumulator = np.sum(feedback_products, axis=1)
    reference_product = design.reference_gain * references
    controller_l1_sum = (
        np.sum(np.abs(feedback_products), axis=1)
        + np.abs(reference_product)
    )
    observe(
        observations,
        "feedback_accumulator_v",
        "controller_intermediate",
        "V",
        case_id,
        feedback_accumulator,
    )
    observe(
        observations,
        "reference_product_v",
        "controller_intermediate",
        "V",
        case_id,
        reference_product,
    )
    observe(
        observations,
        "controller_l1_sum_of_products_v",
        "accumulator_bound",
        "V",
        case_id,
        controller_l1_sum,
    )

    prediction = (model.a @ estimates.T).T
    voltage_terms = commanded_voltages[:, None] * model.b[:, 0][None, :]
    correction = innovation[:, None] * design.observer_gain[:, 0][None, :]
    a_products = (
        model.a[None, :, :] * estimates[:, None, :]
    )
    for state_index, (state_name, unit) in enumerate(
        zip(state_names, state_units)
    ):
        observe(
            observations,
            f"observer_prediction_{state_name}",
            "observer_intermediate",
            unit,
            case_id,
            prediction[:, state_index],
        )
        observe(
            observations,
            f"observer_voltage_term_{state_name}",
            "observer_intermediate",
            unit,
            case_id,
            voltage_terms[:, state_index],
        )
        observe(
            observations,
            f"observer_correction_{state_name}",
            "observer_intermediate",
            unit,
            case_id,
            correction[:, state_index],
        )
        l1_sum = (
            np.sum(np.abs(a_products[:, state_index, :]), axis=1)
            + np.abs(voltage_terms[:, state_index])
            + np.abs(correction[:, state_index])
        )
        observe(
            observations,
            f"observer_{state_name}_l1_sum_of_products",
            "accumulator_bound",
            unit,
            case_id,
            l1_sum,
        )


def observe_closed_loop_result(
    observations: dict[str, ObservedRange],
    case_id: str,
    result: ClosedLoopResult | RobustnessResult,
    model: StateSpaceModel,
    design: ObserverControllerDesign,
) -> None:
    """Accumulate states, I/O, and arithmetic intermediates from one case."""

    state_names = ("position", "speed", "current")
    state_units = ("rad", "rad/s", "A")
    for index, (state_name, unit) in enumerate(zip(state_names, state_units)):
        observe(
            observations,
            f"true_{state_name}",
            "plant_state",
            unit,
            case_id,
            result.true_states[:, index],
        )
        observe(
            observations,
            f"estimated_{state_name}",
            "observer_state",
            unit,
            case_id,
            result.estimated_states[:, index],
        )
    observe(
        observations,
        "measured_position_rad",
        "sensor",
        "rad",
        case_id,
        result.measured_positions_rad,
    )
    observe(
        observations,
        "reference_position_rad",
        "control_input",
        "rad",
        case_id,
        result.references_rad,
    )
    observe(
        observations,
        "load_torque_nm",
        "disturbance",
        "N m",
        case_id,
        result.load_torques_nm,
    )
    observe(
        observations,
        "requested_voltage_v",
        "control_output",
        "V",
        case_id,
        result.requested_voltages_v,
    )
    commanded = getattr(
        result,
        "commanded_voltages_v",
        result.applied_voltages_v,
    )
    observe(
        observations,
        "commanded_voltage_v",
        "control_output",
        "V",
        case_id,
        commanded,
    )
    observe(
        observations,
        "applied_voltage_v",
        "control_output",
        "V",
        case_id,
        result.applied_voltages_v,
    )
    _observe_controller_intermediates(
        observations,
        case_id,
        result.references_rad,
        result.estimated_states,
        result.measured_positions_rad,
        commanded,
        model,
        design,
    )


def observe_observer_result(
    observations: dict[str, ObservedRange],
    case_id: str,
    result: ObserverResult,
    model: StateSpaceModel,
    design: ObserverControllerDesign,
) -> None:
    """Include the MODEL-010 observer-initialisation convergence case."""

    state_names = ("position", "speed", "current")
    state_units = ("rad", "rad/s", "A")
    for index, (state_name, unit) in enumerate(zip(state_names, state_units)):
        observe(
            observations,
            f"true_{state_name}",
            "plant_state",
            unit,
            case_id,
            result.true_states[:, index],
        )
        observe(
            observations,
            f"estimated_{state_name}",
            "observer_state",
            unit,
            case_id,
            result.estimated_states[:, index],
        )
    observe(
        observations,
        "measured_position_rad",
        "sensor",
        "rad",
        case_id,
        result.measured_positions_rad,
    )
    estimates = result.estimated_states[:-1]
    innovation = result.measured_positions_rad - (
        model.c @ estimates.T
    ).reshape(-1)
    observe(
        observations,
        "innovation_rad",
        "sensor",
        "rad",
        case_id,
        innovation,
    )
    prediction = (model.a @ estimates.T).T
    correction = innovation[:, None] * design.observer_gain[:, 0][None, :]
    a_products = model.a[None, :, :] * estimates[:, None, :]
    for state_index, (state_name, unit) in enumerate(
        zip(state_names, state_units)
    ):
        observe(
            observations,
            f"observer_prediction_{state_name}",
            "observer_intermediate",
            unit,
            case_id,
            prediction[:, state_index],
        )
        observe(
            observations,
            f"observer_correction_{state_name}",
            "observer_intermediate",
            unit,
            case_id,
            correction[:, state_index],
        )
        l1_sum = (
            np.sum(np.abs(a_products[:, state_index, :]), axis=1)
            + np.abs(correction[:, state_index])
        )
        observe(
            observations,
            f"observer_{state_name}_l1_sum_of_products",
            "accumulator_bound",
            unit,
            case_id,
            l1_sum,
        )


def finalize_range_records(
    observations: Mapping[str, ObservedRange],
    guard_factor: float,
    configured_hard_bounds: Mapping[str, float],
) -> list[dict[str, Any]]:
    """Apply guarded or configured bounds without choosing a binary point."""

    if not np.isfinite(guard_factor) or guard_factor < 1.0:
        raise ValueError("guard_factor must be finite and at least one")
    records: list[dict[str, Any]] = []
    for signal_id in sorted(observations):
        observed = observations[signal_id]
        if signal_id in configured_hard_bounds:
            budget_bound = float(configured_hard_bounds[signal_id])
            if (
                not np.isfinite(budget_bound)
                or budget_bound <= 0.0
                or budget_bound + 1e-12 < observed.observed_peak_abs
            ):
                raise ValueError(
                    f"configured hard bound does not contain {signal_id}"
                )
            basis = "configured_hard_bound"
            applied_guard = 1.0
        else:
            budget_bound = observed.observed_peak_abs * guard_factor
            basis = "observed_peak_times_synthetic_guard"
            applied_guard = guard_factor
        records.append(
            {
                "signal_id": signal_id,
                "group": observed.group,
                "unit": observed.unit,
                "observed_min": observed.observed_min,
                "observed_max": observed.observed_max,
                "observed_peak_abs": observed.observed_peak_abs,
                "peak_source_case": observed.peak_source_case,
                "budget_abs_bound": budget_bound,
                "budget_basis": basis,
                "guard_factor": applied_guard,
                "minimum_signed_integer_bits_including_sign": (
                    minimum_signed_integer_bits_including_sign(
                        budget_bound
                    )
                ),
                "fractional_bits_status": (
                    "TBD_PENDING_QUANTIZATION_ERROR_STUDY"
                ),
            }
        )
    return records


def coefficient_records(
    model: StateSpaceModel,
    design: ObserverControllerDesign,
) -> list[dict[str, Any]]:
    """Flatten implementation coefficients with derivation classifications."""

    if model.sample_period_s is None:
        raise ValueError("coefficient audit requires a discrete model")
    records: list[dict[str, Any]] = []
    state_names = ("position", "speed", "current")

    for row, target_name in enumerate(state_names):
        for column, source_name in enumerate(state_names):
            records.append(
                {
                    "coefficient_id": f"A_{target_name}_{source_name}",
                    "block": "observer_state_transition",
                    "value": float(model.a[row, column]),
                    "unit": "target-state-unit/source-state-unit",
                    "derivation": (
                        "Zero-order-hold discretisation of SYNTHETIC-DCM-001"
                    ),
                    "source_class": "derived_from_synthetic_inputs",
                }
            )
        records.append(
            {
                "coefficient_id": f"Bv_{target_name}",
                "block": "observer_voltage_input",
                "value": float(model.b[row, 0]),
                "unit": "target-state-unit/V",
                "derivation": (
                    "Zero-order-hold discretisation of SYNTHETIC-DCM-001"
                ),
                "source_class": "derived_from_synthetic_inputs",
            }
        )
        records.append(
            {
                "coefficient_id": f"K_{target_name}",
                "block": "state_feedback",
                "value": float(design.state_feedback_gain[0, row]),
                "unit": "V/source-state-unit",
                "derivation": (
                    "scipy.signal.place_poles on the nominal discrete model"
                ),
                "source_class": "derived_from_synthetic_inputs",
            }
        )
        records.append(
            {
                "coefficient_id": f"L_{target_name}",
                "block": "observer_innovation",
                "value": float(design.observer_gain[row, 0]),
                "unit": "target-state-unit/rad",
                "derivation": (
                    "dual scipy.signal.place_poles on the nominal discrete model"
                ),
                "source_class": "derived_from_synthetic_inputs",
            }
        )
    records.append(
        {
            "coefficient_id": "N_reference",
            "block": "reference_precompensator",
            "value": float(design.reference_gain),
            "unit": "V/rad",
            "derivation": (
                "Nominal discrete steady-state unit-gain linear solve"
            ),
            "source_class": "derived_from_synthetic_inputs",
        }
    )
    records.append(
        {
            "coefficient_id": "C_position",
            "block": "observer_output",
            "value": float(model.c[0, 0]),
            "unit": "rad/rad",
            "derivation": "Declared encoder-position output mapping",
            "source_class": "model_definition",
        }
    )
    return records


def coefficient_dynamic_range(
    records: list[dict[str, Any]],
) -> dict[str, float]:
    """Describe the nonzero coefficient span without assigning a format."""

    magnitudes = np.asarray(
        [abs(float(record["value"])) for record in records],
        dtype=np.float64,
    )
    nonzero = magnitudes[magnitudes > 0.0]
    if nonzero.size == 0:
        raise ValueError("at least one coefficient must be nonzero")
    maximum = float(np.max(nonzero))
    minimum = float(np.min(nonzero))
    ratio = maximum / minimum
    return {
        "smallest_nonzero_abs": minimum,
        "largest_abs": maximum,
        "ratio": ratio,
        "span_bits_log2": float(np.log2(ratio)),
    }
