"""Floating-point state-feedback and observer utilities.

The routines in this module operate on the parameterized motor-model interface
and make no claim about final hardware gains.  The repository's current
controller fixture is synthetic and exists to verify the software workflow.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from numpy.typing import NDArray
from scipy.signal import place_poles

from models.dc_motor import (
    FloatArray,
    StateSpaceModel,
    controllability_matrix,
    observability_matrix,
)

BoolArray = NDArray[np.bool_]


@dataclass(frozen=True)
class ObserverControllerDesign:
    """Discrete state-feedback, precompensator, and observer design."""

    state_feedback_gain: FloatArray
    reference_gain: float
    observer_gain: FloatArray
    requested_controller_poles: NDArray[np.complex128]
    requested_observer_poles: NDArray[np.complex128]
    achieved_controller_poles: NDArray[np.complex128]
    achieved_observer_poles: NDArray[np.complex128]


@dataclass(frozen=True)
class ClosedLoopResult:
    """Sampled result from the observer-based feedback simulation."""

    times_s: FloatArray
    references_rad: FloatArray
    load_torques_nm: FloatArray
    true_states: FloatArray
    estimated_states: FloatArray
    measured_positions_rad: FloatArray
    requested_voltages_v: FloatArray
    applied_voltages_v: FloatArray
    saturated: BoolArray


@dataclass(frozen=True)
class ObserverResult:
    """Sampled result from an observer driven by known plant inputs."""

    times_s: FloatArray
    true_states: FloatArray
    estimated_states: FloatArray
    measured_positions_rad: FloatArray


def continuous_poles_to_discrete(
    continuous_poles_rad_s: Sequence[complex | float],
    sample_period_s: float,
) -> NDArray[np.complex128]:
    """Map continuous-domain pole targets to the z plane."""

    poles = np.asarray(continuous_poles_rad_s, dtype=np.complex128)
    if poles.ndim != 1 or poles.size == 0:
        raise ValueError("continuous poles must be a non-empty one-dimensional array")
    if not np.all(np.isfinite(poles)):
        raise ValueError("continuous poles must be finite")
    if np.any(np.real(poles) >= 0.0):
        raise ValueError("continuous pole targets must have negative real parts")
    if not np.isfinite(sample_period_s) or sample_period_s <= 0.0:
        raise ValueError("sample_period_s must be finite and greater than zero")
    return np.exp(poles * sample_period_s)


def _validate_discrete_poles(
    poles: Sequence[complex | float],
    state_count: int,
    name: str,
) -> NDArray[np.complex128]:
    values = np.asarray(poles, dtype=np.complex128)
    if values.shape != (state_count,):
        raise ValueError(f"{name} must contain exactly {state_count} poles")
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must be finite")
    if np.any(np.abs(values) >= 1.0):
        raise ValueError(f"{name} must lie strictly inside the unit circle")
    return values


def design_observer_controller(
    model: StateSpaceModel,
    controller_poles: Sequence[complex | float],
    observer_poles: Sequence[complex | float],
) -> ObserverControllerDesign:
    """Design voltage-input feedback and a position-output observer.

    The controller is ``v[k] = -K x_hat[k] + N r[k]``.  The observer uses
    ``x_hat[k+1] = A x_hat[k] + B_v v[k] + L(y[k] - C x_hat[k])`` and treats
    load torque as an unmeasured disturbance.
    """

    if model.sample_period_s is None:
        raise ValueError("controller design requires a discrete model")
    state_count = model.a.shape[0]
    if model.b.shape[1] < 2:
        raise ValueError("model must expose voltage and load-torque inputs")
    if model.c.shape != (1, state_count):
        raise ValueError("design requires one measured position output")
    if not np.allclose(model.d, 0.0):
        raise ValueError("design currently requires zero direct feedthrough")

    requested_controller = _validate_discrete_poles(
        controller_poles, state_count, "controller_poles"
    )
    requested_observer = _validate_discrete_poles(
        observer_poles, state_count, "observer_poles"
    )
    voltage_input = model.b[:, [0]]
    if np.linalg.matrix_rank(
        controllability_matrix(model.a, voltage_input)
    ) != state_count:
        raise ValueError("model is not controllable from armature voltage")
    if np.linalg.matrix_rank(
        observability_matrix(model.a, model.c)
    ) != state_count:
        raise ValueError("model is not observable from encoder position")

    state_feedback_gain = np.asarray(
        place_poles(
            model.a, voltage_input, requested_controller
        ).gain_matrix,
        dtype=np.float64,
    )
    observer_gain = np.asarray(
        place_poles(
            model.a.T, model.c.T, requested_observer
        ).gain_matrix.T,
        dtype=np.float64,
    )

    steady_state_response = model.c @ np.linalg.solve(
        np.eye(state_count) - model.a + voltage_input @ state_feedback_gain,
        voltage_input,
    )
    denominator = float(steady_state_response.item())
    if not np.isfinite(denominator) or np.isclose(denominator, 0.0):
        raise ValueError("reference precompensator is singular")
    reference_gain = 1.0 / denominator

    achieved_controller = np.linalg.eigvals(
        model.a - voltage_input @ state_feedback_gain
    ).astype(np.complex128)
    achieved_observer = np.linalg.eigvals(
        model.a - observer_gain @ model.c
    ).astype(np.complex128)
    return ObserverControllerDesign(
        state_feedback_gain=state_feedback_gain,
        reference_gain=reference_gain,
        observer_gain=observer_gain,
        requested_controller_poles=requested_controller,
        requested_observer_poles=requested_observer,
        achieved_controller_poles=achieved_controller,
        achieved_observer_poles=achieved_observer,
    )


def _initial_state(
    values: Sequence[float] | FloatArray | None,
    state_count: int,
    name: str,
) -> FloatArray:
    if values is None:
        return np.zeros(state_count, dtype=np.float64)
    state = np.asarray(values, dtype=np.float64)
    if state.shape != (state_count,) or not np.all(np.isfinite(state)):
        raise ValueError(f"{name} must contain {state_count} finite values")
    return state


def simulate_observer_feedback(
    model: StateSpaceModel,
    design: ObserverControllerDesign,
    references_rad: Sequence[float] | FloatArray,
    load_torques_nm: Sequence[float] | FloatArray,
    voltage_limit_v: float,
    initial_state: Sequence[float] | FloatArray | None = None,
    initial_estimate: Sequence[float] | FloatArray | None = None,
    measurement_noise_rad: Sequence[float] | FloatArray | None = None,
) -> ClosedLoopResult:
    """Simulate saturated observer-based position feedback."""

    if model.sample_period_s is None:
        raise ValueError("simulation requires a discrete model")
    references = np.asarray(references_rad, dtype=np.float64)
    load_torques = np.asarray(load_torques_nm, dtype=np.float64)
    if references.ndim != 1 or references.size == 0:
        raise ValueError("references_rad must be a non-empty one-dimensional array")
    if load_torques.shape != references.shape:
        raise ValueError("load_torques_nm must match references_rad")
    if not np.all(np.isfinite(references)) or not np.all(np.isfinite(load_torques)):
        raise ValueError("references and load torques must be finite")
    if not np.isfinite(voltage_limit_v) or voltage_limit_v <= 0.0:
        raise ValueError("voltage_limit_v must be finite and greater than zero")

    if measurement_noise_rad is None:
        measurement_noise = np.zeros_like(references)
    else:
        measurement_noise = np.asarray(measurement_noise_rad, dtype=np.float64)
        if measurement_noise.shape != references.shape:
            raise ValueError("measurement_noise_rad must match references_rad")
        if not np.all(np.isfinite(measurement_noise)):
            raise ValueError("measurement_noise_rad must be finite")

    sample_count = references.size
    state_count = model.a.shape[0]
    true_states = np.zeros((sample_count + 1, state_count), dtype=np.float64)
    estimated_states = np.zeros_like(true_states)
    measured_positions = np.zeros(sample_count, dtype=np.float64)
    requested_voltages = np.zeros(sample_count, dtype=np.float64)
    applied_voltages = np.zeros(sample_count, dtype=np.float64)
    saturated = np.zeros(sample_count, dtype=np.bool_)
    true_states[0] = _initial_state(initial_state, state_count, "initial_state")
    estimated_states[0] = _initial_state(
        initial_estimate, state_count, "initial_estimate"
    )

    voltage_input = model.b[:, 0]
    load_input = model.b[:, 1]
    for index in range(sample_count):
        measured_positions[index] = (
            float((model.c @ true_states[index]).item())
            + measurement_noise[index]
        )
        requested_voltages[index] = (
            -float(
                (
                    design.state_feedback_gain
                    @ estimated_states[index]
                ).item()
            )
            + design.reference_gain * references[index]
        )
        applied_voltages[index] = np.clip(
            requested_voltages[index], -voltage_limit_v, voltage_limit_v
        )
        saturated[index] = not np.isclose(
            applied_voltages[index], requested_voltages[index]
        )

        true_states[index + 1] = (
            model.a @ true_states[index]
            + voltage_input * applied_voltages[index]
            + load_input * load_torques[index]
        )
        innovation = measured_positions[index] - float(
            (model.c @ estimated_states[index]).item()
        )
        estimated_states[index + 1] = (
            model.a @ estimated_states[index]
            + voltage_input * applied_voltages[index]
            + design.observer_gain[:, 0] * innovation
        )

    times = (
        np.arange(sample_count + 1, dtype=np.float64)
        * model.sample_period_s
    )
    return ClosedLoopResult(
        times_s=times,
        references_rad=references,
        load_torques_nm=load_torques,
        true_states=true_states,
        estimated_states=estimated_states,
        measured_positions_rad=measured_positions,
        requested_voltages_v=requested_voltages,
        applied_voltages_v=applied_voltages,
        saturated=saturated,
    )


def simulate_state_observer(
    model: StateSpaceModel,
    observer_gain: FloatArray,
    inputs: FloatArray,
    initial_state: Sequence[float] | FloatArray,
    initial_estimate: Sequence[float] | FloatArray | None = None,
) -> ObserverResult:
    """Simulate observer convergence when all applied inputs are known."""

    if model.sample_period_s is None:
        raise ValueError("simulation requires a discrete model")
    input_values = np.asarray(inputs, dtype=np.float64)
    if (
        input_values.ndim != 2
        or input_values.shape[1] != model.b.shape[1]
        or input_values.shape[0] == 0
    ):
        raise ValueError(
            f"inputs must have shape (samples, {model.b.shape[1]})"
        )
    if not np.all(np.isfinite(input_values)):
        raise ValueError("inputs must be finite")

    sample_count = input_values.shape[0]
    state_count = model.a.shape[0]
    gain = np.asarray(observer_gain, dtype=np.float64)
    if gain.shape != (state_count, model.c.shape[0]):
        raise ValueError("observer_gain has incompatible dimensions")
    true_states = np.zeros((sample_count + 1, state_count), dtype=np.float64)
    estimated_states = np.zeros_like(true_states)
    measured_positions = np.zeros(sample_count, dtype=np.float64)
    true_states[0] = _initial_state(initial_state, state_count, "initial_state")
    estimated_states[0] = _initial_state(
        initial_estimate, state_count, "initial_estimate"
    )

    for index, input_vector in enumerate(input_values):
        measured_positions[index] = float(
            (model.c @ true_states[index]).item()
        )
        innovation = measured_positions[index] - float(
            (model.c @ estimated_states[index]).item()
        )
        true_states[index + 1] = (
            model.a @ true_states[index] + model.b @ input_vector
        )
        estimated_states[index + 1] = (
            model.a @ estimated_states[index]
            + model.b @ input_vector
            + gain[:, 0] * innovation
        )

    times = (
        np.arange(sample_count + 1, dtype=np.float64)
        * model.sample_period_s
    )
    return ObserverResult(
        times_s=times,
        true_states=true_states,
        estimated_states=estimated_states,
        measured_positions_rad=measured_positions,
    )


def step_response_metrics(
    times_s: FloatArray,
    positions_rad: FloatArray,
    reference_rad: float,
    settling_band_fraction: float = 0.02,
) -> dict[str, float]:
    """Calculate standard positive-step development metrics."""

    times = np.asarray(times_s, dtype=np.float64)
    positions = np.asarray(positions_rad, dtype=np.float64)
    if times.shape != positions.shape or times.ndim != 1 or times.size == 0:
        raise ValueError("times_s and positions_rad must be equal non-empty vectors")
    if reference_rad <= 0.0 or not np.isfinite(reference_rad):
        raise ValueError("reference_rad must be finite and greater than zero")
    if not 0.0 < settling_band_fraction < 1.0:
        raise ValueError("settling_band_fraction must lie between zero and one")

    def first_crossing(level: float) -> int:
        candidates = np.flatnonzero(positions >= level)
        if candidates.size == 0:
            raise ValueError("response does not reach the requested rise-time level")
        return int(candidates[0])

    ten_percent_index = first_crossing(0.1 * reference_rad)
    ninety_percent_index = first_crossing(0.9 * reference_rad)
    absolute_error = np.abs(reference_rad - positions)
    settling_limit = settling_band_fraction * reference_rad
    settling_index: int | None = None
    for index in range(positions.size):
        if np.all(absolute_error[index:] <= settling_limit):
            settling_index = index
            break
    if settling_index is None:
        raise ValueError("response does not settle within the supplied record")

    tail_count = max(10, positions.size // 10)
    steady_state_position = float(np.mean(positions[-tail_count:]))
    return {
        "rise_time_10_90_s": float(
            times[ninety_percent_index] - times[ten_percent_index]
        ),
        "settling_time_2_percent_s": float(times[settling_index]),
        "overshoot_percent": float(
            max(0.0, (np.max(positions) - reference_rad) / reference_rad * 100.0)
        ),
        "steady_state_error_rad": float(
            abs(reference_rad - steady_state_position)
        ),
        "rms_tracking_error_rad": float(
            np.sqrt(np.mean((reference_rad - positions) ** 2))
        ),
    }


def suffix_entry_time(
    times_s: FloatArray,
    values: FloatArray,
    limit: float,
    start_index: int = 0,
) -> float:
    """Return time after ``start_index`` when values stay within a limit."""

    times = np.asarray(times_s, dtype=np.float64)
    magnitudes = np.asarray(values, dtype=np.float64)
    if times.shape != magnitudes.shape or times.ndim != 1:
        raise ValueError("times_s and values must have equal vector shapes")
    if not 0 <= start_index < magnitudes.size:
        raise ValueError("start_index is outside the supplied record")
    if not np.isfinite(limit) or limit < 0.0:
        raise ValueError("limit must be finite and nonnegative")
    for index in range(start_index, values.size):
        if np.all(np.abs(magnitudes[index:]) <= limit):
            return float(times[index] - times[start_index])
    raise ValueError("values do not remain within the limit")
