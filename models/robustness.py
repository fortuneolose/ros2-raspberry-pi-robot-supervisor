"""Deterministic robustness utilities for the synthetic floating-point model.

MODEL-020 deliberately keeps the controller and observer designed for the
nominal synthetic model while varying the simulated plant and nonidealities.
The analysis is a software-development fixture, not evidence that any physical
motor lies inside the configured uncertainty envelope.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Mapping, Sequence

import numpy as np

from models.control import ObserverControllerDesign
from models.dc_motor import FloatArray, MotorParameters, StateSpaceModel

_PARAMETER_NAMES = frozenset(MotorParameters.__dataclass_fields__)


@dataclass(frozen=True)
class RobustnessScenario:
    """One deterministic plant or implementation-uncertainty scenario."""

    scenario_id: str
    category: str
    description: str
    parameter_multipliers: Mapping[str, float]
    measurement_noise_std_rad: float = 0.0
    encoder_counts_per_revolution: int | None = None
    control_delay_samples: int = 0
    voltage_limit_v: float = 6.0

    def validate(self) -> None:
        """Reject malformed or physically meaningless scenario values."""

        if not self.scenario_id.strip():
            raise ValueError("scenario_id must not be empty")
        if not self.category.strip():
            raise ValueError("category must not be empty")
        unknown = set(self.parameter_multipliers) - _PARAMETER_NAMES
        if unknown:
            raise ValueError(
                "unknown motor parameter multiplier(s): "
                + ", ".join(sorted(unknown))
            )
        for name, multiplier in self.parameter_multipliers.items():
            if not np.isfinite(multiplier) or multiplier <= 0.0:
                raise ValueError(
                    f"parameter multiplier {name} must be finite and positive"
                )
        if (
            not np.isfinite(self.measurement_noise_std_rad)
            or self.measurement_noise_std_rad < 0.0
        ):
            raise ValueError(
                "measurement_noise_std_rad must be finite and nonnegative"
            )
        if (
            self.encoder_counts_per_revolution is not None
            and (
                isinstance(self.encoder_counts_per_revolution, bool)
                or not isinstance(self.encoder_counts_per_revolution, int)
                or self.encoder_counts_per_revolution < 4
            )
        ):
            raise ValueError(
                "encoder_counts_per_revolution must be an integer of at least 4"
            )
        if (
            isinstance(self.control_delay_samples, bool)
            or not isinstance(self.control_delay_samples, int)
            or self.control_delay_samples < 0
        ):
            raise ValueError("control_delay_samples must be a nonnegative integer")
        if not np.isfinite(self.voltage_limit_v) or self.voltage_limit_v <= 0.0:
            raise ValueError("voltage_limit_v must be finite and positive")


@dataclass(frozen=True)
class RobustnessResult:
    """Sampled response of a mismatched plant and nominal observer/controller."""

    times_s: FloatArray
    references_rad: FloatArray
    load_torques_nm: FloatArray
    true_states: FloatArray
    estimated_states: FloatArray
    measured_positions_rad: FloatArray
    requested_voltages_v: FloatArray
    commanded_voltages_v: FloatArray
    applied_voltages_v: FloatArray
    saturated: np.ndarray


def scaled_motor_parameters(
    nominal: MotorParameters,
    multipliers: Mapping[str, float],
) -> MotorParameters:
    """Return a validated parameter set after named multiplicative changes."""

    scenario = RobustnessScenario(
        scenario_id="parameter-validation",
        category="validation",
        description="internal parameter validation",
        parameter_multipliers=multipliers,
    )
    scenario.validate()
    values = asdict(nominal)
    for name, multiplier in multipliers.items():
        values[name] *= float(multiplier)
    return MotorParameters.from_mapping(values)


def quantize_encoder_position(
    position_rad: float,
    counts_per_revolution: int | None,
) -> float:
    """Round a position measurement to the nearest encoder count."""

    if counts_per_revolution is None:
        return float(position_rad)
    if (
        isinstance(counts_per_revolution, bool)
        or not isinstance(counts_per_revolution, int)
        or counts_per_revolution < 4
    ):
        raise ValueError("counts_per_revolution must be an integer of at least 4")
    count_angle_rad = 2.0 * np.pi / counts_per_revolution
    return float(np.round(position_rad / count_angle_rad) * count_angle_rad)


def _validate_models(
    plant_model: StateSpaceModel,
    observer_model: StateSpaceModel,
) -> None:
    if plant_model.sample_period_s is None or observer_model.sample_period_s is None:
        raise ValueError("plant and observer models must be discrete")
    if not np.isclose(
        plant_model.sample_period_s,
        observer_model.sample_period_s,
        rtol=0.0,
        atol=1e-15,
    ):
        raise ValueError("plant and observer sample periods must match")
    if plant_model.a.shape != observer_model.a.shape:
        raise ValueError("plant and observer state dimensions must match")
    if plant_model.b.shape[1] < 2 or observer_model.b.shape[1] < 2:
        raise ValueError("models must expose voltage and load-torque inputs")
    if plant_model.c.shape != observer_model.c.shape:
        raise ValueError("plant and observer output dimensions must match")


def simulate_mismatched_observer_feedback(
    plant_model: StateSpaceModel,
    observer_model: StateSpaceModel,
    design: ObserverControllerDesign,
    references_rad: Sequence[float] | FloatArray,
    load_torques_nm: Sequence[float] | FloatArray,
    scenario: RobustnessScenario,
    random_seed: int,
) -> RobustnessResult:
    """Simulate a varied plant with a controller fixed to the nominal design.

    The observer uses the current limited command. A configured command delay
    affects the plant but is intentionally absent from the observer model,
    representing an unmodelled transport or actuation delay.
    """

    scenario.validate()
    _validate_models(plant_model, observer_model)
    references = np.asarray(references_rad, dtype=np.float64)
    load_torques = np.asarray(load_torques_nm, dtype=np.float64)
    if references.ndim != 1 or references.size == 0:
        raise ValueError("references_rad must be a non-empty vector")
    if load_torques.shape != references.shape:
        raise ValueError("load_torques_nm must match references_rad")
    if not np.all(np.isfinite(references)) or not np.all(np.isfinite(load_torques)):
        raise ValueError("references and load torques must be finite")
    if (
        isinstance(random_seed, bool)
        or not isinstance(random_seed, int)
        or random_seed < 0
    ):
        raise ValueError("random_seed must be a nonnegative integer")

    sample_count = references.size
    state_count = plant_model.a.shape[0]
    true_states = np.zeros((sample_count + 1, state_count), dtype=np.float64)
    estimated_states = np.zeros_like(true_states)
    measured_positions = np.zeros(sample_count, dtype=np.float64)
    requested_voltages = np.zeros(sample_count, dtype=np.float64)
    commanded_voltages = np.zeros(sample_count, dtype=np.float64)
    applied_voltages = np.zeros(sample_count, dtype=np.float64)
    saturated = np.zeros(sample_count, dtype=np.bool_)

    rng = np.random.default_rng(random_seed)
    noise = rng.normal(
        loc=0.0,
        scale=scenario.measurement_noise_std_rad,
        size=sample_count,
    )
    delay_line = np.zeros(scenario.control_delay_samples, dtype=np.float64)
    plant_voltage_input = plant_model.b[:, 0]
    plant_load_input = plant_model.b[:, 1]
    observer_voltage_input = observer_model.b[:, 0]

    for index in range(sample_count):
        raw_measurement = float(
            (plant_model.c @ true_states[index]).item()
        ) + noise[index]
        measured_positions[index] = quantize_encoder_position(
            raw_measurement,
            scenario.encoder_counts_per_revolution,
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
        commanded_voltages[index] = np.clip(
            requested_voltages[index],
            -scenario.voltage_limit_v,
            scenario.voltage_limit_v,
        )
        saturated[index] = not np.isclose(
            requested_voltages[index],
            commanded_voltages[index],
        )

        if scenario.control_delay_samples == 0:
            applied_voltages[index] = commanded_voltages[index]
        else:
            applied_voltages[index] = delay_line[0]
            delay_line[:-1] = delay_line[1:]
            delay_line[-1] = commanded_voltages[index]

        true_states[index + 1] = (
            plant_model.a @ true_states[index]
            + plant_voltage_input * applied_voltages[index]
            + plant_load_input * load_torques[index]
        )
        innovation = measured_positions[index] - float(
            (observer_model.c @ estimated_states[index]).item()
        )
        estimated_states[index + 1] = (
            observer_model.a @ estimated_states[index]
            + observer_voltage_input * commanded_voltages[index]
            + design.observer_gain[:, 0] * innovation
        )

    times = (
        np.arange(sample_count + 1, dtype=np.float64)
        * float(plant_model.sample_period_s)
    )
    return RobustnessResult(
        times_s=times,
        references_rad=references,
        load_torques_nm=load_torques,
        true_states=true_states,
        estimated_states=estimated_states,
        measured_positions_rad=measured_positions,
        requested_voltages_v=requested_voltages,
        commanded_voltages_v=commanded_voltages,
        applied_voltages_v=applied_voltages,
        saturated=saturated,
    )


def zero_delay_augmented_spectral_radius(
    plant_model: StateSpaceModel,
    observer_model: StateSpaceModel,
    design: ObserverControllerDesign,
) -> float:
    """Return the nominal-observer/varied-plant linearized spectral radius.

    This calculation excludes command delay, quantisation, noise, saturation,
    and reference input. Those effects are covered by time-domain scenarios.
    """

    _validate_models(plant_model, observer_model)
    plant_voltage = plant_model.b[:, [0]]
    observer_voltage = observer_model.b[:, [0]]
    gain = design.state_feedback_gain
    observer_gain = design.observer_gain
    augmented = np.block(
        [
            [plant_model.a, -plant_voltage @ gain],
            [
                observer_gain @ plant_model.c,
                observer_model.a
                - observer_voltage @ gain
                - observer_gain @ observer_model.c,
            ],
        ]
    )
    return float(np.max(np.abs(np.linalg.eigvals(augmented))))


def summarize_result(
    result: RobustnessResult,
    reference_rad: float,
) -> dict[str, float | int | bool]:
    """Calculate unit-bearing integrity and performance descriptors."""

    if not np.isfinite(reference_rad) or reference_rad <= 0.0:
        raise ValueError("reference_rad must be finite and positive")
    tracking_error = reference_rad - result.true_states[:-1, 0]
    estimation_error = result.true_states - result.estimated_states
    tail_count = max(10, result.references_rad.size // 10)
    arrays = (
        result.true_states,
        result.estimated_states,
        result.measured_positions_rad,
        result.requested_voltages_v,
        result.commanded_voltages_v,
        result.applied_voltages_v,
    )
    return {
        "all_values_finite": bool(all(np.all(np.isfinite(item)) for item in arrays)),
        "rms_tracking_error_rad": float(
            np.sqrt(np.mean(tracking_error**2))
        ),
        "tail_mean_absolute_tracking_error_rad": float(
            abs(np.mean(tracking_error[-tail_count:]))
        ),
        "peak_absolute_position_rad": float(
            np.max(np.abs(result.true_states[:, 0]))
        ),
        "peak_absolute_speed_rad_s": float(
            np.max(np.abs(result.true_states[:, 1]))
        ),
        "peak_absolute_current_a": float(
            np.max(np.abs(result.true_states[:, 2]))
        ),
        "rms_position_estimation_error_rad": float(
            np.sqrt(np.mean(estimation_error[:, 0] ** 2))
        ),
        "maximum_absolute_applied_voltage_v": float(
            np.max(np.abs(result.applied_voltages_v))
        ),
        "saturated_sample_count": int(np.count_nonzero(result.saturated)),
    }
