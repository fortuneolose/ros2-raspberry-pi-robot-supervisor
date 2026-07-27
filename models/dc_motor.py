"""Parameterized floating-point model of an encoder-equipped DC motor.

This module intentionally contains no hardware-specific parameter assumptions.
The included JSON parameter file is synthetic and exists only to make the
software workflow executable before a motor is selected and identified.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np
from numpy.typing import NDArray
from scipy.signal import cont2discrete

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class MotorParameters:
    """Physical parameters referred to the modelled motor shaft."""

    armature_resistance_ohm: float
    armature_inductance_h: float
    torque_constant_nm_per_a: float
    back_emf_constant_v_per_rad_s: float
    rotor_inertia_kg_m2: float
    viscous_friction_nm_s_per_rad: float

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "MotorParameters":
        parameters = cls(
            armature_resistance_ohm=float(values["armature_resistance_ohm"]),
            armature_inductance_h=float(values["armature_inductance_h"]),
            torque_constant_nm_per_a=float(values["torque_constant_nm_per_a"]),
            back_emf_constant_v_per_rad_s=float(
                values["back_emf_constant_v_per_rad_s"]
            ),
            rotor_inertia_kg_m2=float(values["rotor_inertia_kg_m2"]),
            viscous_friction_nm_s_per_rad=float(
                values["viscous_friction_nm_s_per_rad"]
            ),
        )
        parameters.validate()
        return parameters

    def validate(self) -> None:
        for field_name, value in self.__dict__.items():
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{field_name} must be finite and greater than zero")


@dataclass(frozen=True)
class StateSpaceModel:
    """State-space matrices with the associated engineering metadata."""

    a: FloatArray
    b: FloatArray
    c: FloatArray
    d: FloatArray
    state_names: tuple[str, ...]
    state_units: tuple[str, ...]
    input_names: tuple[str, ...]
    input_units: tuple[str, ...]
    output_names: tuple[str, ...]
    output_units: tuple[str, ...]
    sample_period_s: float | None = None


def continuous_dc_motor_model(parameters: MotorParameters) -> StateSpaceModel:
    """Return the continuous model for x = [theta, omega, current].

    Inputs are armature voltage and opposing load torque. Encoder position is
    the measured output. The load-torque sign is negative in the speed
    equation, so positive load torque opposes positive rotation.
    """

    parameters.validate()

    resistance = parameters.armature_resistance_ohm
    inductance = parameters.armature_inductance_h
    torque_constant = parameters.torque_constant_nm_per_a
    back_emf_constant = parameters.back_emf_constant_v_per_rad_s
    inertia = parameters.rotor_inertia_kg_m2
    friction = parameters.viscous_friction_nm_s_per_rad

    a = np.array(
        [
            [0.0, 1.0, 0.0],
            [0.0, -friction / inertia, torque_constant / inertia],
            [0.0, -back_emf_constant / inductance, -resistance / inductance],
        ],
        dtype=np.float64,
    )
    b = np.array(
        [
            [0.0, 0.0],
            [0.0, -1.0 / inertia],
            [1.0 / inductance, 0.0],
        ],
        dtype=np.float64,
    )
    c = np.array([[1.0, 0.0, 0.0]], dtype=np.float64)
    d = np.zeros((1, 2), dtype=np.float64)

    return StateSpaceModel(
        a=a,
        b=b,
        c=c,
        d=d,
        state_names=("shaft_position", "shaft_speed", "armature_current"),
        state_units=("rad", "rad/s", "A"),
        input_names=("armature_voltage", "opposing_load_torque"),
        input_units=("V", "N m"),
        output_names=("encoder_position",),
        output_units=("rad",),
    )


def discretize_zero_order_hold(
    continuous_model: StateSpaceModel, sample_period_s: float
) -> StateSpaceModel:
    """Discretize a continuous model using a zero-order hold."""

    if continuous_model.sample_period_s is not None:
        raise ValueError("expected a continuous model")
    if not np.isfinite(sample_period_s) or sample_period_s <= 0.0:
        raise ValueError("sample_period_s must be finite and greater than zero")

    a_d, b_d, c_d, d_d, returned_period = cont2discrete(
        (
            continuous_model.a,
            continuous_model.b,
            continuous_model.c,
            continuous_model.d,
        ),
        dt=sample_period_s,
        method="zoh",
    )
    return StateSpaceModel(
        a=np.asarray(a_d, dtype=np.float64),
        b=np.asarray(b_d, dtype=np.float64),
        c=np.asarray(c_d, dtype=np.float64),
        d=np.asarray(d_d, dtype=np.float64),
        state_names=continuous_model.state_names,
        state_units=continuous_model.state_units,
        input_names=continuous_model.input_names,
        input_units=continuous_model.input_units,
        output_names=continuous_model.output_names,
        output_units=continuous_model.output_units,
        sample_period_s=float(returned_period),
    )


def controllability_matrix(a: FloatArray, b: FloatArray) -> FloatArray:
    """Construct [B, AB, ..., A^(n-1)B]."""

    state_count = a.shape[0]
    return np.hstack(
        [np.linalg.matrix_power(a, power) @ b for power in range(state_count)]
    )


def observability_matrix(a: FloatArray, c: FloatArray) -> FloatArray:
    """Construct [C; CA; ...; CA^(n-1)]."""

    state_count = a.shape[0]
    return np.vstack([c @ np.linalg.matrix_power(a, power) for power in range(state_count)])


def evaluate_structure(model: StateSpaceModel) -> dict[str, int | bool]:
    """Evaluate voltage-input controllability and encoder observability."""

    state_count = model.a.shape[0]
    voltage_input = model.b[:, [0]]
    controllability_rank = int(
        np.linalg.matrix_rank(controllability_matrix(model.a, voltage_input))
    )
    observability_rank = int(
        np.linalg.matrix_rank(observability_matrix(model.a, model.c))
    )
    return {
        "state_count": state_count,
        "voltage_input_controllability_rank": controllability_rank,
        "encoder_position_observability_rank": observability_rank,
        "fully_controllable_from_voltage": controllability_rank == state_count,
        "fully_observable_from_encoder_position": observability_rank == state_count,
    }


def simulate_discrete(
    model: StateSpaceModel,
    inputs: FloatArray,
    initial_state: FloatArray | None = None,
) -> tuple[FloatArray, FloatArray]:
    """Simulate a discrete model and return state and output sequences."""

    if model.sample_period_s is None:
        raise ValueError("simulation requires a discrete model")

    input_values = np.asarray(inputs, dtype=np.float64)
    if input_values.ndim != 2 or input_values.shape[1] != model.b.shape[1]:
        raise ValueError(
            f"inputs must have shape (steps, {model.b.shape[1]})"
        )
    if not np.all(np.isfinite(input_values)):
        raise ValueError("inputs must contain only finite values")

    state_count = model.a.shape[0]
    output_count = model.c.shape[0]
    states = np.zeros((input_values.shape[0] + 1, state_count), dtype=np.float64)
    outputs = np.zeros((input_values.shape[0], output_count), dtype=np.float64)

    if initial_state is not None:
        initial = np.asarray(initial_state, dtype=np.float64)
        if initial.shape != (state_count,) or not np.all(np.isfinite(initial)):
            raise ValueError(
                f"initial_state must contain {state_count} finite values"
            )
        states[0] = initial

    for index, input_vector in enumerate(input_values):
        outputs[index] = model.c @ states[index] + model.d @ input_vector
        states[index + 1] = model.a @ states[index] + model.b @ input_vector

    return states, outputs
