"""Floating-point reference models for the robot supervisor project."""

from .dc_motor import (
    MotorParameters,
    StateSpaceModel,
    continuous_dc_motor_model,
    controllability_matrix,
    discretize_zero_order_hold,
    evaluate_structure,
    observability_matrix,
    simulate_discrete,
)

__all__ = [
    "MotorParameters",
    "StateSpaceModel",
    "continuous_dc_motor_model",
    "controllability_matrix",
    "discretize_zero_order_hold",
    "evaluate_structure",
    "observability_matrix",
    "simulate_discrete",
]
