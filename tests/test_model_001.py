"""MODEL-001 development tests for the synthetic plant baseline."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

import numpy as np

from models.dc_motor import (
    MotorParameters,
    continuous_dc_motor_model,
    discretize_zero_order_hold,
    evaluate_structure,
    simulate_discrete,
)

ROOT = Path(__file__).resolve().parents[1]
PARAMETER_FILE = ROOT / "models" / "parameters" / "synthetic_motor.json"


def load_fixture() -> tuple[MotorParameters, float]:
    payload = json.loads(PARAMETER_FILE.read_text(encoding="utf-8"))
    return (
        MotorParameters.from_mapping(payload["parameters"]),
        float(payload["sample_period_s"]),
    )


class TestMotorParameters(unittest.TestCase):
    def test_rejects_nonpositive_values(self) -> None:
        with self.assertRaises(ValueError):
            MotorParameters(
                armature_resistance_ohm=0.0,
                armature_inductance_h=0.5,
                torque_constant_nm_per_a=0.01,
                back_emf_constant_v_per_rad_s=0.01,
                rotor_inertia_kg_m2=0.01,
                viscous_friction_nm_s_per_rad=0.1,
            ).validate()


class TestModel001SyntheticBaseline(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        parameters, sample_period_s = load_fixture()
        cls.continuous = continuous_dc_motor_model(parameters)
        cls.discrete = discretize_zero_order_hold(
            cls.continuous, sample_period_s
        )

    def test_documents_expected_signal_dimensions(self) -> None:
        self.assertEqual(self.continuous.a.shape, (3, 3))
        self.assertEqual(self.continuous.b.shape, (3, 2))
        self.assertEqual(self.continuous.c.shape, (1, 3))
        self.assertEqual(self.continuous.d.shape, (1, 2))
        self.assertEqual(
            self.continuous.state_units,
            ("rad", "rad/s", "A"),
        )
        self.assertEqual(self.continuous.input_units, ("V", "N m"))

    def test_zero_order_hold_preserves_declared_sample_period(self) -> None:
        self.assertAlmostEqual(self.discrete.sample_period_s or 0.0, 0.001)
        self.assertTrue(np.all(np.isfinite(self.discrete.a)))
        self.assertTrue(np.all(np.isfinite(self.discrete.b)))

    def test_continuous_model_is_controllable_from_voltage(self) -> None:
        result = evaluate_structure(self.continuous)
        self.assertTrue(result["fully_controllable_from_voltage"])
        self.assertEqual(result["voltage_input_controllability_rank"], 3)

    def test_continuous_model_is_observable_from_encoder_position(self) -> None:
        result = evaluate_structure(self.continuous)
        self.assertTrue(result["fully_observable_from_encoder_position"])
        self.assertEqual(result["encoder_position_observability_rank"], 3)

    def test_discrete_model_preserves_structural_ranks(self) -> None:
        result = evaluate_structure(self.discrete)
        self.assertTrue(result["fully_controllable_from_voltage"])
        self.assertTrue(result["fully_observable_from_encoder_position"])

    def test_zero_input_preserves_zero_state(self) -> None:
        states, outputs = simulate_discrete(
            self.discrete, np.zeros((100, 2), dtype=np.float64)
        )
        np.testing.assert_allclose(states, 0.0)
        np.testing.assert_allclose(outputs, 0.0)

    def test_positive_voltage_produces_finite_positive_response(self) -> None:
        inputs = np.zeros((1000, 2), dtype=np.float64)
        inputs[:, 0] = 1.0
        states, outputs = simulate_discrete(self.discrete, inputs)
        self.assertTrue(np.all(np.isfinite(states)))
        self.assertTrue(np.all(np.isfinite(outputs)))
        self.assertGreater(states[-1, 0], 0.0)
        self.assertGreater(states[-1, 1], 0.0)
        self.assertGreater(states[-1, 2], 0.0)


if __name__ == "__main__":
    unittest.main()
