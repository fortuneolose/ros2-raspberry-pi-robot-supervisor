"""MODEL-020 tests for deterministic synthetic robustness analysis."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

import numpy as np

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
    RobustnessScenario,
    quantize_encoder_position,
    scaled_motor_parameters,
    simulate_mismatched_observer_feedback,
    summarize_result,
    zero_delay_augmented_spectral_radius,
)

ROOT = Path(__file__).resolve().parents[1]
MOTOR_FILE = ROOT / "models" / "parameters" / "synthetic_motor.json"
CONTROLLER_FILE = ROOT / "models" / "parameters" / "synthetic_controller.json"
ROBUSTNESS_FILE = ROOT / "models" / "parameters" / "synthetic_robustness.json"


class TestModel020SyntheticRobustness(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.motor_payload = json.loads(MOTOR_FILE.read_text(encoding="utf-8"))
        cls.controller_payload = json.loads(
            CONTROLLER_FILE.read_text(encoding="utf-8")
        )
        cls.robustness_payload = json.loads(
            ROBUSTNESS_FILE.read_text(encoding="utf-8")
        )
        cls.nominal_parameters = MotorParameters.from_mapping(
            cls.motor_payload["parameters"]
        )
        cls.sample_period_s = float(cls.motor_payload["sample_period_s"])
        cls.nominal_model = discretize_zero_order_hold(
            continuous_dc_motor_model(cls.nominal_parameters),
            cls.sample_period_s,
        )
        controller_poles = continuous_poles_to_discrete(
            cls.controller_payload["controller"]["continuous_poles_rad_s"],
            cls.sample_period_s,
        )
        observer_poles = continuous_poles_to_discrete(
            cls.controller_payload["observer"]["continuous_poles_rad_s"],
            cls.sample_period_s,
        )
        cls.design = design_observer_controller(
            cls.nominal_model,
            controller_poles,
            observer_poles,
        )
        simulation = cls.robustness_payload["simulation"]
        cls.sample_count = int(
            round(float(simulation["duration_s"]) / cls.sample_period_s)
        )
        cls.reference_rad = float(simulation["reference_position_rad"])
        cls.references = np.full(cls.sample_count, cls.reference_rad)
        cls.loads = np.zeros(cls.sample_count)
        cls.seed = int(cls.robustness_payload["random_seed"])
        cls.nominal_scenario = RobustnessScenario(
            scenario_id="nominal",
            category="baseline",
            description="Nominal MODEL-010 regression case.",
            parameter_multipliers={},
            voltage_limit_v=float(simulation["nominal_voltage_limit_v"]),
        )

    def test_rejects_unknown_or_nonpositive_parameter_multiplier(self) -> None:
        with self.assertRaises(ValueError):
            scaled_motor_parameters(
                self.nominal_parameters,
                {"not_a_motor_parameter": 1.0},
            )
        with self.assertRaises(ValueError):
            scaled_motor_parameters(
                self.nominal_parameters,
                {"rotor_inertia_kg_m2": 0.0},
            )

    def test_rejects_invalid_encoder_and_delay_values(self) -> None:
        with self.assertRaises(ValueError):
            RobustnessScenario(
                scenario_id="bad-encoder",
                category="sensor",
                description="invalid fixture",
                parameter_multipliers={},
                encoder_counts_per_revolution=3,
            ).validate()
        with self.assertRaises(ValueError):
            RobustnessScenario(
                scenario_id="bad-delay",
                category="timing",
                description="invalid fixture",
                parameter_multipliers={},
                control_delay_samples=-1,
            ).validate()

    def test_rejects_negative_random_seed(self) -> None:
        with self.assertRaises(ValueError):
            simulate_mismatched_observer_feedback(
                self.nominal_model,
                self.nominal_model,
                self.design,
                self.references,
                self.loads,
                self.nominal_scenario,
                -1,
            )

    def test_encoder_quantisation_uses_declared_grid(self) -> None:
        counts = 2048
        quantized = quantize_encoder_position(0.01, counts)
        count_angle = 2.0 * np.pi / counts
        self.assertAlmostEqual(quantized / count_angle, round(0.01 / count_angle))
        self.assertLessEqual(abs(quantized - 0.01), count_angle / 2.0)

    def test_nominal_scenario_regresses_to_model_010_simulator(self) -> None:
        robust = simulate_mismatched_observer_feedback(
            self.nominal_model,
            self.nominal_model,
            self.design,
            self.references,
            self.loads,
            self.nominal_scenario,
            self.seed,
        )
        baseline = simulate_observer_feedback(
            self.nominal_model,
            self.design,
            self.references,
            self.loads,
            self.nominal_scenario.voltage_limit_v,
        )
        np.testing.assert_allclose(robust.true_states, baseline.true_states)
        np.testing.assert_allclose(
            robust.estimated_states,
            baseline.estimated_states,
        )
        np.testing.assert_allclose(
            robust.applied_voltages_v,
            baseline.applied_voltages_v,
        )

    def test_seeded_noise_is_exactly_repeatable(self) -> None:
        scenario = RobustnessScenario(
            scenario_id="seed-repeat",
            category="sensor",
            description="repeatability test",
            parameter_multipliers={},
            measurement_noise_std_rad=0.00002,
        )
        first = simulate_mismatched_observer_feedback(
            self.nominal_model,
            self.nominal_model,
            self.design,
            self.references,
            self.loads,
            scenario,
            self.seed,
        )
        second = simulate_mismatched_observer_feedback(
            self.nominal_model,
            self.nominal_model,
            self.design,
            self.references,
            self.loads,
            scenario,
            self.seed,
        )
        np.testing.assert_array_equal(
            first.measured_positions_rad,
            second.measured_positions_rad,
        )
        np.testing.assert_array_equal(first.true_states, second.true_states)

    def test_different_seed_changes_noisy_measurements(self) -> None:
        scenario = RobustnessScenario(
            scenario_id="seed-change",
            category="sensor",
            description="seed sensitivity test",
            parameter_multipliers={},
            measurement_noise_std_rad=0.00002,
        )
        first = simulate_mismatched_observer_feedback(
            self.nominal_model,
            self.nominal_model,
            self.design,
            self.references,
            self.loads,
            scenario,
            self.seed,
        )
        second = simulate_mismatched_observer_feedback(
            self.nominal_model,
            self.nominal_model,
            self.design,
            self.references,
            self.loads,
            scenario,
            self.seed + 1,
        )
        self.assertFalse(
            np.array_equal(
                first.measured_positions_rad,
                second.measured_positions_rad,
            )
        )

    def test_voltage_limit_is_preserved_with_command_delay(self) -> None:
        scenario = RobustnessScenario(
            scenario_id="delayed-limited",
            category="combined",
            description="voltage invariant test",
            parameter_multipliers={},
            control_delay_samples=2,
            voltage_limit_v=4.5,
        )
        result = simulate_mismatched_observer_feedback(
            self.nominal_model,
            self.nominal_model,
            self.design,
            self.references,
            self.loads,
            scenario,
            self.seed,
        )
        self.assertLessEqual(
            np.max(np.abs(result.commanded_voltages_v)),
            scenario.voltage_limit_v,
        )
        self.assertLessEqual(
            np.max(np.abs(result.applied_voltages_v)),
            scenario.voltage_limit_v,
        )

    def test_parameter_sweep_remains_finite_and_zero_delay_stable(self) -> None:
        envelope = self.robustness_payload["development_integrity_envelope"]
        for parameter_name in self.robustness_payload["parameter_sweep"][
            "parameters"
        ]:
            for scale in self.robustness_payload["parameter_sweep"][
                "scale_factors"
            ]:
                parameters = scaled_motor_parameters(
                    self.nominal_parameters,
                    {parameter_name: float(scale)},
                )
                plant = discretize_zero_order_hold(
                    continuous_dc_motor_model(parameters),
                    self.sample_period_s,
                )
                radius = zero_delay_augmented_spectral_radius(
                    plant,
                    self.nominal_model,
                    self.design,
                )
                result = simulate_mismatched_observer_feedback(
                    plant,
                    self.nominal_model,
                    self.design,
                    self.references,
                    self.loads,
                    RobustnessScenario(
                        scenario_id=f"{parameter_name}-{scale}",
                        category="plant_parameter",
                        description="test sweep point",
                        parameter_multipliers={parameter_name: float(scale)},
                    ),
                    self.seed,
                )
                metrics = summarize_result(result, self.reference_rad)
                self.assertTrue(metrics["all_values_finite"])
                self.assertLessEqual(
                    radius,
                    envelope["zero_delay_spectral_radius_max"],
                )


if __name__ == "__main__":
    unittest.main()
