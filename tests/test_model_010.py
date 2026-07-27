"""MODEL-010 development tests for synthetic floating-point control."""

from __future__ import annotations

import json
import unittest
from pathlib import Path

import numpy as np

from models.control import (
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

ROOT = Path(__file__).resolve().parents[1]
MOTOR_FILE = ROOT / "models" / "parameters" / "synthetic_motor.json"
CONTROLLER_FILE = (
    ROOT / "models" / "parameters" / "synthetic_controller.json"
)


class TestModel010SyntheticControl(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        motor_payload = json.loads(MOTOR_FILE.read_text(encoding="utf-8"))
        cls.controller_payload = json.loads(
            CONTROLLER_FILE.read_text(encoding="utf-8")
        )
        cls.sample_period_s = float(motor_payload["sample_period_s"])
        cls.model = discretize_zero_order_hold(
            continuous_dc_motor_model(
                MotorParameters.from_mapping(motor_payload["parameters"])
            ),
            cls.sample_period_s,
        )
        cls.requested_controller_poles = continuous_poles_to_discrete(
            cls.controller_payload["controller"][
                "continuous_poles_rad_s"
            ],
            cls.sample_period_s,
        )
        cls.requested_observer_poles = continuous_poles_to_discrete(
            cls.controller_payload["observer"][
                "continuous_poles_rad_s"
            ],
            cls.sample_period_s,
        )
        cls.design = design_observer_controller(
            cls.model,
            cls.requested_controller_poles,
            cls.requested_observer_poles,
        )

    def test_rejects_unstable_pole_target(self) -> None:
        with self.assertRaises(ValueError):
            design_observer_controller(
                self.model,
                [1.0, 0.9, 0.8],
                self.requested_observer_poles,
            )

    def test_controller_poles_match_requested_values(self) -> None:
        np.testing.assert_allclose(
            np.sort_complex(self.design.achieved_controller_poles),
            np.sort_complex(self.requested_controller_poles),
            atol=1e-9,
            rtol=0.0,
        )
        self.assertLess(
            np.max(np.abs(self.design.achieved_controller_poles)),
            1.0,
        )

    def test_observer_poles_match_requested_values(self) -> None:
        np.testing.assert_allclose(
            np.sort_complex(self.design.achieved_observer_poles),
            np.sort_complex(self.requested_observer_poles),
            atol=1e-9,
            rtol=0.0,
        )
        self.assertLess(
            np.max(np.abs(self.design.achieved_observer_poles)),
            1.0,
        )

    def test_reference_precompensator_has_unit_nominal_gain(self) -> None:
        voltage_input = self.model.b[:, [0]]
        steady_state = np.linalg.solve(
            np.eye(self.model.a.shape[0])
            - self.model.a
            + voltage_input @ self.design.state_feedback_gain,
            voltage_input * self.design.reference_gain,
        )
        self.assertAlmostEqual(
            float((self.model.c @ steady_state).item()),
            1.0,
            places=9,
        )

    def test_nominal_step_meets_development_limits(self) -> None:
        simulation = self.controller_payload["simulation"]
        limits = self.controller_payload["development_limits"]
        sample_count = int(
            round(
                float(simulation["duration_s"])
                / self.sample_period_s
            )
        )
        reference = float(simulation["reference_position_rad"])
        result = simulate_observer_feedback(
            self.model,
            self.design,
            np.full(sample_count, reference),
            np.zeros(sample_count),
            float(simulation["voltage_limit_v"]),
        )
        metrics = step_response_metrics(
            result.times_s[:-1],
            result.true_states[:-1, 0],
            reference,
        )
        self.assertLessEqual(
            metrics["rise_time_10_90_s"],
            limits["rise_time_10_90_s_max"],
        )
        self.assertLessEqual(
            metrics["settling_time_2_percent_s"],
            limits["settling_time_2_percent_s_max"],
        )
        self.assertLessEqual(
            metrics["overshoot_percent"],
            limits["overshoot_percent_max"],
        )
        self.assertLessEqual(
            metrics["steady_state_error_rad"],
            limits["steady_state_error_rad_max"],
        )
        self.assertFalse(np.any(result.saturated))

    def test_observer_converges_from_known_initial_error(self) -> None:
        simulation = self.controller_payload["simulation"]
        limits = self.controller_payload["development_limits"]
        sample_count = int(
            round(
                float(simulation["observer_duration_s"])
                / self.sample_period_s
            )
        )
        result = simulate_state_observer(
            self.model,
            self.design.observer_gain,
            np.zeros((sample_count, self.model.b.shape[1])),
            simulation["observer_initial_state"],
            simulation["observer_initial_estimate"],
        )
        component_limits = np.array(
            [
                limits["observer_component_limits"]["position_error_rad"],
                limits["observer_component_limits"]["speed_error_rad_s"],
                limits["observer_component_limits"]["current_error_a"],
            ]
        )
        normalized_error = np.max(
            np.abs(result.true_states - result.estimated_states)
            / component_limits,
            axis=1,
        )
        convergence_time = suffix_entry_time(
            result.times_s,
            normalized_error,
            1.0,
        )
        self.assertLessEqual(
            convergence_time,
            limits["observer_convergence_time_s_max"],
        )
        self.assertLessEqual(
            normalized_error[-1],
            1.0,
        )

    def test_load_pulse_recovers_within_development_limit(self) -> None:
        simulation = self.controller_payload["simulation"]
        limits = self.controller_payload["development_limits"]
        sample_count = int(
            round(
                float(simulation["duration_s"])
                / self.sample_period_s
            )
        )
        reference = float(simulation["reference_position_rad"])
        load = np.zeros(sample_count)
        start_index = int(
            round(
                float(simulation["load_pulse_start_s"])
                / self.sample_period_s
            )
        )
        end_index = int(
            round(
                float(simulation["load_pulse_end_s"])
                / self.sample_period_s
            )
        )
        load[start_index:end_index] = float(simulation["load_pulse_nm"])
        result = simulate_observer_feedback(
            self.model,
            self.design,
            np.full(sample_count, reference),
            load,
            float(simulation["voltage_limit_v"]),
        )
        tracking_error = reference - result.true_states[:-1, 0]
        recovery_time = suffix_entry_time(
            result.times_s[:-1],
            tracking_error,
            0.02 * reference,
            start_index=end_index,
        )
        self.assertGreater(
            np.max(np.abs(tracking_error[start_index:])),
            0.02 * reference,
        )
        self.assertLessEqual(
            recovery_time,
            limits["disturbance_recovery_time_s_max"],
        )

    def test_large_reference_is_safely_saturated(self) -> None:
        voltage_limit = float(
            self.controller_payload["simulation"]["voltage_limit_v"]
        )
        result = simulate_observer_feedback(
            self.model,
            self.design,
            np.full(100, 0.1),
            np.zeros(100),
            voltage_limit,
        )
        self.assertTrue(np.any(result.saturated))
        self.assertLessEqual(
            np.max(np.abs(result.applied_voltages_v)),
            voltage_limit,
        )


if __name__ == "__main__":
    unittest.main()
