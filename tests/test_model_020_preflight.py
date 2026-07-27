"""Pre-fixed-point provenance and range-budget tests for MODEL-020."""

from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from models.range_budget import (
    ObservedRange,
    finalize_range_records,
    minimum_signed_integer_bits_including_sign,
)
from models.validate_range_budget import build_audit

ROOT = Path(__file__).resolve().parents[1]
MOTOR_FILE = ROOT / "models" / "parameters" / "synthetic_motor.json"
CONTROLLER_FILE = ROOT / "models" / "parameters" / "synthetic_controller.json"
ROBUSTNESS_FILE = ROOT / "models" / "parameters" / "synthetic_robustness.json"
RANGE_CONFIG_FILE = (
    ROOT / "models" / "parameters" / "synthetic_range_budget.json"
)


class TestRangeBudgetUtilities(unittest.TestCase):
    def test_minimum_integer_bits_handle_fractional_and_power_of_two_bounds(
        self,
    ) -> None:
        self.assertEqual(
            minimum_signed_integer_bits_including_sign(0.5),
            1,
        )
        self.assertEqual(
            minimum_signed_integer_bits_including_sign(1.0),
            2,
        )
        self.assertEqual(
            minimum_signed_integer_bits_including_sign(6.0),
            4,
        )
        self.assertEqual(
            minimum_signed_integer_bits_including_sign(128.0),
            9,
        )

    def test_minimum_integer_bits_reject_invalid_bounds(self) -> None:
        for value in (-1.0, np.inf, np.nan):
            with self.assertRaises(ValueError):
                minimum_signed_integer_bits_including_sign(value)

    def test_observed_range_rejects_empty_or_nonfinite_values(self) -> None:
        record = ObservedRange("signal", "group", "unit")
        with self.assertRaises(ValueError):
            record.update("empty", [])
        with self.assertRaises(ValueError):
            record.update("nonfinite", [1.0, np.inf])

    def test_configured_hard_bound_must_contain_observed_peak(self) -> None:
        record = ObservedRange("limited", "control_output", "V")
        record.update("case", [-6.0, 6.0])
        with self.assertRaises(ValueError):
            finalize_range_records(
                {"limited": record},
                guard_factor=2.0,
                configured_hard_bounds={"limited": 5.0},
            )


class TestModel020PreFixedPointAudit(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        (
            cls.report,
            cls.range_records,
            cls.coefficients,
        ) = build_audit(
            MOTOR_FILE,
            CONTROLLER_FILE,
            ROBUSTNESS_FILE,
            RANGE_CONFIG_FILE,
        )
        cls.range_lookup = {
            record["signal_id"]: record
            for record in cls.range_records
        }
        cls.coefficient_lookup = {
            record["coefficient_id"]: record
            for record in cls.coefficients
        }

    def test_audit_executes_but_readiness_remains_on_hold(self) -> None:
        self.assertEqual(self.report["result"], "PASS")
        self.assertTrue(all(self.report["checks"].values()))
        self.assertEqual(
            self.report["coefficient_freeze_readiness"],
            "HOLD",
        )
        self.assertEqual(
            self.report["fixed_point_conversion_readiness"],
            "HOLD",
        )
        self.assertEqual(
            self.report["pr4_recommendation"],
            "KEEP_DRAFT_PENDING_HUMAN_REVIEW",
        )

    def test_all_24_source_cases_and_required_signal_ranges_are_covered(
        self,
    ) -> None:
        self.assertEqual(self.report["case_count"], 24)
        self.assertEqual(len(self.range_records), 31)
        self.assertIn("controller_l1_sum_of_products_v", self.range_lookup)
        self.assertIn(
            "observer_current_l1_sum_of_products",
            self.range_lookup,
        )

    def test_every_budget_contains_observations_and_leaves_fractional_bits_tbd(
        self,
    ) -> None:
        for record in self.range_records:
            self.assertGreaterEqual(
                record["budget_abs_bound"] + 1e-12,
                record["observed_peak_abs"],
            )
            self.assertEqual(
                record["fractional_bits_status"],
                "TBD_PENDING_QUANTIZATION_ERROR_STUDY",
            )

    def test_voltage_outputs_use_configured_hard_bound(self) -> None:
        for signal_id in ("commanded_voltage_v", "applied_voltage_v"):
            record = self.range_lookup[signal_id]
            self.assertEqual(record["observed_peak_abs"], 6.0)
            self.assertEqual(record["budget_abs_bound"], 6.0)
            self.assertEqual(
                record["budget_basis"],
                "configured_hard_bound",
            )

    def test_derived_controller_coefficients_match_recorded_values(self) -> None:
        self.assertAlmostEqual(
            self.coefficient_lookup["K_position"]["value"],
            554.4338217446472,
        )
        self.assertAlmostEqual(
            self.coefficient_lookup["N_reference"]["value"],
            554.4338217446472,
        )
        self.assertAlmostEqual(
            self.coefficient_lookup["L_current"]["value"],
            4.0626979057837795,
        )

    def test_one_uniform_18_bit_coefficient_binary_point_is_rejected(
        self,
    ) -> None:
        assessment = self.report[
            "uniform_18_bit_coefficient_hypothesis"
        ]
        self.assertEqual(
            assessment["verdict"],
            "NOT_SUPPORTED_WITH_ONE_GLOBAL_BINARY_POINT",
        )
        self.assertGreater(
            self.report["coefficient_dynamic_range"]["span_bits_log2"],
            40.0,
        )
        self.assertIn(
            "Bv_position",
            assessment["coefficients_below_half_lsb"],
        )


if __name__ == "__main__":
    unittest.main()
