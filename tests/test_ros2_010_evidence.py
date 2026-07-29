"""Deterministic ROS2-010 evidence regeneration tests."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(
    0, str(ROOT / "ros2_ws" / "src" / "robot_supervisor")
)

from robot_supervisor.generate_evidence import (  # noqa: E402
    build_evidence,
    write_csv,
)


class TestRos2010Evidence(unittest.TestCase):
    def setUp(self) -> None:
        self.sim = (
            ROOT / "models" / "parameters" / "synthetic_sim_010.json"
        )
        self.motor = (
            ROOT / "models" / "parameters" / "synthetic_motor.json"
        )

    def test_ros2_010_evidence_passes_and_replays_exactly(self) -> None:
        report, traces = build_evidence(self.sim, self.motor)
        self.assertEqual(report["result"], "PASS")
        self.assertTrue(report["checks"]["exact_replay_matches"])
        self.assertGreater(len(traces), 0)

    def test_ros2_010_committed_evidence_matches_regeneration(self) -> None:
        report, traces = build_evidence(self.sim, self.motor)
        committed_report = json.loads(
            (
                ROOT
                / "data"
                / "processed"
                / "ros2_010_synthetic_validation_report.json"
            ).read_text(encoding="utf-8")
        )
        self.assertEqual(committed_report, report)
        with tempfile.TemporaryDirectory() as directory:
            regenerated = Path(directory) / "trace.csv"
            write_csv(regenerated, traces)
            committed = (
                ROOT
                / "data"
                / "processed"
                / "ros2_010_synthetic_message_trace.csv"
            )
            self.assertEqual(
                committed.read_bytes(), regenerated.read_bytes()
            )


if __name__ == "__main__":
    unittest.main()
