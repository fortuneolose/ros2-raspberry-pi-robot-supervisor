# Floating-point plant and control models

This directory contains the executable, hardware-independent plant and control
models for the single-axis robot-supervisor project.

## Current evidence level

`SYNTHETIC-DCM-001` verifies the model equations, units, zero-order-hold
discretisation, simulation path, and controllability/observability checks.
`MODEL-010-SYNTHETIC` adds state feedback, reference precompensation, a
Luenberger observer, voltage limiting, nominal position tracking, and a load
pulse. `MODEL-020-SYNTHETIC` adds deterministic parameter sweeps and sensor,
timing, and actuator nonidealities before fixed-point conversion.
`SIM-010-SYNTHETIC` adds a sample-indexed supervisor and simulated motor,
encoder, H-bridge, relay, E-stop, watchdog, supply, telemetry, and fault
interfaces. `ROS2-010-SYNTHETIC` imports that supervisor as the authoritative
ROS-independent state machine and adapts typed ROS 2 Jazzy messages around it;
it does not alter or duplicate `models/sil.py`. All are software fixtures.
None is an identified model, accepted controller, physical validation of the
eventual motor, Raspberry Pi, FPGA, or plant.

## Model

The state vector is:

```text
x = [shaft position (rad), shaft speed (rad/s), armature current (A)]
```

The inputs are armature voltage in volts and opposing load torque in newton
metres. Encoder position in radians is the measured output. The development
sample period is 1 ms, matching the preliminary 1 kHz requirement; it remains
subject to plant identification and timing analysis.

## Reproduce

From the repository root:

```bash
python -m unittest discover -s tests -p "test_*.py" -v
MPLCONFIGDIR=.matplotlib python -m models.validate_model
MPLCONFIGDIR=.matplotlib python -m models.validate_controller
MPLCONFIGDIR=.matplotlib python -m models.validate_robustness
MPLCONFIGDIR=.matplotlib python -m models.validate_range_budget
python -m models.validate_sim
```

Generated development evidence is written to:

- `data/processed/model_001_synthetic_report.json`
- `data/processed/model_001_synthetic_step.csv`
- `docs/media/model_001_synthetic_step.png`
- `data/processed/model_010_synthetic_report.json`
- `data/processed/model_010_synthetic_response.csv`
- `docs/media/model_010_synthetic_closed_loop.png`
- `docs/media/model_010_synthetic_observer.png`
- `data/processed/model_020_synthetic_robustness_report.json`
- `data/processed/model_020_synthetic_robustness_results.csv`
- `docs/media/model_020_parameter_sweep.png`
- `docs/media/model_020_nonideality_summary.png`
- `data/processed/model_020_synthetic_preflight_report.json`
- `data/processed/model_020_synthetic_numeric_range_budget.csv`
- `data/processed/model_020_synthetic_coefficient_provenance.csv`
- `docs/media/model_020_synthetic_range_budget.png`
- `data/processed/sim_010_synthetic_validation_report.json`
- `data/processed/sim_010_synthetic_scenario_trace.csv`
- `data/processed/ros2_010_synthetic_validation_report.json`
- `data/processed/ros2_010_synthetic_message_trace.csv`

Replace `models/parameters/synthetic_motor.json` only with a reviewed,
source-backed or experimentally identified parameter set. Preserve the
synthetic fixture for regression tests. Do not transfer the provisional gains
in `models/parameters/synthetic_controller.json` to hardware without redesign
against the identified plant, reviewed limits, and safety envelope.
Likewise, the ranges in `models/parameters/synthetic_robustness.json` are not
measured tolerances. Freeze a source-backed parameter set and numeric range
budget before beginning fixed-point conversion. The current preflight audit
leaves coefficient freeze and fixed-point readiness on hold and assigns no
fractional bits or binary points.

SIM-010 configuration is in
`models/parameters/synthetic_sim_010.json`. Every plant value, operating value,
limit, and threshold in that configuration is synthetic. The bench does not
use ROS 2, GPIO, wall-clock timing, fixed-point formats, or RTL. The synthetic
supervisor requires an encoder sequence transition before READY and a
disarmed READY sample before a later arm/run request. Finite commands beyond
the synthetic absolute limit are clipped with saturation telemetry and do not
fault; malformed or non-finite commands latch a safe-output fault.

ROS2-010 keeps this module authoritative. Its adapter converts accepted
monotonic ROS publisher sequences into `SupervisorInputs`; required telemetry
loss becomes the existing stale/watchdog inputs, and every output message is
derived from the resulting `TelemetryRecord`. The ROS node's 20 ms default
timer is a synthetic middleware fixture and is not a replacement for the
model's 1 ms sample metadata or a physical timing claim. See
`docs/ros2_010_middleware_integration.md`.
