# Floating-point plant and control models

This directory contains the executable, hardware-independent plant and control
models for the single-axis robot-supervisor project.

## Current evidence level

`SYNTHETIC-DCM-001` verifies the model equations, units, zero-order-hold
discretisation, simulation path, and controllability/observability checks.
`MODEL-010-SYNTHETIC` adds state feedback, reference precompensation, a
Luenberger observer, voltage limiting, nominal position tracking, and a load
pulse. `MODEL-020-SYNTHETIC` adds deterministic parameter sweeps and sensor,
timing, and actuator nonidealities before fixed-point conversion. All three
are software fixtures. None is an identified model or accepted controller for
the eventual motor, Raspberry Pi, FPGA, or physical plant.

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
