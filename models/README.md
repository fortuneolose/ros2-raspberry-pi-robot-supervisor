# Floating-point plant model

This directory contains the first executable, hardware-independent model for
the single-axis robot-supervisor project.

## Current evidence level

`SYNTHETIC-DCM-001` is a software fixture. It verifies the model equations,
units, zero-order-hold discretisation, simulation path, and automated
controllability/observability checks. It is **not** an identified model of the
eventual motor and does not validate a controller, observer, Raspberry Pi,
FPGA, or physical safety function.

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
```

Generated development evidence is written to:

- `data/processed/model_001_synthetic_report.json`
- `data/processed/model_001_synthetic_step.csv`
- `docs/media/model_001_synthetic_step.png`

Replace `models/parameters/synthetic_motor.json` only with a reviewed,
source-backed or experimentally identified parameter set. Preserve the
synthetic fixture for regression tests.
