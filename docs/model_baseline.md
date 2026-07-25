# Preliminary floating-point plant-model baseline

## Evidence status

This document records an implemented and automatically checked **synthetic
software baseline**. It is not physical plant-identification evidence and does
not complete the controller, observer, fixed-point, Raspberry Pi, FPGA, HIL, or
safety milestones.

## Purpose

The baseline makes the first non-hardware project stage executable while the
final motor, encoder, driver, supply, and load remain unselected. It provides a
stable interface for later plant identification, controller/observer design,
fixed-point comparison, and Raspberry Pi benchmarking.

## Continuous-time model

The state, input and measured-output vectors are:

```text
x = [theta, omega, i]^T
u = [v, tau_load]^T
y = [theta]
```

where:

- `theta` is shaft position in radians;
- `omega` is shaft speed in radians per second;
- `i` is armature current in amperes;
- `v` is armature voltage in volts; and
- `tau_load` is opposing load torque in newton metres.

The governing equations are:

```text
d(theta)/dt = omega
J d(omega)/dt = K_t i - b omega - tau_load
L d(i)/dt = v - R i - K_e omega
```

The software constructs the corresponding `A`, `B`, `C`, and `D` matrices from
named parameters and discretises them with a zero-order hold.

## Model assumptions and omissions

The first model is linear and time invariant. It assumes a rigid shaft, an
ideal applied armature voltage, linear viscous friction, constant electrical
and mechanical parameters, and an ideal encoder-position measurement. It does
not yet represent saturation, PWM ripple, dead zone, stiction, gearbox
backlash, encoder quantisation, sensor noise, supply limits, load variation, or
temperature dependence. These effects must be added or bounded when the
physical plant and intended operating range are known.

## Development parameter set

`SYNTHETIC-DCM-001` deliberately uses illustrative values:

| Parameter | Value | Unit |
|---|---:|---|
| Armature resistance, `R` | 1.0 | ohm |
| Armature inductance, `L` | 0.5 | H |
| Torque constant, `K_t` | 0.01 | N m/A |
| Back-EMF constant, `K_e` | 0.01 | V/(rad/s) |
| Referred inertia, `J` | 0.01 | kg m² |
| Viscous friction, `b` | 0.1 | N m s/rad |
| Sample period | 0.001 | s |

These values must not be quoted as properties of the final motor. The 1 ms
sample period is the preliminary requirement value, not a final bandwidth or
timing decision.

## Automated development checks

The `MODEL-001-SYNTHETIC` test set checks:

1. parameter validation and matrix dimensions;
2. documented state, input, output and unit definitions;
3. continuous voltage-input controllability;
4. continuous encoder-position observability;
5. preservation of the structural ranks after 1 ms ZOH discretisation;
6. the zero-input equilibrium; and
7. finite, directionally consistent response to a 1 V development step.

Passing these checks proves that the parameterized model workflow is
executable for the synthetic fixture. Final `MODEL-001` acceptance remains
open until the motor is selected, its parameters are documented or identified,
and the model is reviewed against physical data.

## Reproducible evidence

Run:

```bash
python -m unittest discover -s tests -p "test_*.py" -v
MPLCONFIGDIR=.matplotlib python -m models.validate_model
```

The validation command generates a machine-readable report, step-response CSV,
and plot. The report also records the Python and scientific-library versions.
Each generated artifact repeats the synthetic-data limitation.

## Next non-hardware gate

The next stage is to add state-feedback and observer design around this stable
model interface. Final gain acceptance, performance thresholds, and
fixed-point conversion remain blocked on a reviewed physical parameter set.
