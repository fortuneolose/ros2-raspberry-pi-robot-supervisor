# Synthetic robustness and uncertainty baseline

## Evidence status

`MODEL-020-SYNTHETIC` is an implemented and automatically checked
floating-point development baseline. It tests whether the existing nominal
MODEL-010 controller/observer software behaves deterministically and remains
numerically bounded under an explicitly synthetic uncertainty set.

It is not physical robustness acceptance. None of the ±20% plant variations,
encoder resolutions, noise levels, delays, voltage limits, or integrity bounds
were measured from selected hardware.

## Analysis method

The MODEL-010 gains remain fixed. Each scenario replaces only the simulated
plant or implementation condition:

```text
nominal K, L, N
        │
        ├── varied floating-point plant
        ├── noisy or quantised position measurement
        ├── unmodelled command delay
        └── reduced voltage availability
```

For plant mismatch, the observer continues to use the nominal `A`, `B`, and
`C` matrices while the true state evolves with the varied plant. A one- or
two-sample delay postpones the voltage seen by the plant while the observer
uses the current limited command. This deliberately represents an unmodelled
actuation or transport delay.

Seed `20260727` makes every measurement-noise sequence exactly repeatable.
The generated validator runs each scenario twice and compares its arrays and
metrics exactly.

## Scenario matrix

Twenty scenarios are executed:

| Category | Count | Coverage |
|---|---:|---|
| Baseline | 1 | Exact regression to the nominal MODEL-010 simulator |
| Plant parameter | 12 | One-at-a-time 0.8× and 1.2× changes to `R`, `L`, `Kt`, `Ke`, `J`, and `b` |
| Sensor | 3 | 2048-count encoder, 20 µrad seeded noise, and combined 4096-count/noise case |
| Timing | 2 | One- and two-sample unmodelled command delays |
| Actuator | 1 | Supply/command limit reduced from 6.0 V to 4.5 V |
| Combined | 1 | 1.2× inertia, 0.8× torque constant, sensor limitations, one-sample delay, and 4.5 V limit |

The single-parameter sweep is deliberately simple and interpretable. It is not
a statistical Monte Carlo analysis and does not claim that the parameters are
independent or uniformly distributed.

## Checked invariants

The validator passes only when all of the following are true:

- all configured scenario identifiers are unique and every case executes;
- seeded results reproduce exactly;
- the nominal case matches the MODEL-010 state, estimate, and voltage arrays;
- all states, measurements, estimates, and voltage sequences remain finite;
- commanded and delayed applied voltages remain within each case's limit;
- each varied plant's zero-delay linearized augmented spectral radius remains
  below `0.999999`; and
- each time-domain response remains inside the declared synthetic numerical
  integrity envelope of 0.03 rad position, 0.25 rad/s speed, and 2.0 A current.

These are development integrity gates, not final performance criteria. Rise
time, tracking error, estimation error, and saturation are reported for
engineering interpretation without turning invented limits into requirements.

## Results

All 20 scenarios pass all 8 checks.

| Observation | Result |
|---|---:|
| Nominal tail mean absolute tracking error | 3.16e-8 rad |
| Largest one-at-a-time tail error | 3.38e-6 rad (`L` at 1.2×) |
| Largest RMS position-estimation error | 9.11e-4 rad (2048-count encoder) |
| Largest RMS tracking error | 3.431e-3 rad (combined stress case) |
| Largest zero-delay spectral radius | 0.995907 (combined stress case) |
| Combined-case tail mean absolute tracking error | 9.41e-5 rad |
| Combined-case saturated samples | 12 of 2000 |

The 2048-count/revolution fixture has a count angle of approximately
3.07 mrad, which is large relative to the 10 mrad position command. Its
estimation degradation is therefore expected and useful: encoder selection
and gearing must be based on the required motion resolution rather than on a
generic counts-per-revolution value.

The combined case has the smallest linearized stability margin and the largest
RMS tracking error. It still remains finite and bounded in this short,
synthetic run. That is a reason to investigate the condition after plant
identification, not evidence that the eventual hardware will tolerate it.

## Reproduce

From the repository root:

```bash
python -m unittest discover -s tests -p "test_*.py" -v
MPLCONFIGDIR=.matplotlib python -m models.validate_robustness
```

Generated evidence:

- `data/processed/model_020_synthetic_robustness_report.json`
- `data/processed/model_020_synthetic_robustness_results.csv`
- `docs/media/model_020_parameter_sweep.png`
- `docs/media/model_020_nonideality_summary.png`
- `tests/test_model_020.py`

## Decision before fixed-point conversion

MODEL-020 provides the software gate requested before fixed-point work. Before
coefficients or states are converted, the following inputs still require
review:

1. selected motor, gearing, encoder, driver, and supply;
2. identified parameter values with provenance and uncertainty estimates;
3. required motion range, resolution, and acceptable control performance;
4. realistic delay and sampling-jitter measurements; and
5. state, estimate, coefficient, accumulator, and control-signal range budgets.

Fixed-point conversion is intentionally not implemented in this milestone.
The subsequent
[coefficient-provenance and numeric-range audit](coefficient_provenance_and_numeric_range_budget.md)
now reproduces the derivation chain and records a provisional 31-signal budget.
Its execution passes, but coefficient freeze and fixed-point readiness remain
on hold because the physical sources and valid operating envelope are absent.
No word length, binary point, rounding, saturation, or overflow policy has
been selected.

## Not demonstrated

- physical parameter distributions, correlations, or tolerances;
- backlash, Coulomb friction, cogging, thermal drift, PWM ripple, or supply
  dynamics;
- persistent-load rejection;
- final controller/observer performance or stability margin;
- fixed-point arithmetic or quantized closed-loop stability;
- Raspberry Pi scheduling, ROS 2 execution, FPGA RTL, HIL, or safe physical
  operation.
