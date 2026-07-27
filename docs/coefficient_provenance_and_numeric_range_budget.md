# Coefficient provenance and numeric range budget

## Gate decision

The `MODEL-020-PREFLIGHT-SYNTHETIC` audit executes successfully, but the
engineering decision is:

| Decision | Status |
|---|---|
| Derivation chain reproducible | **PASS** |
| Floating-point synthetic range observations complete | **PASS** |
| Coefficient set ready to freeze | **HOLD** |
| Fixed-point conversion ready to begin | **HOLD** |
| PR #4 disposition | **Keep draft pending human review** |

This distinction is intentional. Reproducing coefficients from versioned code
does not prove that their source assumptions represent the eventual motor,
encoder, driver, timing, or operating envelope.

No word length, binary point, rounding mode, saturation rule, or overflow
policy is selected by this audit.

## Provenance review

Repository history and source files establish the following chain:

| Item | Repository origin | How it was produced | Review finding |
|---|---|---|---|
| Continuous motor parameters | `models/parameters/synthetic_motor.json`, introduced in `12be797` | Six illustrative values entered as `SYNTHETIC-DCM-001` | Traceable to a file, but no datasheet, measurement, identification dataset, or uncertainty source exists |
| Sample period | Same file, 1 ms | Preliminary 1 kHz design target | Not yet justified by identified bandwidth or measured platform timing |
| Controller pole targets | `models/parameters/synthetic_controller.json`, introduced in `864e878` | `[-8, -10, -14] rad/s` development targets | No approved performance or actuator-limit derivation |
| Observer pole targets | Same file | `[-15, -18, -22] rad/s` development targets | No encoder-noise, estimator-peaking, or numeric-range justification |
| Discrete `A`, `B`, `C` | `models/dc_motor.py` | Zero-order-hold discretisation at 1 ms | Deterministic derivation from synthetic plant inputs |
| State-feedback gain `K` | `models/control.py` | `scipy.signal.place_poles` on the nominal discrete model | Reproduces exactly, but remains derived from synthetic inputs and targets |
| Reference gain `N` | `models/control.py` | Nominal steady-state unit-gain solve | Reproduces exactly; valid only for the exact nominal architecture |
| Observer gain `L` | `models/control.py` | Dual discrete pole placement | Reproduces exactly, but remains derived from synthetic inputs and targets |
| Uncertainty cases | `models/parameters/synthetic_robustness.json`, introduced in `07eca8c` | Illustrative ±20%, sensor, delay, and voltage cases | Not a measured tolerance distribution or worst-case hardware envelope |

The current generated controller coefficients are:

| Coefficient | Floating-point value | Derivation status |
|---|---:|---|
| `K_position` | 554.4338217446 | Reproducible, synthetic |
| `K_speed` | 55.4334319258 | Reproducible, synthetic |
| `K_current` | 9.9184734770 | Reproducible, synthetic |
| `N_reference` | 554.4338217446 | Reproducible, synthetic |
| `L_position` | 0.0425386055 | Reproducible, synthetic |
| `L_speed` | 0.4497786789 | Reproducible, synthetic |
| `L_current` | 4.0626979058 | Reproducible, synthetic |

These values remain useful regression fixtures. They are not approved
hardware coefficients.

## Range-budget method

The audit collects floating-point extrema from 24 deterministic cases:

- MODEL-010 nominal step;
- MODEL-010 load pulse;
- MODEL-010 observer-initialisation case;
- the existing 0.1 rad synthetic saturation probe; and
- all 20 MODEL-020 robustness scenarios.

It records plant states, observer states, measurements, innovation, references,
disturbances, requested/limited/applied voltages, individual feedback products,
observer prediction/input/correction terms, and order-independent sums of
absolute products for controller and observer accumulators.

Most provisional bounds equal:

```text
2.0 × largest observed floating-point magnitude
```

The factor of two is an explicit synthetic guard, not a statistical confidence
interval or physical safety factor. Commanded and applied voltage instead use
the configured hard bound of ±6 V.

## Key observed ranges

| Signal or intermediate | Largest observed magnitude | Provisional absolute bound | Minimum integer bits including sign* | Peak source |
|---|---:|---:|---:|---|
| True position | 0.0288515 rad | 0.0577030 rad | 1 | Observer initialisation |
| Estimated position | 0.0288515 rad | 0.0577030 rad | 1 | Observer initialisation |
| True speed | 0.100000 rad/s | 0.200000 rad/s | 1 | Observer initialisation |
| Estimated speed | 0.101916 rad/s | 0.203831 rad/s | 1 | Observer initialisation |
| True/estimated current | 1.08759 A | 2.17517 A | 3 | Saturation probe |
| Innovation | 0.0100000 rad | 0.0200000 rad | 1 | Observer initialisation |
| Requested voltage | 55.4434 V | 110.887 V | 8 | Saturation probe |
| Commanded/applied voltage | 6.00000 V | 6.00000 V | 4 | Configured limiter |
| Position-feedback product | 6.95294 V | 13.9059 V | 5 | 2048-count encoder case |
| Speed-feedback product | 2.24275 V | 4.48551 V | 4 | Saturation probe |
| Current-feedback product | 10.6897 V | 21.3793 V | 6 | Saturation probe |
| Feedback accumulator | 13.7447 V | 27.4894 V | 6 | Saturation probe |
| Reference product | 55.4434 V | 110.887 V | 8 | Saturation probe |
| Controller sum of absolute products | 69.1881 V | 138.376 V | 9 | Saturation probe |
| Observer current sum of absolute products | 1.08759 A | 2.17518 A | 3 | Saturation probe |

\*Integer-bit counts assume unity engineering-unit scaling and include the sign
bit. They are range observations, not selected word lengths. Fractional bits
remain `TBD_PENDING_QUANTIZATION_ERROR_STUDY`.

The complete 31-signal budget is in
`data/processed/model_020_synthetic_numeric_range_budget.csv`.

## Review of the 18-bit coefficient hypothesis

The numerical architecture previously proposed signed 18-bit coefficients as
a starting hypothesis. The current implementation coefficients have:

- largest absolute value: `554.4338217446`;
- smallest nonzero absolute value: `3.3233539621e-10`;
- magnitude ratio: `1.668296028872e12`; and
- binary span: approximately `40.60 bits`.

With one global 18-bit binary point, the largest coefficient requires 11
integer bits including sign. Only 7 fractional bits remain, giving an LSB of
`0.0078125`. The following coefficients fall below half that LSB:

- `A_position_speed`;
- `A_position_current`;
- `Bv_position`;
- `A_speed_current`;
- `Bv_speed`;
- `A_current_speed`; and
- `Bv_current`.

Therefore:

> A single shared 18-bit coefficient binary point is not supported by the
> current model.

This does not prove that every coefficient needs more than 18 bits. It shows
that fixed-point work must first evaluate state normalization and block-, row-,
or coefficient-specific scaling. Some very small terms may prove negligible,
but removing them requires an error and closed-loop stability study rather
than silent quantisation to zero.

## Remaining blockers

The coefficient set and fixed-point conversion remain on hold until:

1. motor parameters have datasheet or identification provenance and defensible
   uncertainty bounds;
2. valid reference, speed, current, disturbance, and operating ranges are
   approved;
3. encoder resolution and control-path timing are selected or measured;
4. controller/observer arithmetic order is frozen;
5. coefficient/state quantisation error and closed-loop stability are tested;
   and
6. rounding, saturation, overflow, and fault-reporting policies are frozen.

The first three items depend on hardware selection or measurement. The last
three are software/digital-design decisions that can begin only after the
floating-point inputs and valid ranges are reviewed.

## Reproduce

From the repository root:

```bash
python -m unittest discover -s tests -p "test_*.py" -v
MPLCONFIGDIR=.matplotlib python -m models.validate_range_budget
```

Evidence:

- `data/processed/model_020_synthetic_preflight_report.json`
- `data/processed/model_020_synthetic_numeric_range_budget.csv`
- `data/processed/model_020_synthetic_coefficient_provenance.csv`
- `docs/media/model_020_synthetic_range_budget.png`
- `tests/test_model_020_preflight.py`
