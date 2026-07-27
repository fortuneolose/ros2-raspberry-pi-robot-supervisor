# Synthetic floating-point controller and observer baseline

## Evidence status

`MODEL-010-SYNTHETIC` is an implemented and automatically checked software
development baseline. It demonstrates the control-design and evidence
pipeline, but it is not a hardware-ready controller and does not close final
MODEL-010 acceptance.

The motor parameters, controller poles, observer poles, reference, voltage
limit, load pulse, and pass thresholds are synthetic fixtures. They must be
replaced or justified after motor selection and plant identification.

## Control law

The sampled plant uses:

```text
x[k+1] = A x[k] + B_v v[k] + B_load tau_load[k]
y[k]   = C x[k]
```

The controller applies estimated-state feedback and a nominal reference
precompensator:

```text
v_requested[k] = -K x_hat[k] + N r[k]
v_applied[k]   = clip(v_requested[k], -6 V, +6 V)
```

For the synthetic fixture:

```text
K = [554.4338217, 55.4334319, 9.9184735]
N = 554.4338217
```

`N` produces unit steady-state position gain only for the exact nominal model.
It is not integral action and therefore does not guarantee zero error under a
constant unknown load or plant mismatch.

## Controller poles

Continuous-domain development targets `[-8, -10, -14] rad/s` are mapped with
`z = exp(s T_s)` at `T_s = 0.001 s` and placed in the discrete model.

| Pole | Requested z-plane value | Achieved value |
|---|---:|---:|
| 1 | 0.9860975443 | 0.9860975443 |
| 2 | 0.9900498337 | 0.9900498337 |
| 3 | 0.9920319148 | 0.9920319148 |

The maximum placement error in the generated report is 6.07e-14. All achieved
poles are inside the unit circle for this nominal fixture.

## Observer

The discrete Luenberger observer is:

```text
x_hat[k+1] =
    A x_hat[k] + B_v v_applied[k] + L(y[k] - C x_hat[k])
```

Load torque is deliberately not supplied to the observer, so the load-pulse
test exercises rejection of an unmeasured disturbance. The provisional gain
is:

```text
L = [0.04253861, 0.44977868, 4.06269791]^T
```

Continuous-domain observer targets `[-15, -18, -22] rad/s` map to achieved
discrete poles `[0.9851119396, 0.9821610324, 0.9782402351]`.

Observer convergence is evaluated componentwise to preserve units:

| State error | Development limit |
|---|---:|
| Position | 1e-5 rad |
| Speed | 1e-4 rad/s |
| Current | 1e-4 A |

All component errors remain within their limits after 0.741 s for the
configured initial mismatch. The normalized maximum error peaks at
approximately 7113 times the component limits before decaying. In physical
units, the dominant transient is current-estimate peaking. This is not hidden
by the pass result: later work must assess measurement noise, numeric range,
saturation, and whether slower observer poles provide a better trade-off.

## Nominal development result

The nominal scenario commands a 0.01 rad position step for 2 s with a ±6 V
limit.

| Metric | Result | Development limit |
|---|---:|---:|
| 10–90% rise time | 0.424 s | ≤ 0.600 s |
| 2% settling time | 0.759 s | ≤ 1.000 s |
| Overshoot | 0% | ≤ 5% |
| Steady-state error | 3.16e-8 rad | ≤ 1e-4 rad |
| Peak applied voltage | 5.544 V | ≤ 6 V |
| Saturated samples | 0 | 0 nominally |

These limits were chosen to make the synthetic workflow testable. They are not
final control-performance requirements.

## Load-pulse result

An opposing 0.001 N m torque is applied from 1.0 s to 1.2 s:

- peak absolute tracking error after pulse application: 0.001088 rad;
- return to and continued residence in the 2% band: 0.561 s after pulse end;
- final recorded tracking error: -6.28e-5 rad.

The transient recovers after the pulse is removed. Rejection of a persistent
load requires integral action, disturbance estimation, or another justified
architecture and remains future work.

## Reproduce

From the repository root:

```bash
python -m unittest discover -s tests -p "test_*.py" -v
MPLCONFIGDIR=.matplotlib python -m models.validate_controller
```

Generated evidence:

- `data/processed/model_010_synthetic_report.json`
- `data/processed/model_010_synthetic_response.csv`
- `docs/media/model_010_synthetic_closed_loop.png`
- `docs/media/model_010_synthetic_observer.png`

## What this milestone does not prove

- physical motor parameters or model validity;
- final controller or observer gains;
- robustness to measurement noise, quantisation, delay, parameter uncertainty,
  backlash, friction, dead zone, PWM ripple, thermal change, or supply limits;
- integral rejection of a constant load;
- fixed-point stability or numeric range;
- Raspberry Pi scheduling or ROS 2 execution;
- FPGA RTL equivalence; or
- safe physical operation.

`MODEL-020-SYNTHETIC` now addresses a bounded, explicitly synthetic subset of
the robustness items above. It does not retroactively make MODEL-010 a
physical acceptance result. The next gate is to review the floating-point
coefficient provenance and numeric range budget before building a bit-accurate
fixed-point reference model.
