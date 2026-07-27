# Verification and Test Report

## 1. Baseline status

**Status: partially executed for synthetic software development.** The
MODEL-001 and MODEL-010 synthetic checks have executable evidence. They do not
constitute final plant or controller acceptance. ROS 2, RTL, timing, hardware,
safety, and HIL tests remain unexecuted.

## 2. Test configuration

Complete for each released test campaign:

| Field | Value |
|---|---|
| Repository commit | TBD |
| Test date / operator / reviewer | TBD |
| Raspberry Pi hardware and OS image | TBD |
| ROS 2 distribution and package versions | TBD |
| Basys 3 board revision and FPGA part | TBD |
| Vivado version / bitstream identifier | TBD |
| Motor / encoder / driver / load | TBD |
| Interface PCB and wiring revision | TBD |
| Supplies, fuse, limits, and E-stop | TBD |
| Test equipment and calibration status | TBD |
| Coefficient set / fixed-point format | TBD |
| Raw-data location | `data/raw/` |
| Processed results | `data/processed/` |

## 3. Verification stages

| Test ID | Test | Key pass condition | Status |
|---|---|---|---|
| DOC-001 | Requirements traceability audit | Every mandatory requirement has design and verification links | Not run |
| DOC-002 | Acceptance-criteria review | Final numerical thresholds are frozen before acceptance testing | Not run |
| MODEL-001 | Plant/controller structural checks | Model documented; controllability and observability checks pass | Development pass — synthetic fixture |
| MODEL-010 | Floating-point control tests | Frozen stability and performance limits pass | Development pass — synthetic fixture |
| RTL-001 | Sample-enable and reset | Exactly one update pulse per configured interval; safe reset | Not run |
| RTL-002 | Encoder synchronisation/decode | Valid directions/counts; invalid transitions detected | Not run |
| RTL-020 | Bit-accurate controller regression | Matches approved fixed-point reference tolerance | Not run |
| RTL-021 | Numerical boundaries | Rounding, saturation, sign, and overflow cases pass | Not run |
| UNIT-010 | Protocol codec/parser | Version, length, sequence, integrity, and malformed cases pass | Not run |
| INT-001 | UART/SPI loopback | Repeatable transactions at selected rates | Not run |
| INT-002 | Protocol fault injection | Corrupt/stale/repeated packets rejected and reported | Not run |
| INT-003 | Dedicated control signals | ARM and IRQ/FAULT directions and safe defaults verified | Not run |
| INT-010 | ROS 2 node and interface test | Required nodes, topics, parameters, commands, telemetry, and fault interfaces operate coherently | Not run |
| INT-020 | Integrated FPGA control path | Observer, controller, PWM, telemetry, and faults operate coherently | Not run |
| INT-021 | Atomic telemetry | Snapshot values are stable and sample counter is monotonic | Not run |
| SYN-001 | Vivado implementation | Required timing closes; constraints complete; usage recorded | Not run |
| HW-DR-001 | Electrical-interface design review | Voltage compatibility, pin assignments, protection, and safe defaults are approved | Not run |
| HW-DR-002 | Power and component-ratings review | Driver, supplies, fuse, conductors, and connectors have documented worst-case margin | Not run |
| SAFE-001 | Safe-state disable latency | PWM/enable reach safe state within frozen limit | Not run |
| SAFE-002 | E-stop/fuse/current limit | Physical protections present and functional | Not run |
| SAFE-003 | Arming interlock | No invalid input combination enables the actuator | Not run |
| SAFE-004 | Fault-injection suite | Each required fault produces the specified state/status | Not run |
| SAFE-005 | Reset and re-arm | No automatic restart; explicit sequence required | Not run |
| SAFE-010 | Raspberry Pi process-loss test | FPGA watchdog reaches the specified safe state without relying on software recovery | Not run |
| TIM-001 | FPGA update timing | No missed update; rate and jitter meet frozen limits | Not run |
| TIM-010 | Pi scheduling under load | Period, latency, jitter, and misses recorded | Not run |
| HIL-001 | Mechanical/pre-power inspection | Mount, guard, polarity, continuity, limits approved | Not run |
| HIL-002 | Wiring inspection | Power paths, grounding, keying, labels, strain relief, and test points match approved drawings | Not run |
| HIL-010 | Low-duty open-loop test | Direction, encoder, current, PWM, and stop controls correct | Not run |
| HIL-020 | Comparative closed-loop experiment | Pi-only and hybrid datasets complete under matched conditions | Not run |
| DATA-001 | Logging audit | Required fields and metadata present | Not run |
| DATA-002 | Data reproducibility audit | Raw data remains immutable and processed results reproduce from versioned scripts | Not run |
| REP-001 | Clean-checkout reproduction | Build, vectors, analysis, and report reproduce successfully | Not run |
| REPORT-001 | Final metric review | Required control, timing, numerical, implementation, robustness, and safety metrics are reported | Not run |

## 4. Comparative metrics

Final reports shall include:

- rise time, settling time, overshoot, steady-state and RMS tracking error;
- disturbance and load-change recovery;
- mean period, maximum jitter, latency distribution, and missed deadlines;
- maximum state/control error against the fixed-point model;
- pole movement after quantisation and saturation/overflow counts;
- FPGA DSP/LUT/FF/BRAM usage, worst negative slack, clock estimate, and power
  estimate;
- sensitivity to sensor noise, plant mismatch, communications loss, and supply
  or load variation; and
- emergency/fault disable latency and reset/re-arm behaviour.

## 5. Result record

For each test, add a subsection containing objective, mapped requirements,
preconditions, procedure, limits, observed results, raw evidence links,
deviations, pass/fail decision, operator, and reviewer. Failed or waived tests
must link to a tracked corrective action or an explicitly accepted limitation.

## 6. MODEL-010-SYNTHETIC development record

**Objective:** demonstrate an executable floating-point controller and observer
workflow before motor selection and plant identification.

**Mapped requirements:** CTL-003 and the development portions of CTL-004,
CTL-005, VER-001, and VER-004.

**Method:** map continuous-domain pole targets into the z plane at 1 ms, place
the discrete controller and observer poles, compute a nominal steady-state
reference gain, simulate observer-based position feedback with a ±6 V limit,
and inject a 1 mN m opposing load pulse.

**Result:** development pass. All 16 repository unit tests and all 13
machine-readable MODEL-010 checks passed in the recorded software environment.

Key results:

- 10–90% nominal rise time: 0.424 s;
- 2% nominal settling time: 0.759 s;
- nominal overshoot: 0%;
- nominal steady-state error: 3.16e-8 rad;
- peak nominal applied voltage: 5.544 V with no saturated samples;
- observer component limits continuously satisfied after 0.741 s;
- the normalized observer error initially peaks above its initial value,
  documenting observer peaking that requires later noise and range analysis;
- 1 mN m load-pulse recovery to the 2% band: 0.561 s.

Evidence:

- `data/processed/model_010_synthetic_report.json`
- `data/processed/model_010_synthetic_response.csv`
- `docs/media/model_010_synthetic_closed_loop.png`
- `docs/media/model_010_synthetic_observer.png`
- `tests/test_model_010.py`

**Acceptance limitation:** the plant parameters, pole targets, gains,
reference, voltage limit, disturbance, and thresholds are synthetic
development fixtures. Integral disturbance rejection, measurement noise,
parameter uncertainty, fixed-point arithmetic, scheduling, RTL, and physical
operation have not been demonstrated.
