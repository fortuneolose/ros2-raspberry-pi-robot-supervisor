# Verification and Test Report

## 1. Baseline status

**Status: partially executed for synthetic software development.** The
MODEL-001, MODEL-010, MODEL-020, SIM-010, and ROS2-010 synthetic checks have
executable evidence. They do not constitute final plant, controller,
robustness, supervisor, middleware timing, or safety acceptance. The
MODEL-020 preflight audit, SIM-010, and ROS2-010 preserve coefficient freeze
and fixed-point readiness on hold. ROS2-010 executes a Jazzy runtime in a
local container; native Raspberry Pi ROS 2, GPIO, RTL, platform timing,
physical hardware, safety, and HIL tests remain unexecuted.

## 2. Test configuration

Complete for each released test campaign:

| Field | Value |
|---|---|
| Repository commit | TBD |
| Test date / operator / reviewer | TBD |
| Raspberry Pi hardware and OS image | TBD |
| ROS 2 distribution and package versions | Jazzy container validation; exact package versions recorded by the pinned image/apt transaction |
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
| MODEL-020 | Floating-point robustness, provenance, and range analysis | Synthetic scenarios are repeatable and bounded; derivation and ranges are traceable without falsely approving physical coefficients | Development pass — coefficient/fixed-point readiness hold |
| SIM-010 | Hardware-independent supervisor software-in-the-loop | All required scenarios and exact replay pass; every detected/latched fault record has relay disabled and zero motor command | Development pass — synthetic software only; coefficient/fixed-point readiness hold |
| RTL-001 | Sample-enable and reset | Exactly one update pulse per configured interval; safe reset | Not run |
| RTL-002 | Encoder synchronisation/decode | Valid directions/counts; invalid transitions detected | Not run |
| RTL-020 | Bit-accurate controller regression | Matches approved fixed-point reference tolerance | Not run |
| RTL-021 | Numerical boundaries | Rounding, saturation, sign, and overflow cases pass | Not run |
| UNIT-010 | Protocol codec/parser | Version, length, sequence, integrity, and malformed cases pass | Not run |
| INT-001 | UART/SPI loopback | Repeatable transactions at selected rates | Not run |
| INT-002 | Protocol fault injection | Corrupt/stale/repeated packets rejected and reported | Not run |
| INT-003 | Dedicated control signals | ARM and IRQ/FAULT directions and safe defaults verified | Not run |
| INT-010 | ROS 2 node and interface test | Required nodes, topics, parameters, commands, telemetry, faults, restart, and reset operate coherently | Development pass — ROS2-010 synthetic Jazzy container only |
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

## 7. MODEL-020-SYNTHETIC development record

**Objective:** evaluate the nominal floating-point controller/observer against
an explicit synthetic uncertainty matrix before beginning fixed-point
conversion.

**Mapped requirements:** CTL-003 and the development portion of VER-004.

**Method:** keep the MODEL-010 gains fixed; run 0.8× and 1.2× one-at-a-time
changes to all six motor parameters; add encoder quantisation, seeded position
noise, one- and two-sample unmodelled command delays, reduced voltage, and a
combined stress case; evaluate exact reproducibility, MODEL-010 regression,
finite values, voltage bounds, zero-delay augmented spectral radius, and a
synthetic numerical integrity envelope.

**Result:** development pass for the robustness harness. All 35 repository
unit tests, all 8 machine-readable MODEL-020 robustness checks, and all 9
preflight-audit checks passed. Coefficient freeze and fixed-point conversion
remain on hold.

Key observations:

- the exact nominal case reproduces MODEL-010 state, estimate, and voltage
  arrays;
- all zero-delay linearized augmented spectral radii remain below 1.0;
- the combined stress case has the largest spectral radius, 0.995907, and the
  largest RMS tracking error, 3.431e-3 rad;
- the 2048-count/revolution case has the largest RMS position-estimation
  error, 9.11e-4 rad;
- the combined case has 12 saturated samples at the 4.5 V limit; and
- all states and outputs remain finite and inside the declared synthetic
  integrity envelope.
- the 24-case preflight records 31 signal/intermediate ranges with a declared
  synthetic guard factor;
- the coefficient derivation reproduces exactly, but no physical motor or pole
  provenance exists; and
- one shared 18-bit coefficient binary point is rejected because the current
  coefficient set spans approximately 40.60 binary orders of magnitude.

Evidence:

- `data/processed/model_020_synthetic_robustness_report.json`
- `data/processed/model_020_synthetic_robustness_results.csv`
- `docs/media/model_020_parameter_sweep.png`
- `docs/media/model_020_nonideality_summary.png`
- `data/processed/model_020_synthetic_preflight_report.json`
- `data/processed/model_020_synthetic_numeric_range_budget.csv`
- `data/processed/model_020_synthetic_coefficient_provenance.csv`
- `docs/media/model_020_synthetic_range_budget.png`
- `tests/test_model_020.py`
- `tests/test_model_020_preflight.py`

**Acceptance limitation:** all uncertainty factors and nonidealities are
synthetic. No physical distributions, correlations, parameter identification,
backlash, Coulomb friction, thermal drift, PWM behavior, platform timing, or
hardware operation have been demonstrated. Fixed-point arithmetic was
deliberately not started in this milestone.

**Pre-fixed-point gate:** keep PR #4 in draft pending human review. Do not
freeze coefficients or begin fixed-point conversion until physical parameter
provenance, valid operating ranges, encoder/timing evidence, arithmetic order,
quantisation/stability analysis, and numerical policies are resolved.

## 8. SIM-010-SYNTHETIC development record

**Objective:** implement a deterministic, hardware-independent supervisor
software-in-the-loop bench with configurable synthetic interfaces and
machine-readable safety-state evidence.

**Mapped requirements:** SW-006, SAF-009, VER-006, and the software-development
portions of SAF-001, SAF-004, SAF-005, and SAF-006.

**Method:** reuse the existing `SYNTHETIC-DCM-001` discrete motor model; drive
sample-indexed simulated encoder, H-bridge command, relay enable/feedback,
E-stop, watchdog, supply-monitor, telemetry, and fault interfaces; execute the
safe-startup, ready, running, fault-latched, safe-shutdown, and controlled
recovery state machine; exercise exact threshold and malformed-input
boundaries with independent bench instances; run all scenarios twice and
require exact equality; regenerate committed evidence in CI and reject drift.

**Result:** development pass. All 73 repository unit tests passed, including
38 independent SIM-010 tests. All 11 scenarios and all 7 top-level
machine-readable checks passed. The scenario set produced 85 ordered
telemetry records. Thirteen
records contain a newly detected fault or an active fault latch; every one has
relay enable command false and motor command exactly `0.0`.

Scenario results:

| Scenario | Result | Safety result |
|---|---|---|
| Normal startup and operation | PASS | Safe startup/shutdown; nonzero command only in running |
| E-stop activation | PASS | `EMERGENCY_STOP` latched; same-sample safe output |
| Watchdog timeout | PASS | `WATCHDOG_TIMEOUT` latched at the synthetic sample threshold |
| Stale encoder telemetry | PASS | `ENCODER_STALE` latched at the synthetic sample threshold |
| Encoder failure | PASS | `ENCODER_FAILURE` latched |
| Relay feedback failure | PASS | Command inhibited; `RELAY_FEEDBACK_FAILURE` latched |
| Undervoltage | PASS | `UNDERVOLTAGE` latched |
| Command-voltage saturation | PASS | Finite over-limit command clipped to the synthetic limit; saturation recorded; no fault latched |
| Fault latching | PASS | Source clearance does not clear latch or safe output |
| Rejected unsafe restart | PASS | Reset rejected while arm/run asserted |
| Successful controlled recovery | PASS | Explicit reset enters safe shutdown; a complete disarmed READY sample precedes a later new re-arm sample |

Evidence:

- `models/sil.py`
- `models/validate_sim.py`
- `models/parameters/synthetic_sim_010.json`
- `tests/test_sim_010.py`
- `data/processed/sim_010_synthetic_validation_report.json`
- `data/processed/sim_010_synthetic_scenario_trace.csv`
- `docs/sim_010_software_in_loop.md`

**Synthetic values:** the test bench uses the explicitly synthetic
`SYNTHETIC-DCM-001` plant and the explicitly synthetic SIM-010 configuration:
6.0 V nominal supply, 2.0 V normal command, 6.0 V absolute command limit,
4.5 V undervoltage threshold, two startup samples, two shutdown samples,
three missed-heartbeat samples, three stale-encoder samples, two
relay-mismatch samples, and 4096 encoder counts/revolution. None is a physical
setting, rating, threshold, or validation result.

The synthetic watchdog, encoder-stale, and relay-mismatch counters fault
exactly at their configured `N` samples; `N-1` does not fault. Encoder startup
requires a valid sequence transition after its baseline. Supply exactly at
the synthetic threshold is healthy. Finite commands exactly at the positive
or negative synthetic limit pass unchanged, while finite over-limit commands
clip and continue. Strings, `None`, Booleans in numeric fields, NaN,
infinities, prohibited negative values, and malformed Boolean fields produce
deterministic safe fault telemetry rather than uncaught conversion errors.

**Acceptance limitation:** SIM-010 does not execute ROS 2, GPIO, serial
communications, FPGA RTL, wall-clock scheduling, or hardware. It does not
model physical disable latency, welded relay contacts, E-stop power removal,
ADC error, supply transients, asynchronous encoder behaviour, or component
failure distributions. It does not close SAFE-001 through SAFE-005 hardware
or RTL verification.

**Pre-fixed-point gate:** coefficient freeze and fixed-point conversion remain
on hold. SIM-010 adds no fixed-point coefficients, binary points, word
lengths, arithmetic policies, or RTL.

## 9. ROS2-010-SYNTHETIC development record

**Objective:** integrate typed ROS 2 Jazzy middleware around the authoritative
SIM-010 supervisor without duplicating its safety state machine or introducing
hardware access.

**Mapped requirements:** SW-005, SW-007, COM-003 development portion,
SAF-009, SAF-010, VER-006, and VER-007.

**Method:** create one custom interface package and two ament Python packages;
validate publisher-owned sequences and per-tick freshness; map accepted frames
to `SupervisorInputs`; call `SupervisorTestBench`; publish typed actuator,
safety, supervisor, and fault telemetry; implement an explicit reset service;
run a deterministic synthetic telemetry/fault node; execute independent unit
and real multi-process launch tests; regenerate exact JSON/CSV evidence.

**Local Windows result:** the original preflight baseline passed with 73
tests. After implementation, all 94 repository Python tests passed under
Python 3.13.1. `compileall` and `git diff --check` passed. Native `ros2`,
`colcon`, and `rclpy` were unavailable.

**Local container result:** a locally built image based on the digest-pinned
official `ros:jazzy-ros-base-noble` image ran under Docker Desktop 4.43.2 /
Engine 28.3.2. Python 3.12.3 ran all 94 repository tests. `colcon build`
completed for all three packages. Six distinct ROS-native lint checks passed:
interface `lint_cmake`/`xmllint`, supervisor `ament_flake8`/`ament_pep257`,
and simulator `ament_flake8`/`ament_pep257`. `colcon test` ran those checks
and both launch tests; `colcon test-result --verbose` reported 10 tests, 0
errors, 0 failures, and 0 skips. Explicit lint and launch-test replays also
passed. The evidence generator reported 13 scenarios, 98 traces, 6 passing
top-level checks, exact replay, and byte-equal regeneration.

Launch coverage includes:

- safe startup, a complete disarmed `READY` sample, and running;
- finite voltage clipping without a fault;
- software E-stop, malformed/non-finite command, heartbeat timeout, duplicate
  and out-of-order encoder data, encoder failure, relay feedback failure, and
  undervoltage;
- fault latching, unsafe reset rejection, accepted reset, and controlled
  recovery after every injected fault;
- simultaneous encoder/safety telemetry disappearance; and
- supervisor process exit/respawn with sample-index reset and safe actuator
  output.

Evidence:

- `ros2_ws/src/robot_supervisor_interfaces/`
- `ros2_ws/src/robot_supervisor/`
- `ros2_ws/src/robot_supervisor_sim/`
- `tests/test_ros2_010_core.py`
- `tests/test_ros2_010_evidence.py`
- `data/processed/ros2_010_synthetic_validation_report.json`
- `data/processed/ros2_010_synthetic_message_trace.csv`
- `docs/ros2_010_middleware_integration.md`

**GitHub Actions status:** `.github/workflows/ros2-tests.yml` contains the
same digest-pinned Jazzy build/test/evidence route. It was not run because
this implementation remained uncommitted and unpushed as required.

**Acceptance limitation:** the ROS E-stop is only a software test topic.
Reliable/transient-local QoS, a safe shutdown publication, and a synthetic
actuator consumer do not prove physical power removal or bounded disable
latency. No Raspberry Pi timing, GPIO, physical encoder, relay, supply,
watchdog, motor, E-stop, FPGA, HIL, fixed-point, or RTL result is claimed.

**Pre-fixed-point gate:** coefficient freeze and fixed-point conversion remain
on hold. ROS2-010 adds no coefficients, binary points, word lengths, arithmetic
policy, or RTL.
