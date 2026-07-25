# Verification and Test Report

## 1. Baseline status

**Status: not executed.** This file defines the initial verification record
structure. No hardware, software, RTL, timing, safety, or performance result is
claimed by the repository-initialisation commit.

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
| MODEL-001 | Plant/controller structural checks | Model documented; controllability and observability checks pass | Not run |
| MODEL-010 | Floating-point control tests | Frozen stability and performance limits pass | Not run |
| RTL-001 | Sample-enable and reset | Exactly one update pulse per configured interval; safe reset | Not run |
| RTL-002 | Encoder synchronisation/decode | Valid directions/counts; invalid transitions detected | Not run |
| RTL-020 | Bit-accurate controller regression | Matches approved fixed-point reference tolerance | Not run |
| RTL-021 | Numerical boundaries | Rounding, saturation, sign, and overflow cases pass | Not run |
| UNIT-010 | Protocol codec/parser | Version, length, sequence, integrity, and malformed cases pass | Not run |
| INT-001 | UART/SPI loopback | Repeatable transactions at selected rates | Not run |
| INT-002 | Protocol fault injection | Corrupt/stale/repeated packets rejected and reported | Not run |
| INT-003 | Dedicated control signals | ARM and IRQ/FAULT directions and safe defaults verified | Not run |
| INT-020 | Integrated FPGA control path | Observer, controller, PWM, telemetry, and faults operate coherently | Not run |
| INT-021 | Atomic telemetry | Snapshot values are stable and sample counter is monotonic | Not run |
| SYN-001 | Vivado implementation | Required timing closes; constraints complete; usage recorded | Not run |
| SAFE-001 | Safe-state disable latency | PWM/enable reach safe state within frozen limit | Not run |
| SAFE-002 | E-stop/fuse/current limit | Physical protections present and functional | Not run |
| SAFE-003 | Arming interlock | No invalid input combination enables the actuator | Not run |
| SAFE-004 | Fault-injection suite | Each required fault produces the specified state/status | Not run |
| SAFE-005 | Reset and re-arm | No automatic restart; explicit sequence required | Not run |
| TIM-001 | FPGA update timing | No missed update; rate and jitter meet frozen limits | Not run |
| TIM-010 | Pi scheduling under load | Period, latency, jitter, and misses recorded | Not run |
| HIL-001 | Mechanical/pre-power inspection | Mount, guard, polarity, continuity, limits approved | Not run |
| HIL-010 | Low-duty open-loop test | Direction, encoder, current, PWM, and stop controls correct | Not run |
| HIL-020 | Comparative closed-loop experiment | Pi-only and hybrid datasets complete under matched conditions | Not run |
| DATA-001 | Logging audit | Required fields and metadata present | Not run |
| REP-001 | Clean-checkout reproduction | Build, vectors, analysis, and report reproduce successfully | Not run |

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
