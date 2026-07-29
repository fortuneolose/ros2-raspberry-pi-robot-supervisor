# Raspberry Pi–FPGA Robot Supervisor

This repository is the engineering workspace for an independent capstone-style
engineering project implementing and evaluating an observer-based, fixed-point
state-space controller for an encoder-equipped robotic drive.

The target platform combines:

- a Raspberry Pi 5 for ROS 2 supervision, experiment orchestration, reference
  generation, telemetry, logging, and a software-only controller benchmark;
- a Digilent Basys 3 (Artix-7 XC7A35T) for deterministic encoder capture,
  state estimation, state feedback, fixed-point arithmetic, PWM, watchdogs,
  and hardware-level fault handling; and
- a protected interface PCB, rated motor driver, encoder-equipped motor, and
  independent power and emergency-stop provisions.

## Project status

Requirements, architecture, and safety baselines plus four executable
synthetic software milestones: a parameterized plant model, an observer-based
floating-point controller, a deterministic robustness/uncertainty analysis,
and the SIM-010 hardware-independent supervisor software-in-the-loop bench.
The compulsory demonstrator is a single, securely mounted motor test rig. A
differential-drive mobile robot is an optional extension after the controller,
observer, fixed-point datapath, and safety functions pass their acceptance
tests.

Initial design targets are a 1 kHz control/observer update and an approximately
20 kHz PWM carrier. These values and the final control-performance thresholds
must be justified after plant identification.

## Intended control partition

```text
ROS 2 references/configuration                 deterministic control and safety
┌────────────────────┐       SPI/UART        ┌───────────────────────────────┐
│ Raspberry Pi 5     │ ────────────────────> │ Basys 3 FPGA                  │
│ supervisor/logger  │ <──────────────────── │ encoder/observer/control/PWM  │
└────────────────────┘   telemetry + faults  └───────────────┬───────────────┘
                                                             │
                                                        motor driver
                                                             │
                                                    encoder-equipped motor
```

Dedicated `ENABLE/ARM` and `IRQ/FAULT` signals supplement the serial link.
FPGA safety logic always has authority to disable PWM independently of Linux
or ROS 2.

## Repository layout

```text
requirements/   System requirements and requirements-to-test traceability
docs/           Architecture, safety concept, test evidence, and media
hardware/       Schematics, wiring, BOM, and component datasheets
models/         Plant identification and floating/fixed-point models
rtl/            FPGA SystemVerilog or Verilog modules
constraints/    Basys 3 XDC constraints
scripts/        Coefficient generation, builds, and result processing
ros2_ws/        ROS 2 packages
src/            ROS 2 interfaces, GPIO, feedback, safety, and bring-up code
tests/          Unit, integration, and hardware-in-the-loop tests
data/           Raw captures and processed experimental results
.github/        Continuous-integration workflows
```

## First executable non-hardware milestone

The repository now includes a parameterized floating-point DC motor model and
automated structural tests. The included `SYNTHETIC-DCM-001` values are a
software fixture, not measurements or specifications for the final motor.

Reproduce the current development evidence:

```bash
python -m unittest discover -s tests -p "test_*.py" -v
MPLCONFIGDIR=.matplotlib python -m models.validate_model
```

The checks exercise model construction, units, 1 ms zero-order-hold
discretisation, voltage-input controllability, encoder-position observability,
zero-input equilibrium, and a finite 1 V open-loop development response. See
[the model baseline](docs/model_baseline.md) for assumptions and limitations.

## Second executable non-hardware milestone

The repository also includes discrete state-feedback pole placement, a
steady-state reference precompensator, a Luenberger observer, voltage
saturation, closed-loop simulation, load-pulse testing, and dimensioned
observer-convergence checks. Reproduce the evidence with:

```bash
MPLCONFIGDIR=.matplotlib python -m models.validate_controller
```

The current gains, pole targets, 10 mrad reference, 6 V limit, 1 mN m load
pulse, and performance thresholds are synthetic development fixtures. They
are not final control requirements or hardware-ready coefficients. See the
[controller and observer baseline](docs/controller_observer_baseline.md).

## Third executable non-hardware milestone

`MODEL-020-SYNTHETIC` keeps the MODEL-010 controller and observer fixed while
varying all six plant parameters one at a time and exercising encoder
quantisation, seeded sensor noise, one- and two-sample command delay, reduced
voltage, and a combined development stress case:

```bash
MPLCONFIGDIR=.matplotlib python -m models.validate_robustness
```

Twenty scenarios pass eight deterministic integrity checks. The ±20%
parameter factors and nonidealities are illustrative software fixtures, not
measured hardware uncertainties. See the
[robustness and uncertainty baseline](docs/robustness_uncertainty_baseline.md).
No fixed-point conversion is included in this milestone.

A separate
[coefficient-provenance and numeric-range audit](docs/coefficient_provenance_and_numeric_range_budget.md)
reproduces the current gains and records 31 signal/intermediate ranges across
24 synthetic cases. Audit execution passes, but coefficient freeze and
fixed-point readiness remain **on hold** because the plant, pole targets,
operating ranges, and uncertainty bounds are not physical. The audit also
rejects one global binary point for the provisional 18-bit coefficient-width
hypothesis.

## Fourth executable non-hardware milestone

`SIM-010-SYNTHETIC` is a deterministic, hardware-independent supervisor test
bench. It reuses the existing synthetic DC motor model and provides
configurable in-memory interfaces for the encoder, H-bridge command, relay
enable/feedback, emergency stop, watchdog, supply monitor, telemetry, and
fault events. It contains no ROS 2 runtime dependency or GPIO access.

```bash
python -m models.validate_sim
```

The 11 scenarios cover normal startup/operation, E-stop, watchdog timeout,
stale and failed encoder telemetry, relay feedback failure, undervoltage,
command-voltage saturation, fault latching, rejected unsafe restart, and
controlled recovery. Every detected or latched safety fault record must have
relay command disabled and motor command exactly zero. The validator executes
the full suite twice and requires exact equality before writing:

- `data/processed/sim_010_synthetic_validation_report.json`
- `data/processed/sim_010_synthetic_scenario_trace.csv`

Startup requires a real encoder sequence transition, not only an initial
sequence value. Recovery requires a complete READY sample with arm/run low
before a later new arm/run sample; requests held through safe shutdown are
ignored. Finite voltage requests beyond the synthetic limit are clipped and
reported without latching, while malformed or non-finite commands fault and
force safe outputs. CI regenerates the JSON and CSV and rejects evidence
drift.

All plant values, operating values, limits, and fault thresholds are
explicitly synthetic. SIM-010 is software-behaviour evidence, not physical,
ROS 2 runtime, GPIO, timing, or hardware validation. See the
[SIM-010 software-in-the-loop baseline](docs/sim_010_software_in_loop.md).
Coefficient freeze and fixed-point conversion remain **on hold**.

## Development sequence

1. Freeze scope, select the motor/driver, and review the safety concept.
2. Identify and validate the plant model.
3. Design the controller and observer in floating point.
4. Evaluate plant uncertainty, sensing limits, delay, and actuator constraints.
5. Freeze numeric ranges and build a bit-accurate fixed-point reference model.
6. Implement and instrument the Raspberry Pi benchmark.
7. Verify the FPGA modules and integrated datapath against common vectors.
8. Bring up UART, SPI, telemetry, watchdog, and fault handling.
9. Commission hardware at conservative current, voltage, speed, and duty
   limits before comparative experiments.

## Safety

This is a design-stage repository, not an approval to energise hardware.
Component ratings, electrical levels, pin assignments, FPGA constraints,
guards, and laboratory procedures must be verified for the exact hardware
revision before use. Motor power must use suitable fusing, current limiting,
a reachable emergency stop, and a driver selected for motor stall current.
See [docs/safety_concept.md](docs/safety_concept.md).

## Documentation baseline

- [System requirements](requirements/system_requirements.md)
- [Traceability matrix](requirements/traceability_matrix.md)
- [Architecture](docs/architecture.md)
- [Safety concept](docs/safety_concept.md)
- [Test report](docs/test_report.md)
- [SIM-010 software-in-the-loop baseline](docs/sim_010_software_in_loop.md)
- [Bill of materials](hardware/bill_of_materials.csv)
- [Pre-hardware software activity record, 25 July 2026](docs/reports/pre_hardware_software_activity_record_2026-07-25.docx)
- [Controller and observer work session record, 27 July 2026](docs/reports/controller_observer_work_session_record_2026-07-27.docx)
