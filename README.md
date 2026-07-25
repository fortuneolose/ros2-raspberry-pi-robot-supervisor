# Raspberry Pi–FPGA Robot Supervisor

This repository is the engineering workspace for an MEng capstone project that
implements and evaluates an observer-based, fixed-point state-space controller
for an encoder-equipped robotic drive.

The target platform combines:

- a Raspberry Pi 5 for ROS 2 supervision, experiment orchestration, reference
  generation, telemetry, logging, and a software-only controller benchmark;
- a Digilent Basys 3 (Artix-7 XC7A35T) for deterministic encoder capture,
  state estimation, state feedback, fixed-point arithmetic, PWM, watchdogs,
  and hardware-level fault handling; and
- a protected interface PCB, rated motor driver, encoder-equipped motor, and
  independent power and emergency-stop provisions.

## Project status

Initial repository and requirements baseline. The compulsory demonstrator is a
single, securely mounted motor test rig. A differential-drive mobile robot is
an optional extension after the controller, observer, fixed-point datapath, and
safety functions pass their acceptance tests.

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
src/            ROS 2 interfaces, GPIO, feedback, safety, and bring-up code
tests/          Unit, integration, and hardware-in-the-loop tests
data/           Raw captures and processed experimental results
.github/        Continuous-integration workflows
```

## Development sequence

1. Freeze scope, select the motor/driver, and review the safety concept.
2. Identify and validate the plant model.
3. Design the controller and observer in floating point.
4. build a bit-accurate fixed-point reference model.
5. Implement and instrument the Raspberry Pi benchmark.
6. Verify the FPGA modules and integrated datapath against common vectors.
7. Bring up UART, SPI, telemetry, watchdog, and fault handling.
8. Commission hardware at conservative current, voltage, speed, and duty
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
- [Bill of materials](hardware/bill_of_materials.csv)
