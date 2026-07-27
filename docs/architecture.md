# System Architecture

## 1. Architectural goals

The architecture separates nondeterministic, high-level supervision from the
hard-real-time control and safety path. It must support an apples-to-apples
comparison between a Raspberry Pi-only controller and a hybrid implementation
without allowing Linux scheduling or a failed ROS 2 process to defeat the
hardware safe state.

## 2. System context

| Element | Responsibilities |
|---|---|
| Development workstation | Plant identification, controller design, coefficient generation, Vivado builds, automated tests, analysis, and reports |
| Raspberry Pi 5 | ROS 2 graph, references, experiment state machine, FPGA configuration, telemetry, logging, dashboard, and software benchmark |
| Basys 3 FPGA | Sample timing, encoder capture, velocity estimation, observer, state feedback, fixed-point arithmetic, PWM, watchdog, fault latch, and telemetry snapshots |
| Interface PCB | Protected and labelled Pi/Pmod/encoder/driver interconnect, safe defaults, optional conditioning, and test points |
| Motor driver | Rated power conversion from PWM/direction/enable to bidirectional motor current |
| Plant and sensors | Encoder-equipped DC motor, mechanical load, and optional current/voltage/temperature sensing |
| Safety and power | Independent rated supplies, current limiting, fuse, emergency stop, driver disable, guards, and controlled grounding |

## 3. Logical data flow

```text
reference_node ─┐
experiment_mgr ─┼─> fpga_bridge_node ─SPI─> command/config registers
parameters ─────┘          ▲                      │
                           │                      ▼
logger/dashboard <── telemetry/status <── atomic snapshot

encoder A/B ─> synchronisers ─> decoder ─> measurement vector
                                              │
sample_enable ─> observer ─> state feedback ─> saturation ─> PWM/driver
                       ▲             ▲               │
                       └ coefficients/reference ─────┘

E-stop + watchdog + encoder/arithmetic faults ─> fault manager ─┐
ARM + valid coefficients + reset state ─────────────────────────┼─> enable gate
controller PWM request ──────────────────────────────────────────┘
```

The fault manager and final enable gate are architecturally downstream of the
controller. A valid numerical command cannot override a safety fault.

## 4. Raspberry Pi software

The initial ROS 2 decomposition is:

| Component | Role |
|---|---|
| `reference_node` | Generates speed/position steps, trajectories, and disturbances |
| `fpga_bridge_node` | Encodes SPI commands, decodes atomic telemetry, manages version/sequence/integrity checks, and publishes faults |
| `pi_controller_node` | Runs the floating-point benchmark with timing instrumentation |
| `experiment_manager` | Owns disarmed/armed/running/stopping experiment transitions and metadata |
| `logger_node` | Records synchronized command, state, timing, control, and fault data |
| analysis/dashboard | Displays live status and produces reproducible post-run metrics |

The software benchmark uses an absolute next-deadline schedule. The logger
records the requested deadline, actual start time, computation time, and missed
deadline count rather than assuming a nominal sample period.

## 5. FPGA RTL

The intended module boundaries are:

| Module | Function |
|---|---|
| `clock_reset_manager` | Synchronous reset handling, free-running counters, one-cycle sample enable |
| `quadrature_decoder` | Input synchronisation, transition validation, signed position and direction |
| `velocity_estimator` | Count-window or reciprocal-period estimate and filtering |
| `spi_slave` | Framed register transactions, telemetry reads, version/status/integrity handling |
| `uart_debug` | Bring-up and low-rate diagnostics |
| `state_observer` | Fixed-point prediction and innovation correction |
| `state_feedback` | State-feedback dot product and reference precompensation |
| `saturation_rounding` | Defined rounding, clipping, overflow, and sticky flags |
| `pwm_generator` | Carrier, signed command conversion, direction, and safe disable |
| `watchdog_fault_manager` | Communications timeout, encoder/E-stop/arithmetic faults, latching and re-arm |
| `telemetry_registers` | Atomic sample-boundary snapshot and monotonic counter |

All logic remains in the 100 MHz board-clock domain wherever practical.
The controller advances on a one-cycle `sample_enable`; no generated 1 kHz
clock is required.

## 6. Numerical architecture

The original starting hypothesis was signed 24-bit states, signed 18-bit
coefficients, and full-width products accumulated into 48 bits where DSP
mapping permits. No part of that hypothesis is a frozen format.

The `MODEL-020-PREFLIGHT-SYNTHETIC` audit shows that the current coefficients
span approximately 40.60 binary orders of magnitude. A single shared 18-bit
coefficient binary point is therefore not supported: representing the largest
gain leaves only seven fractional bits and collapses several smaller model
terms toward zero. The next numerical study must evaluate state normalization
and block-, row-, or coefficient-specific scaling. The current 24- and 48-bit
state/accumulator suggestions also remain unapproved until valid physical
ranges and operation ordering are frozen.

The bit-accurate reference model and RTL must agree on:

- matrix-operation ordering;
- coefficient and state scale factors;
- signed extension and intermediate widths;
- rounding mode;
- saturation limits;
- overflow detection; and
- whether the control law uses the current or next observer state.

## 7. Communications

UART is the first bring-up interface. SPI becomes the primary register and
telemetry transport after the protocol is stable. A frame contains protocol
version/synchronisation, operation or address, sequence, payload length,
payload, status, and an integrity check. Registers and telemetry are
little/big-endian only after an interface-control decision records the choice.

Dedicated signals:

| Signal | Direction | Function |
|---|---|---|
| MOSI | Pi to FPGA | Commands, references, coefficients, modes |
| MISO | FPGA to Pi | State, encoder, control, timing, and status |
| SCLK | Pi to FPGA | SPI clock |
| CS | Pi to FPGA | Transaction framing |
| ENABLE/ARM | Pi to FPGA | High-level permission request only |
| IRQ/FAULT | FPGA to Pi | Telemetry-ready or urgent-fault indication |
| GND | Shared reference | Required for the proposed non-isolated 3.3 V interface |

Final GPIO/Pmod/package assignments remain open until checked against the exact
Raspberry Pi configuration, Basys 3 master constraints, and PCB revision.

## 8. Timing baseline

| Function | Initial value |
|---|---|
| FPGA master clock | 100 MHz |
| Control/observer update | 1 kHz, subject to plant analysis |
| PWM carrier | Approximately 20 kHz, subject to driver/motor review |
| Pi reference update | 50–100 Hz |
| Normal telemetry | 100–500 Hz plus diagnostic burst mode |
| Watchdog | Multiple control periods; value selected by hazard and nuisance-trip analysis |

## 9. Operating modes

1. **Reset/disarmed:** outputs forced safe; register access and diagnostics only.
2. **Configured:** compatible protocol and valid coefficient set loaded.
3. **Armed:** safety prerequisites valid; actuator remains at zero until a
   controlled run transition.
4. **Running:** observer/control/PWM active within configured limits.
5. **Faulted:** PWM and driver enable forced inactive; fault status latched as
   required.

Fault recovery returns through the disarmed state. It cannot transition
directly from faulted to running.
