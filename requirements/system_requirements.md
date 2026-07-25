# System Requirements

## 1. Purpose and conventions

These requirements establish the initial baseline for the single-axis motor
demonstrator described in the project design dossier dated 19 July 2026.
“Shall” statements are mandatory. Values marked **preliminary** must be
confirmed after plant identification and hardware selection.

Each requirement has a stable identifier used by the
[traceability matrix](traceability_matrix.md). Changes to an identifier or its
acceptance method require review.

## 2. Scope and plant

| ID | Requirement | Verification |
|---|---|---|
| SYS-001 | The compulsory demonstrator shall control one securely mounted, encoder-equipped DC gearmotor in speed or position mode. | Inspection and HIL test |
| SYS-002 | The system shall provide comparable Raspberry Pi-only and FPGA/hybrid controller configurations using the same plant, commands, gains, and experiment conditions. | Integration test |
| SYS-003 | A second motor, mobile chassis, navigation, and perception shall remain optional until all mandatory acceptance tests pass. | Project review |
| SYS-004 | Every top-level requirement shall map to a design element and at least one verification activity. | Traceability review |
| SYS-005 | Builds, coefficient generation, test vectors, bitstream instructions, and result processing shall be reproducible from version-controlled artefacts. | Repository audit |

## 3. Control and numerical behaviour

| ID | Requirement | Verification |
|---|---|---|
| CTL-001 | The controller shall use a discrete-time state-space plant model and document its sample period, states, inputs, outputs, and physical units. | Analysis review |
| CTL-002 | Controllability and observability shall be established for the selected model before gains are accepted. | Automated model test |
| CTL-003 | The implementation shall provide state feedback, reference precompensation, and a state observer when required states are not directly measured. | Model and integration test |
| CTL-004 | The initial control and observer update target shall be 1 kHz; the final rate shall be justified from plant bandwidth, latency, and computation budget. | Timing analysis |
| CTL-005 | Final rise-time, overshoot, settling-time, steady-state error, RMS tracking error, and disturbance-recovery limits shall be frozen before acceptance testing. | Requirements review |
| CTL-006 | The fixed-point reference model shall reproduce the RTL arithmetic order, word lengths, rounding, saturation, and overflow behaviour. | Model-to-RTL comparison |
| CTL-007 | Closed-loop stability shall be re-evaluated after coefficient and state quantisation. | Automated model test |
| CTL-008 | In non-saturating operation, FPGA state and control sequences shall match the bit-accurate reference model within the approved tolerance; the initial target is one least significant bit. | Cocotb regression |
| CTL-009 | Saturation, rounding, and overflow behaviour shall be explicit, deterministic, and covered by boundary tests. | Unit and integration test |

## 4. Raspberry Pi and ROS 2 software

| ID | Requirement | Verification |
|---|---|---|
| SW-001 | The Raspberry Pi shall provide reference generation, experiment orchestration, parameter management, telemetry publication, and data logging. | Integration test |
| SW-002 | The Raspberry Pi shall provide a floating-point software-only controller benchmark. | Integration and HIL test |
| SW-003 | A software control loop shall use absolute next-deadline scheduling and record actual period, computation time, and missed deadlines. | Timing test |
| SW-004 | The system shall log timestamps, references, measured states, estimated states, control effort, saturation, and fault events with experiment metadata. | Data audit |
| SW-005 | Loss or failure of a Raspberry Pi process shall not leave the actuator enabled indefinitely. | Fault-injection test |

## 5. FPGA behaviour

| ID | Requirement | Verification |
|---|---|---|
| FPGA-001 | The FPGA shall perform sample timing, encoder capture, state estimation, state feedback, actuator limiting, PWM generation, watchdog monitoring, and fault latching. | RTL and integration test |
| FPGA-002 | FPGA logic shall use the 100 MHz board clock and a one-cycle sample-enable pulse rather than a derived control clock. | RTL inspection and timing test |
| FPGA-003 | External asynchronous inputs shall use suitable synchronisation and transition validation before use by control logic. | RTL inspection and simulation |
| FPGA-004 | The initial PWM carrier target shall be approximately 20 kHz and shall be confirmed against motor-driver ratings, losses, and motor behaviour. | Measurement and review |
| FPGA-005 | Telemetry shall be captured atomically at a control-sample boundary and include a monotonic sample counter. | Integration test |
| FPGA-006 | Synthesis and implementation shall meet the approved 100 MHz timing constraints without critical warnings or unconstrained required I/O paths. | Vivado report review |
| FPGA-007 | DSP, LUT, flip-flop, block-RAM usage, timing margin, and power estimate shall be recorded for each evaluated state-space realisation. | Implementation report |

## 6. Communications and interfaces

| ID | Requirement | Verification |
|---|---|---|
| COM-001 | UART shall be available for initial bring-up and diagnostics; SPI shall be the primary final command and telemetry link. | Integration test |
| COM-002 | The protocol shall identify its version and carry message type/address, payload length, sequence information, status, and an integrity check. | Protocol unit test |
| COM-003 | The Raspberry Pi shall detect dropped, repeated, malformed, and incompatible transactions. | Fault-injection test |
| COM-004 | The interface shall provide dedicated high-level `ENABLE/ARM` and FPGA `IRQ/FAULT` signals in addition to serial communications. | Inspection and integration test |
| COM-005 | Raspberry Pi and Basys 3 signal voltage compatibility and final pin assignments shall be verified against current manufacturer documentation before connection. | Design review |

## 7. Hardware and safety

| ID | Requirement | Verification |
|---|---|---|
| HW-001 | The motor driver shall be rated with margin above the selected motor’s measured or specified stall current and shall expose a hardware enable input. | Datasheet and design review |
| HW-002 | Raspberry Pi, FPGA, and motor power shall use appropriately rated supplies; logic-board 5 V rails shall not be paralleled. | Schematic and inspection |
| HW-003 | High motor, battery, H-bridge, and other switching currents shall not pass through a solderless breadboard or logic-board return path. | Wiring inspection |
| HW-004 | Final interconnects shall be keyed, labelled, strain-relieved where needed, and provide accessible signal-grounded test points. | Inspection |
| HW-005 | The rotating assembly shall be rigidly mounted and guarded for powered tests. | Pre-test inspection |
| SAF-001 | The safe state shall force PWM inactive and motor-driver enable inactive. | Fault-injection and measurement |
| SAF-002 | A reachable physical emergency stop, suitable fuse, and current-limited commissioning supply shall be provided in the motor power path. | Inspection and functional test |
| SAF-003 | FPGA fault logic shall override controller commands and disable the actuator without waiting for Raspberry Pi or ROS 2 action. | RTL and HIL fault test |
| SAF-004 | PWM shall be enabled only when reset is released, valid coefficients are loaded, emergency stop is inactive, watchdog is healthy, no latching fault is active, and an arm command is present. | Safety-state transition test |
| SAF-005 | Communication timeout, encoder invalidity, arithmetic overflow, and emergency-stop assertion shall latch or maintain the safe state according to the approved fault policy. | Fault-injection test |
| SAF-006 | Re-arming after a fault shall require the fault source to be absent and an explicit reset/re-arm sequence; reset alone shall not cause motion. | HIL test |
| SAF-007 | Each commissioning stage shall define maximum voltage, current, speed, and command limits before motor power is enabled. | Test-plan review |
| SAF-008 | Hardware changes shall be followed by power-off continuity, polarity, connector-orientation, and default-enable checks. | Signed checklist |

## 8. Verification and evidence

| ID | Requirement | Verification |
|---|---|---|
| VER-001 | MATLAB or an approved equivalent shall provide the authoritative floating-point plant and control model until model changes are accepted. | Model review |
| VER-002 | Each RTL module shall have automated unit tests, and the integrated RTL shall use shared deterministic vectors from the reference models. | CI/regression report |
| VER-003 | Physical commissioning shall proceed from power-off inspection through separated power, communications, synthetic encoder, low-duty open loop, observer-only, conservative closed loop, and fault injection. | Signed test record |
| VER-004 | Comparative experiments shall report control performance, timing, numerical error, resource usage, robustness, and safety-response metrics. | Test report review |
| VER-005 | Raw experimental captures shall be immutable; processing steps and generated results shall be reproducible and versioned. | Data audit |
