# Requirements Traceability Matrix

Status values are `Planned`, `Implemented`, `Verified`, or `Deferred`.
`Implemented` means that a design artefact exists; it does not imply final
acceptance. The plant, controller, and observer rows are implemented only for
the explicitly synthetic development fixture. Hardware-dependent acceptance
remains planned.

| Requirement(s) | Primary design element | Verification / evidence | Status |
|---|---|---|---|
| SYS-001, SYS-003 | Single-axis bench rig; guarded plant | HIL-001 mechanical and plant inspection | Planned |
| SYS-002 | Pi benchmark mode and FPGA/hybrid mode | HIL-020 comparative experiment | Planned |
| SYS-004 | Requirements and matrix in `requirements/` | DOC-001 traceability audit | Planned |
| SYS-005 | Build scripts, model generation, vectors, analysis | REP-001 clean-checkout reproduction | Planned |
| CTL-001, CTL-002 | Parameterized plant model | MODEL-001-SYNTHETIC structural checks | Implemented |
| CTL-003 | State feedback, reference precompensator, and observer | MODEL-010-SYNTHETIC floating-point checks | Implemented |
| CTL-003; VER-004 development portion | Floating-point uncertainty harness, provenance audit, and provisional range budget | MODEL-020-SYNTHETIC robustness and preflight checks | Implemented — fixed-point readiness hold |
| CTL-004 | Sample-enable generator; software scheduler | TIM-001 update-rate and jitter test | Planned |
| CTL-005 | Experiment acceptance limits | DOC-002 acceptance-criteria review | Planned |
| CTL-006, CTL-007, CTL-008, CTL-009 | Fixed-point model and RTL arithmetic | RTL-020 bit-accurate regression; RTL-021 boundary tests | Planned |
| SW-001 | ROS 2 nodes and experiment manager | INT-010 node/interface test | Planned |
| SW-002, SYS-002 | Pi controller node | HIL-020 comparative experiment | Planned |
| SW-003 | Absolute-deadline software loop | TIM-010 Linux load/jitter test | Planned |
| SW-004 | Logger and experiment metadata | DATA-001 logging schema audit | Planned |
| SW-005 | Heartbeat/watchdog interaction | SAFE-010 Pi-process-loss test | Planned |
| FPGA-001 | RTL module hierarchy | RTL unit suite and INT-020 controller integration | Planned |
| FPGA-002 | Clock/reset manager and sample enable | RTL-001 pulse/timing test | Planned |
| FPGA-003 | Input synchronisers and encoder decoder | RTL-002 asynchronous-input test | Planned |
| FPGA-004 | PWM generator | HIL-010 PWM frequency and waveform measurement | Planned |
| FPGA-005 | Telemetry snapshot registers | INT-021 atomic snapshot test | Planned |
| FPGA-006, FPGA-007 | Vivado constraints and build reports | SYN-001 timing/resource review | Planned |
| COM-001 | UART debug and SPI bridge | INT-001 loopback and bring-up | Planned |
| COM-002, COM-003 | Versioned framed protocol | UNIT-010 protocol parser; INT-002 corruption tests | Planned |
| COM-004 | ENABLE/ARM and IRQ/FAULT lines | INT-003 dedicated signal test | Planned |
| COM-005 | Pin map, PCB schematic, constraints | HW-DR-001 electrical-interface review | Planned |
| HW-001, HW-002 | BOM, power design, motor-driver interface | HW-DR-002 ratings review | Planned |
| HW-003, HW-004 | Schematic and wiring diagram | HIL-002 wiring inspection | Planned |
| HW-005 | Mechanical mount and guard | HIL-001 pre-power inspection | Planned |
| SAF-001, SAF-003 | Fault manager, PWM and driver-enable gating | SAFE-001 safe-state latency test | Planned |
| SAF-002 | Fuse, E-stop, current-limited supply | SAFE-002 physical safety test | Planned |
| SAF-004 | Hardware arming interlock | SAFE-003 interlock truth-table test | Planned |
| SAF-005 | Watchdog, encoder, arithmetic, E-stop faults | SAFE-004 fault-injection suite | Planned |
| SAF-006 | Fault reset and re-arm state machine | SAFE-005 reset/re-arm sequence test | Planned |
| SAF-007, SAF-008 | Commissioning plan and checklist | HIL test-record review | Planned |
| VER-001 | Floating-point golden model | MODEL-010 reference-model review | Planned |
| VER-002 | RTL unit and integration tests | CI and regression artefacts | Planned |
| VER-003 | Staged hardware bring-up | HIL-001 through HIL-020 records | Planned |
| VER-004 | Analysis and report pipeline | REPORT-001 final metric review | Planned |
| VER-005 | `data/raw`, `data/processed`, processing scripts | DATA-002 reproducibility audit | Planned |

## Evidence naming

Automated test names should begin with the identifier shown above. Manual test
records should include the test identifier, repository commit, hardware
revision, bitstream/software versions, operator, date, equipment, calibration
status, limits, raw-data path, result, and reviewer.
