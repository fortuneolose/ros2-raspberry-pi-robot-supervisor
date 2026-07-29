# ROS2-010 hardware-independent middleware integration

## Evidence status and scope

`ROS2-010-SYNTHETIC` integrates ROS 2 Jazzy middleware with the verified
`SIM-010-SYNTHETIC` supervisor. The integration is hardware-independent and
explicitly synthetic. It verifies typed ROS interfaces, process boundaries,
message ordering/freshness, periodic supervisor invocation, reset handling,
fault injection, restart behaviour, and safe command publication.

`models/sil.py` remains ROS-independent and authoritative. The ROS packages do
not contain a second safety state machine. `robot_supervisor.core` only:

1. validates publisher-owned message sequences and freshness;
2. maps accepted transport samples to `models.sil.SupervisorInputs`;
3. advances `SupervisorTestBench` once per qualified supervisor tick; and
4. maps its `TelemetryRecord` to typed actuator and diagnostic messages.

This is not physical validation. No GPIO, motor driver, electrical relay,
physical encoder, physical supply monitor, independent watchdog, or physical
emergency-stop function is accessed or represented. In particular,
`software_estop_active` is a simulation input and is **not** a physical safety
function or independent removal of motor power.

Coefficient freeze and fixed-point conversion remain **HOLD**. ROS2-010 adds
no controller coefficients, binary points, word lengths, arithmetic policy,
or RTL.

## Workspace and package ownership

```text
ros2_ws/src/
├── robot_supervisor_interfaces/  ament_cmake interface definitions
├── robot_supervisor/             ament_python SIM-010 adapter and node
└── robot_supervisor_sim/         ament_python synthetic topology
```

| Owner | Responsibility |
|---|---|
| Command publisher | Monotonic `SupervisorCommand.sequence`; arm, run, shutdown, and finite/malformed voltage request |
| `simulator_node` | Monotonic encoder and safety sequences; synthetic telemetry; deterministic fault injection; actuator-command consumption |
| `supervisor_node` | Transport validation, periodic calls into SIM-010, typed output publication, explicit reset service |
| `SupervisorTestBench` in `models/sil.py` | Authoritative state, fault latch, reset decision, recovery interlock, command limit, and safe-output gate |
| Actuator consumer | Retains and acts only on the latest `ActuatorCommand`; ROS2-010's consumer is synthetic |

The `robot_supervisor` ament package installs the existing repository
`models` package and its two authoritative synthetic JSON sources as package
data. There is no copied safety implementation or second parameter file.

## Node, topic, and service graph

```text
test/command owner
  └── /robot_supervisor/command [SupervisorCommand] ───────────────┐
                                                                  │
simulator_node                                                    v
  ├── /robot_supervisor/encoder [EncoderTelemetry] ─────> supervisor_node
  ├── /robot_supervisor/safety_input [SafetyInput] ─────>      │
  ├── /robot_supervisor/set_fault_injection [service]           │
  └── consumes /robot_supervisor/actuator_command <─────────────┤
                                                               │
supervisor_node                                                 │
  ├── /robot_supervisor/actuator_command [ActuatorCommand] ─────┘
  ├── /robot_supervisor/safety_status [SafetyStatus]
  ├── /robot_supervisor/telemetry [SupervisorTelemetry]
  ├── /robot_supervisor/faults [FaultTelemetry]
  └── /robot_supervisor/reset_fault [ResetFault service]
```

`ResetFault` is deliberately separate from `SupervisorCommand`. Its callback
executes one immediate, serialized SIM-010 tick with `reset_request=true`, so
the returned accepted/rejected result and reason come from the authoritative
state machine. Reset does not restart either process and accepted reset does
not cause motion.

`SetFaultInjection` belongs only to the simulator test fixture. Supported
injections cover software E-stop, heartbeat loss, encoder suppression,
duplicate and out-of-order encoder sequences, encoder failure/invalidity/
non-finite data, relay-feedback failure, undervoltage, and safety-stream
suppression/ordering.

## Interface intent

| Interface | Important fields and policy |
|---|---|
| `SupervisorCommand` | Publisher timestamp and sequence; arm/run/shutdown; requested voltage; no reset field |
| `EncoderTelemetry` | Publisher timestamp and sequence; position; health and validity |
| `SafetyInput` | Publisher timestamp and sequence; synthetic supply, software E-stop, heartbeat, and relay feedback |
| `ActuatorCommand` | Supervisor sample, relay enable, motor voltage, saturation, state, publication reason |
| `SafetyStatus` | State, safe-output flag, liveness and exact sample counters, reset decision |
| `SupervisorTelemetry` | SIM-010 input/output mapping plus command/encoder/safety transport dispositions |
| `FaultTelemetry` | Detected, latched, and raw fault sources; input diagnostics; reset result/reason |
| `ResetFault` | Requester label; accepted/rejected, reason, deciding sample, and resulting state |

ROS generated field types prevent many wire-format type errors, but floating
fields can still contain NaN or infinity. Those values are passed through the
adapter to SIM-010's existing strict validation and therefore latch the
corresponding safe fault.

## QoS choices

| Flow | QoS | Reason |
|---|---|---|
| Command, encoder, safety input | Reliable, volatile, keep last 10 | Reject application-level loss/order anomalies explicitly while avoiding replay of an old input after subscriber restart |
| Actuator and all status/telemetry outputs | Reliable, transient local, keep last 1 | Make the latest safe/status sample available to late-joining synthetic consumers and avoid command backlog |
| Services | ROS 2 service default | Short serialized request/response operations |

QoS delivery is not treated as a physical safety mechanism. The application
sequence and age rules still reject duplicates, out-of-order data, and
disappearance.

## Timing, ordering, and age semantics

- The default supervisor timer is an explicitly synthetic 20 ms middleware
  fixture. The simulator publishes at 10 ms. These are not accepted control
  rates or real-time guarantees.
- One qualified ROS supervisor timer callback advances exactly one SIM-010
  sample. SIM-010's inherited motor model still identifies its internal
  synthetic sample period as 1 ms; ROS2-010 does not claim wall-clock
  equivalence between that model sample and the middleware timer.
- Before first contact from both required telemetry owners, the node publishes
  safe actuator output on every timer callback but does not advance SIM-010.
  This DDS-discovery gate cannot establish encoder liveness or reach `READY`.
- After first contact, publisher timestamps are diagnostic only. Freshness is
  based on receipt of a strictly increasing publisher-owned `uint64` sequence
  since the preceding supervisor tick, avoiding cross-host clock assumptions.
- A duplicate is ignored and reported `DUPLICATE`. It supplies no new
  heartbeat or encoder advance. An out-of-order sequence is rejected and
  reported `OUT_OF_ORDER`; encoder ordering invalidates encoder input, command
  ordering produces invalid-command input, and safety ordering produces
  invalid-supply input.
- Missing accepted encoder advancement maps to `encoder_stale=true`.
  SIM-010 retains its exact age definition: the first baseline is age zero,
  `N-1` is healthy, and `N` faults.
- No fresh safety sample maps heartbeat false. SIM-010 retains its exact
  missed-heartbeat boundary: `N-1` is healthy and `N` faults.
- A command retained for the configured synthetic stale window is forced to
  arm/run low, voltage zero, and shutdown requested. It cannot remain an
  indefinite motion request.
- A fresh safety sample compares relay feedback with the prior supervisor
  relay command. Feedback failure remains a raw reset blocker.

## Safety behaviour

SIM-010 continues to enforce these properties:

- every newly detected or latched fault produces relay disabled and motor
  command exactly zero in the same SIM-010 tick;
- finite over-limit voltage clips and reports saturation without faulting;
- malformed/non-finite command input latches `INVALID_COMMAND`;
- missing heartbeat and required telemetry reach deterministic fault
  boundaries after first contact;
- startup and recovery require real accepted encoder-sequence advancement;
- clearing a source cannot clear a latch;
- reset is rejected with arm, run, a detected fault, or a raw source active;
- requests held through safe shutdown or into `READY` cannot start motion;
- one complete disarmed `READY` sample and a later new arm/run sample are
  required;
- node startup publishes safe output before its first SIM-010 tick;
- normal and test-triggered shutdown publish safe output before process exit;
  the synthetic actuator consumer preserves the last safe command it receives.

Neither ROS transient-local durability nor software publication proves
physical power removal or bounded disable latency.

## Simulator topology

`simulator_node` is intentionally simple and nonphysical. It:

- publishes sequence-bearing encoder and safety samples;
- emits optional low arm/run command samples for the demonstration launch;
- consumes `ActuatorCommand` and advances a synthetic position scalar;
- mirrors the latest relay command unless relay failure is injected; and
- applies sample-counted or held deterministic fault injections.

It does not reproduce the authoritative safety state machine. Its position
update is a transport fixture, not a plant-validation result. The
authoritative SIM-010 bench still reuses `SYNTHETIC-DCM-001`.

## Tests and evidence

Host-independent unit tests in `tests/test_ros2_010_core.py` cover mapping,
normal state progression, E-stop, exact watchdog/encoder/relay boundaries,
missing/duplicate/out-of-order encoder samples, encoder failure, undervoltage,
saturation, malformed/non-finite command, latching, unsafe reset, controlled
recovery, restart, telemetry disappearance, and startup/shutdown mappings.

ROS launch testing starts real `simulator_node` and `supervisor_node`
processes. It exercises normal operation, saturation, E-stop, invalid command,
heartbeat timeout, duplicate/out-of-order/failing encoder, relay failure,
undervoltage, unsafe/safe reset, repeated controlled recovery, telemetry
disappearance, safe process exit, and supervisor respawn returning to a safe
sample index.

ROS-native lint is part of `colcon test`, not a separate advisory procedure.
`robot_supervisor_interfaces` uses `ament_lint_auto` for `lint_cmake` and
`xmllint`; each Python package registers executable `ament_flake8` and
`ament_pep257` pytest checks over its maintained source, launch, setup, and
test files. Generated interfaces and build/install/log trees are outside that
source-lint scope. The shared validation script also executes the six distinct
lint checks explicitly before the full colcon test.

The deterministic generator writes:

- `data/processed/ros2_010_synthetic_validation_report.json`
- `data/processed/ros2_010_synthetic_message_trace.csv`

The current evidence contains 13 passing scenarios and 98 message-level
traces. It executes twice and requires exact equality. CI/container
validation regenerates temporary files and requires byte equality with both
committed artifacts.

## Reproduction

With an existing ROS 2 Jazzy environment:

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/validate_ros2_010.sh
```

With Docker:

```bash
docker build -f ros2_ws/Dockerfile.jazzy -t ros2-010-jazzy:local .
docker run --rm ros2-010-jazzy:local
```

The Dockerfile pins the official Jazzy base image by digest. It installs
dependencies inside the image only; it does not install or configure ROS,
WSL, or Docker on the host.

After a manual build:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_ws/src
source install/setup.bash
ros2 launch robot_supervisor_sim simulated_topology.launch.py
```

## Recorded execution boundary

- Windows host: original 73-test preflight and final 94-test Python suite
  passed with Python 3.13.1; ROS 2, colcon, and rclpy were unavailable.
- Local Docker container: official digest-pinned ROS 2 Jazzy/Noble image,
  Python 3.12.3; all three packages built, six distinct lint checks passed,
  `colcon test-result --verbose` reported 10 tests with no failures, and both
  launch tests passed under `colcon test` and explicit replay.
- GitHub Actions: `.github/workflows/ros2-tests.yml` is implemented but was
  not run from this uncommitted, unpushed working tree.

Passing software tests do not close any GPIO, electrical, physical safety,
real-time, HIL, fixed-point, or RTL acceptance activity.
