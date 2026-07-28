# SIM-010 deterministic supervisor software-in-the-loop baseline

## Evidence status

`SIM-010-SYNTHETIC` is an implemented, deterministic, hardware-independent
software test bench. It verifies supervisor logic and simulated interface
contracts before ROS 2, GPIO, FPGA RTL, or physical hardware exists.

This is not physical validation. Every plant value, operating value, limit,
and fault threshold used by SIM-010 is explicitly synthetic. Passing SIM-010
does not validate a motor, encoder, H-bridge, relay, E-stop, supply, watchdog,
Raspberry Pi, ROS 2 process, FPGA, or disable latency.

Coefficient freezing and fixed-point conversion remain **on hold**. SIM-010
introduces no coefficients, binary points, word lengths, or RTL.

## Design

The bench advances one sample at a time. It uses no wall clock, thread,
random source, ROS 2 runtime, or GPIO access:

```text
scripted SupervisorInputs
        |
        +--> E-stop / watchdog / supply monitors --------+
        +--> encoder interface --> freshness / health ---+--> supervisor
        +--> arm, run, shutdown, reset requests ----------+      |
                                                               safety gate
                                                                  |
                              +-------------------+---------------+
                              |                   |
                        relay command       H-bridge command
                              |                   |
                        relay feedback -----------+
                                                  |
                                      existing synthetic DC motor
                                                  |
                                ordered telemetry and fault channels
```

Inputs are type-checked and normalised before any interface or state-machine
logic runs. Malformed values produce ordered invalid-input fault telemetry
instead of an uncaught conversion exception. Primary safety faults are
evaluated before actuation. Relay feedback is then checked after the simulated
relay command. A newly detected relay-feedback fault overrides the output in
that same sample. Any detected fault or active fault latch requires:

```text
relay_enable_command == false
motor_command_v == 0.0
```

The H-bridge also requires both the relay command and relay feedback before it
passes a nonzero command. This means a missing relay feedback signal inhibits
the simulated motor even during the synthetic mismatch-detection window.

## Simulated interfaces

| Interface | SIM-010 behaviour | Configuration or injection |
|---|---|---|
| DC motor | Reuses the existing three-state, zero-order-hold `SYNTHETIC-DCM-001` floating-point model | Synthetic motor JSON and synthetic opposing load |
| Encoder | Quantised position plus validity, health, liveness, and monotonic sequence | Synthetic counts/revolution; per-sample stale, failed, or invalid injection |
| H-bridge command | Enable gating and finite-command clipping | Synthetic absolute command limit |
| Relay enable | Command plus simulated feedback | Per-sample feedback-failed injection; synthetic mismatch threshold |
| Emergency stop | Boolean sampled input | Per-sample active/inactive injection |
| Watchdog | Counts consecutive samples without heartbeat | Synthetic missed-heartbeat threshold |
| Supply monitor | Compares a finite sampled voltage with a threshold | Synthetic nominal voltage and undervoltage threshold |
| Telemetry channel | Ordered in-memory `TelemetryRecord` sequence | One record per simulated sample |
| Fault channel | Ordered in-memory latch and reset events | Fault code, action, state, and sample index |

All interfaces are ordinary Python objects. They do not open devices, sockets,
ROS 2 topics, serial ports, or GPIO lines.

## Supervisor state machine

```text
SAFE_STARTUP --liveness + healthy window--> READY --qualified arm + run--> RUNNING
      |                           |                    |
      | fault                     | fault              | stop/de-arm
      v                           v                    v
FAULT_LATCHED <---------------- fault ---------- SAFE_SHUTDOWN
      |
      | source clear + arm/run low + explicit reset
      v
SAFE_SHUTDOWN --safe hold--> READY --disarmed sample, then new arm + run--> RUNNING
```

- `SAFE_STARTUP` holds safe outputs until a valid encoder sequence baseline is
  followed by a genuine valid sequence transition and the synthetic healthy
  window completes.
- `READY` remains safe. On every entry it first requires one complete sample
  with arm and run both low. Only a subsequent sample with both high can enter
  `RUNNING`.
- `RUNNING` is the only state that can request relay enable and a nonzero motor
  command.
- `FAULT_LATCHED` preserves safe outputs after the initiating source clears.
- `SAFE_SHUTDOWN` holds safe outputs before returning to `READY`.
- Reset is rejected while a fault source remains or either arm or run is
  asserted. A persistent raw relay-failure source blocks reset even after the
  relay mismatch counter returns to zero with relay command and feedback low.
- An accepted reset never causes motion. Recovery must pass through
  `SAFE_SHUTDOWN`; arm/run values sampled there are ignored. A held request on
  entry to `READY` is also ignored until a full disarmed READY sample and a
  later new arm/run sample have been observed. Recovery cannot bypass encoder
  liveness: safe shutdown cannot return to READY until a genuine valid
  sequence transition has been observed.

## Explicitly synthetic configuration

The versioned values below come from
`models/parameters/synthetic_sim_010.json`. They are software fixtures, not
physical limits or specifications.

| Label in configuration | Synthetic value | Meaning in SIM-010 only |
|---|---:|---|
| `synthetic_plant.motor_parameter_set_id` | `SYNTHETIC-DCM-001` | Existing illustrative motor model |
| `synthetic_plant.opposing_load_torque_nm` | 0.0 N m | Test-bench plant input |
| `synthetic_operating_values.nominal_supply_voltage_v` | 6.0 V | Healthy scenario supply sample |
| `synthetic_operating_values.normal_run_command_voltage_v` | 2.0 V | Normal scenario H-bridge request |
| `synthetic_operating_limits.command_voltage_abs_max_v` | 6.0 V | Finite-command clipping boundary |
| `synthetic_operating_limits.undervoltage_threshold_v` | 4.5 V | Supply-fault boundary |
| `synthetic_fault_thresholds.safe_startup_healthy_samples` | 2 samples | Startup hold |
| `synthetic_fault_thresholds.safe_shutdown_hold_samples` | 2 samples | Shutdown/recovery hold |
| `synthetic_fault_thresholds.watchdog_missed_heartbeat_samples` | 3 samples | Timeout boundary |
| `synthetic_fault_thresholds.encoder_stale_samples` | 3 samples | Consecutive no-advance interval boundary |
| `synthetic_fault_thresholds.relay_feedback_mismatch_samples` | 2 samples | Relay mismatch boundary |
| `synthetic_interfaces.encoder_counts_per_revolution` | 4096 counts/revolution | Encoder quantisation fixture |
| Synthetic motor sample period | 0.001 s | Inherited from `SYNTHETIC-DCM-001` |

The validation evidence embeds the full synthetic configuration so that
results cannot be separated from these limitations.

The synthetic watchdog threshold is inclusive: missed-heartbeat samples
`1..N-1` do not fault and sample `N` does. The synthetic relay threshold is
also inclusive: mismatches `1..N-1` inhibit the H-bridge through missing
feedback without latching, and mismatch `N` latches and forces relay command
low in that same sample. Supply voltage exactly at the synthetic undervoltage
threshold is healthy; a finite value below it faults.

Encoder staleness is defined as consecutive sample intervals without a valid,
healthy sequence advance. The first valid sequence only establishes a
baseline at age zero. Each subsequent valid repeated sequence increments age;
age `N-1` is not stale and age `N` latches `ENCODER_STALE`. A valid sequence
change resets age to zero and is the only event that establishes startup
liveness. Failed or explicitly invalid telemetry faults immediately.

## Scenarios

| Scenario | Expected result |
|---|---|
| Normal startup and operation | Safe startup, ready, running with a nonzero command, safe shutdown, ready |
| E-stop activation | `EMERGENCY_STOP` latched; safe output in the detection sample |
| Watchdog timeout | `WATCHDOG_TIMEOUT` after the synthetic missed-sample threshold |
| Stale encoder telemetry | `ENCODER_STALE` after the synthetic stale-sequence threshold |
| Encoder failure | `ENCODER_FAILURE` latched |
| Relay feedback failure | Motor inhibited immediately; `RELAY_FEEDBACK_FAILURE` latched at the synthetic mismatch threshold |
| Undervoltage | `UNDERVOLTAGE` latched |
| Command-voltage saturation | Finite request above the synthetic absolute limit is clipped, saturation telemetry is true, and running continues without a fault |
| Fault latching | Restoring a healthy source does not clear the latch or safe output |
| Rejected unsafe restart | Reset with arm/run asserted is rejected |
| Successful controlled recovery | Source clear, arm/run low, explicit reset, safe shutdown, a complete disarmed READY sample, then a new arm/run sample |

Finite command values inside or exactly at `+/-` the synthetic limit pass
unchanged. Finite values outside it are clipped to the nearer limit and set
`command_saturated=true`; ordinary clipping is diagnostic telemetry and is
not a safety fault. A Boolean, string, `None`, NaN, or infinity command is an
`INVALID_COMMAND`, latches the supervisor, and forces safe outputs. Supply
values must be finite, numeric, non-Boolean, and nonnegative; malformed,
non-finite, or negative values latch `INVALID_SUPPLY_VOLTAGE`. Boolean runtime
fields require actual Boolean values. Configuration numeric fields likewise
reject Booleans, strings, missing values, non-finite values, and invalid
ranges before the bench is constructed.

## Execution and evidence

From the repository root:

```bash
python -m unittest discover -s tests -p "test_*.py" -v
python -m models.validate_sim
```

The validator executes all scenarios twice and compares the complete scenario
summaries and telemetry records for exact equality. It then writes:

- `data/processed/sim_010_synthetic_validation_report.json`
- `data/processed/sim_010_synthetic_scenario_trace.csv`

The generated baseline contains 11 passing scenarios, 85 telemetry records,
13 records with a detected or latched fault, and 7 passing top-level checks.
Every one of the 13 fault-related records has the safe output.

The JSON intentionally omits a generation timestamp and environment-version
fields so unchanged inputs produce byte-stable logical content across
repeated executions. Finite floating telemetry is normalized to 12
significant decimal digits only when serialized to reduce platform-library
representation drift; this is evidence formatting, not fixed-point
conversion. CI regenerates both files and fails if `git diff` detects evidence
drift. The CSV contains one ordered row per scenario sample,
including inputs, state transition, interface observations, detected and
latched faults, reset decision, plant state, and safe-output result.

## Assumptions and limitations

- Execution is synchronous and sample-indexed; it does not model scheduler,
  transport, interrupt, ROS 2 executor, or physical response latency.
- Interface failures are deliberate scripted values, not electrical or
  statistical failure models.
- The relay model represents a feedback-disabled failure. It does not model
  welded contacts or prove removal of motor power.
- The E-stop is a Boolean input. No independent hardwired power-removal path
  is represented.
- The supply monitor uses an ideal sampled scalar and has no ADC accuracy,
  filtering, ripple, wiring-drop, or transient model.
- Encoder staleness and startup liveness are sequence-based. Physical
  transition validity, cable
  faults, and asynchronous sampling remain outside this bench.
- The DC motor remains the existing linear synthetic model. No physical
  friction, backlash, PWM, thermal, current-limit, or supply dynamics are
  validated.
- No safe-state latency in seconds is claimed. The sample thresholds are
  synthetic and must be replaced by approved, source-backed values.
- No physical safety requirement, HIL test, ROS 2 integration test, or FPGA
  fault-manager test is closed by SIM-010.
