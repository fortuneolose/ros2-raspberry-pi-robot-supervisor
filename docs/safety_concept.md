# Safety Concept

## 1. Scope

This concept covers the single-axis bench demonstrator from logic interfaces
through the motor power stage and guarded rotating load. It is an engineering
baseline and does not replace university laboratory approval, manufacturer
instructions, or a task-specific risk assessment.

## 2. Safe state

The defined safe state is:

- PWM output inactive;
- motor-driver enable inactive;
- commanded torque/current effectively zero;
- fault and diagnostic information retained where power permits; and
- no automatic restart when reset or communications return.

Loss of Raspberry Pi software, a stale serial command, or an invalid controller
calculation must lead to or maintain this state.

## 3. Principal hazards and controls

| Hazard | Initial severity | Preventive controls | Detection / protective response |
|---|---|---|---|
| Motor or driver overcurrent/overheating | High | Driver rated from stall current, fuse, current-limited supply, conservative command limits, suitable conductors | Driver fault/current monitoring where available; FPGA disables and latches fault |
| Contact with rotating parts | High | Rigid mount, guard, controlled access, low-energy commissioning | E-stop; immediate driver disable |
| Unexpected start/restart | High | Safe pull states, explicit arm sequence, reset-to-disarmed, run command limits | Hardware enable gating; latched fault; manual re-arm |
| Pi/FPGA I/O damage | Medium | Verify 3.3 V compatibility, series resistors, keyed connectors, separate 5 V rails | Power-off continuity/polarity checks; interface test before connection |
| Fixed-point overflow or unstable control | High | Range analysis, quantised pole check, wide accumulators, saturation, conservative gains | Sticky arithmetic flags; FPGA disables under approved policy |
| Encoder loss or invalid transitions | High | Conditioned/synchronised inputs, robust connectors, guarded cable routing | Plausibility/transition monitoring; fault and safe state |
| Communications loss/corruption | Medium/High | Versioned framed protocol, integrity check, sequence count | FPGA watchdog and Pi protocol monitor; actuator disable |
| Wiring or PCB error | High | Design review, DRC/ERC, labelled/keyed connectors, independent power bring-up | Power-off inspection; current-limited first power; test points |

## 4. Safety authority and interlock

Controller arithmetic provides only a requested actuator command. The final
hardware enable is:

```text
safe_to_drive =
    reset_released
    AND coefficients_valid
    AND protocol_compatible
    AND arm_request
    AND watchdog_healthy
    AND encoder_healthy
    AND estop_inactive
    AND no_latched_fault
```

`safe_to_drive == false` forces PWM and driver enable inactive independently of
the requested duty cycle. The emergency-stop path and driver characteristics
must be reviewed to determine whether additional hardwired removal of motor
power is required.

## 5. Fault policy

| Fault | Detection owner | Minimum response | Recovery |
|---|---|---|---|
| Emergency stop | Hardware/FPGA | Immediate safe state | Release E-stop, inspect, explicit fault reset and re-arm |
| Communications timeout | FPGA watchdog | Safe state within approved timeout | Restore valid sequence, explicit reset/re-arm |
| Pi process loss | FPGA watchdog | Same as communications timeout | Restart software, inspect logs, explicit reset/re-arm |
| Invalid encoder behaviour | FPGA | Safe state or controlled stop as hazard analysis approves | Correct cause, validate sensor, explicit reset/re-arm |
| Arithmetic overflow | FPGA datapath/fault manager | Saturate and flag; latch safe state where control validity is compromised | Review scaling/cause, explicit reset/re-arm |
| Driver fault | Driver/FPGA input | Disable command and latch status | Remove cause and follow driver procedure |
| Protocol/version/integrity error | Both endpoints | Reject frame; watchdog eventually disables if valid frames stop | Restore compatible software, explicit reset/re-arm if tripped |

Exact response times are open verification parameters. They must be frozen
before powered acceptance tests.

## 6. Power and grounding rules

- Use separate, appropriately rated supplies for the Pi, Basys 3, and motor
  power stage; do not parallel Pi and FPGA 5 V rails.
- Share only the required signal reference for the proposed non-isolated
  interface.
- Return motor current directly through the motor driver and motor supply.
- Do not place motor/battery/H-bridge current on a solderless breadboard.
- Fuse the motor power path and use current limiting during commissioning.
- Select wire, connectors, PCB copper, and the driver from worst-case current
  and thermal conditions, including stall.

## 7. Commissioning gates

Motor power must not advance to the next gate until evidence for the current
gate is reviewed.

1. Mechanical mount and guard inspection.
2. Power-off continuity, polarity, connector orientation, and safe-default
   checks.
3. Separate Pi, FPGA, and driver power-up with the motor disconnected.
4. Logic levels, reset states, E-stop, driver-enable, and watchdog checks.
5. UART/SPI loopback and corrupt/stale packet testing.
6. Synthetic encoder stimulation and invalid-transition testing.
7. Low-duty, current-limited open-loop direction/encoder verification.
8. Observer operation with actuation disabled.
9. Conservative closed-loop operation within documented limits.
10. Fault injection followed by comparative experiments.

Each powered test plan records maximum supply voltage/current, command/duty,
speed, test duration, stop conditions, operator, observer, and emergency
procedure.

## 8. Open safety decisions

- Exact motor, driver, power-supply, fuse, wiring, and connector ratings.
- Whether the physical E-stop removes motor power, driver enable, or both.
- Watchdog interval and maximum measured disable latency.
- Encoder plausibility thresholds and controlled-stop versus immediate-disable
  policy.
- Arithmetic events that warn versus events that latch safe state.
- Guard design and the permitted operating envelope.
- Laboratory review, supervision, and sign-off requirements.
