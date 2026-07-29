"""
ROS-independent transport adaptation around authoritative SIM-010.

This module does not contain a safety state machine. It validates transport
ordering/freshness, maps message-neutral frames into ``SupervisorInputs``,
steps ``SupervisorTestBench``, and maps the resulting ``TelemetryRecord``.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

from models.sil import (
    SupervisorInputs,
    SupervisorState,
    SupervisorTestBench,
    SyntheticSilConfig,
    TelemetryRecord,
)
from models.validate_sim import build_motor_model


ACCEPTED = 'ACCEPTED'
DUPLICATE = 'DUPLICATE'
OUT_OF_ORDER = 'OUT_OF_ORDER'
MALFORMED = 'MALFORMED'
MISSING = 'MISSING'
STALE = 'STALE'


@dataclass(frozen=True)
class CommandFrame:
    """Message-neutral command sample owned by the command publisher."""

    sequence: Any
    arm_request: Any
    run_request: Any
    shutdown_request: Any
    requested_motor_voltage_v: Any


@dataclass(frozen=True)
class EncoderFrame:
    """Message-neutral external encoder sample."""

    sequence: Any
    position_rad: Any
    healthy: Any
    valid: Any


@dataclass(frozen=True)
class SafetyFrame:
    """Message-neutral synthetic safety-interface sample."""

    sequence: Any
    supply_voltage_v: Any
    software_estop_active: Any
    watchdog_heartbeat: Any
    relay_feedback_enabled: Any
    relay_feedback_healthy: Any


@dataclass(frozen=True)
class AdapterResult:
    """One authoritative SIM-010 record plus ROS transport diagnostics."""

    record: TelemetryRecord
    inputs: SupervisorInputs
    command_disposition: str
    encoder_disposition: str
    safety_disposition: str
    input_diagnostics: tuple[str, ...]


def _valid_sequence(value: Any) -> bool:
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and 0 <= value <= (2**64 - 1)
    )


def _load_object(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(payload, dict):
        raise ValueError(f'{path} must contain a JSON object')
    return payload


class SupervisorAdapter:
    """Validate ROS transport state and invoke SIM-010 exactly once per tick."""

    def __init__(
        self,
        config: SyntheticSilConfig,
        motor_model: Any,
        *,
        command_stale_samples: int | None = None,
    ) -> None:
        self.config = config
        self.bench = SupervisorTestBench(config, motor_model)
        stale_samples = (
            config.synthetic_watchdog_missed_heartbeat_samples
            if command_stale_samples is None
            else command_stale_samples
        )
        if (
            isinstance(stale_samples, bool)
            or not isinstance(stale_samples, int)
            or stale_samples < 1
        ):
            raise ValueError('command_stale_samples must be a positive integer')
        self.command_stale_samples = stale_samples

        self._command: CommandFrame | None = None
        self._encoder: EncoderFrame | None = None
        self._safety: SafetyFrame | None = None
        self._last_sequences: dict[str, int | None] = {
            'command': None,
            'encoder': None,
            'safety': None,
        }
        self._received_at_sample: dict[str, int | None] = {
            'command': None,
            'encoder': None,
            'safety': None,
        }
        self._updated_since_tick = {
            'command': False,
            'encoder': False,
            'safety': False,
        }
        self._pending_disposition: dict[str, str] = {
            'command': '',
            'encoder': '',
            'safety': '',
        }
        self._protocol_invalid = {
            'command': False,
            'encoder': False,
            'safety': False,
        }
        self._last_relay_enable = False

    @classmethod
    def from_files(
        cls,
        sim_config_path: Path,
        motor_config_path: Path,
        *,
        command_stale_samples: int | None = None,
    ) -> 'SupervisorAdapter':
        sim_payload = _load_object(sim_config_path)
        motor_payload = _load_object(motor_config_path)
        config = SyntheticSilConfig.from_mapping(sim_payload)
        if (
            motor_payload.get('parameter_set_id')
            != config.synthetic_motor_parameter_set_id
        ):
            raise ValueError('ROS2-010 synthetic motor parameter-set mismatch')
        return cls(
            config,
            build_motor_model(motor_payload),
            command_stale_samples=command_stale_samples,
        )

    def _classify_sequence(self, channel: str, sequence: Any) -> str:
        if not _valid_sequence(sequence):
            self._protocol_invalid[channel] = True
            return MALFORMED
        previous = self._last_sequences[channel]
        if previous is not None:
            if sequence == previous:
                return DUPLICATE
            if sequence < previous:
                self._protocol_invalid[channel] = True
                return OUT_OF_ORDER
        self._last_sequences[channel] = sequence
        self._received_at_sample[channel] = self.bench.sample_index
        self._updated_since_tick[channel] = True
        return ACCEPTED

    def _ingest(self, channel: str, frame: Any) -> str:
        disposition = self._classify_sequence(channel, frame.sequence)
        self._pending_disposition[channel] = disposition
        if disposition == ACCEPTED:
            setattr(self, f'_{channel}', frame)
        return disposition

    def ingest_command(self, frame: CommandFrame) -> str:
        if not isinstance(frame, CommandFrame):
            self._protocol_invalid['command'] = True
            self._pending_disposition['command'] = MALFORMED
            return MALFORMED
        return self._ingest('command', frame)

    def ingest_encoder(self, frame: EncoderFrame) -> str:
        if not isinstance(frame, EncoderFrame):
            self._protocol_invalid['encoder'] = True
            self._pending_disposition['encoder'] = MALFORMED
            return MALFORMED
        return self._ingest('encoder', frame)

    def ingest_safety(self, frame: SafetyFrame) -> str:
        if not isinstance(frame, SafetyFrame):
            self._protocol_invalid['safety'] = True
            self._pending_disposition['safety'] = MALFORMED
            return MALFORMED
        return self._ingest('safety', frame)

    def _age(self, channel: str) -> int | None:
        received = self._received_at_sample[channel]
        if received is None:
            return None
        return self.bench.sample_index - received

    @property
    def initial_telemetry_received(self) -> bool:
        """Both required telemetry owners have made first contact."""
        return self._encoder is not None and self._safety is not None

    def _disposition(self, channel: str) -> str:
        pending = self._pending_disposition[channel]
        if pending:
            return pending
        if self._last_sequences[channel] is None:
            return MISSING
        return STALE

    def _build_inputs(self, *, reset_request: bool) -> tuple[
        SupervisorInputs, tuple[str, ...]
    ]:
        diagnostics: list[str] = []

        command = self._command
        command_age = self._age('command')
        command_stale = (
            command_age is not None
            and command_age >= self.command_stale_samples
        )
        if command is None:
            arm_request: Any = False
            run_request: Any = False
            shutdown_request: Any = False
            requested_voltage: Any = 0.0
            diagnostics.append('command_missing_safe_default')
        elif command_stale:
            arm_request = False
            run_request = False
            shutdown_request = True
            requested_voltage = 0.0
            diagnostics.append('command_stale_safe_shutdown')
        else:
            arm_request = command.arm_request
            run_request = command.run_request
            shutdown_request = command.shutdown_request
            requested_voltage = command.requested_motor_voltage_v
        if self._protocol_invalid['command']:
            requested_voltage = math.nan
            diagnostics.append('command_protocol_invalid')

        encoder = self._encoder
        encoder_fresh = self._updated_since_tick['encoder']
        encoder_failed: Any = False
        encoder_invalid: Any = False
        if encoder is None:
            diagnostics.append('encoder_missing')
        else:
            encoder_failed = (
                not encoder.healthy
                if isinstance(encoder.healthy, bool)
                else encoder.healthy
            )
            finite_position = (
                isinstance(encoder.position_rad, (int, float))
                and not isinstance(encoder.position_rad, bool)
                and math.isfinite(float(encoder.position_rad))
            )
            encoder_invalid = (
                (not encoder.valid)
                if isinstance(encoder.valid, bool)
                else encoder.valid
            )
            encoder_invalid = bool(encoder_invalid or not finite_position)
            if not finite_position:
                diagnostics.append('encoder_position_nonfinite_or_malformed')
        if self._protocol_invalid['encoder']:
            encoder_invalid = True
            diagnostics.append('encoder_protocol_invalid')
        encoder_stale = not encoder_fresh
        if encoder_stale:
            diagnostics.append('encoder_no_accepted_advance_this_tick')

        safety = self._safety
        safety_fresh = self._updated_since_tick['safety']
        if safety is None:
            supply_voltage: Any = (
                self.config.synthetic_nominal_supply_voltage_v
            )
            estop_active: Any = False
            heartbeat: Any = False
            relay_feedback_failed: Any = False
            diagnostics.append('safety_input_missing_safe_default')
        else:
            supply_voltage = safety.supply_voltage_v
            estop_active = safety.software_estop_active
            heartbeat = (
                safety.watchdog_heartbeat if safety_fresh else False
            )
            if not safety_fresh:
                diagnostics.append('heartbeat_not_refreshed')
            if (
                isinstance(safety.relay_feedback_healthy, bool)
                and isinstance(safety.relay_feedback_enabled, bool)
            ):
                relay_feedback_failed = (
                    not safety.relay_feedback_healthy
                    or safety.relay_feedback_enabled
                    != self._last_relay_enable
                )
            else:
                relay_feedback_failed = safety.relay_feedback_healthy
        if self._protocol_invalid['safety']:
            supply_voltage = math.nan
            diagnostics.append('safety_protocol_invalid')

        return (
            SupervisorInputs(
                supply_voltage_v=supply_voltage,
                requested_motor_voltage_v=requested_voltage,
                arm_request=arm_request,
                run_request=run_request,
                shutdown_request=shutdown_request,
                reset_request=reset_request,
                watchdog_heartbeat=heartbeat,
                estop_active=estop_active,
                encoder_stale=encoder_stale,
                encoder_failed=encoder_failed,
                encoder_invalid=encoder_invalid,
                relay_feedback_failed=relay_feedback_failed,
            ),
            tuple(diagnostics),
        )

    def tick(self, *, reset_request: bool = False) -> AdapterResult:
        inputs, diagnostics = self._build_inputs(
            reset_request=reset_request
        )
        dispositions = {
            channel: self._disposition(channel)
            for channel in ('command', 'encoder', 'safety')
        }
        record = self.bench.step(inputs)
        self._last_relay_enable = record.relay_enable_command
        for channel in self._updated_since_tick:
            self._updated_since_tick[channel] = False
            self._pending_disposition[channel] = ''
            self._protocol_invalid[channel] = False
        return AdapterResult(
            record=record,
            inputs=inputs,
            command_disposition=dispositions['command'],
            encoder_disposition=dispositions['encoder'],
            safety_disposition=dispositions['safety'],
            input_diagnostics=diagnostics,
        )

    def reset_tick(self) -> tuple[AdapterResult, bool, str]:
        """Execute one authoritative SIM-010 tick containing reset_request."""
        was_faulted = self.bench.state == SupervisorState.FAULT_LATCHED
        result = self.tick(reset_request=True)
        record = result.record
        if record.reset_accepted:
            return result, True, 'accepted by SIM-010; entered safe shutdown'
        if record.reset_rejected:
            reasons: list[str] = []
            if result.inputs.arm_request or result.inputs.run_request:
                reasons.append('arm_or_run_active')
            if record.detected_faults:
                reasons.append('detected_fault_active')
            if record.active_raw_fault_sources:
                reasons.append('raw_fault_source_active')
            return (
                result,
                False,
                'rejected by SIM-010: '
                + (','.join(reasons) if reasons else 'unsafe_state'),
            )
        if not was_faulted:
            return result, False, 'rejected: supervisor is not fault latched'
        return result, False, 'rejected by SIM-010'


def actuator_mapping(
    record: TelemetryRecord,
    *,
    reason: str = 'sim_010_tick',
) -> dict[str, Any]:
    return {
        'sample_index': record.sample_index,
        'relay_enable': record.relay_enable_command,
        'motor_voltage_v': record.motor_command_v,
        'saturated': record.command_saturated,
        'state': record.state.value,
        'reason': reason,
    }


def safe_actuator_mapping(
    sample_index: int,
    *,
    state: str,
    reason: str,
) -> dict[str, Any]:
    return {
        'sample_index': sample_index,
        'relay_enable': False,
        'motor_voltage_v': 0.0,
        'saturated': False,
        'state': state,
        'reason': reason,
    }


def safety_status_mapping(record: TelemetryRecord) -> dict[str, Any]:
    return {
        'sample_index': record.sample_index,
        'state': record.state.value,
        'safe_output': record.safe_output,
        'encoder_liveness_established': (
            record.encoder_liveness_established
        ),
        'encoder_stale_age_samples': record.encoder_stale_age_samples,
        'watchdog_missed_samples': record.watchdog_missed_samples,
        'relay_feedback_mismatch_samples': (
            record.relay_feedback_mismatch_samples
        ),
        'supply_healthy': record.supply_healthy,
        'reset_accepted': record.reset_accepted,
        'reset_rejected': record.reset_rejected,
    }


def supervisor_telemetry_mapping(result: AdapterResult) -> dict[str, Any]:
    record = result.record
    return {
        'sample_index': record.sample_index,
        'state_before': record.state_before.value,
        'state': record.state.value,
        'requested_motor_voltage_v': record.requested_motor_voltage_v,
        'motor_command_v': record.motor_command_v,
        'command_saturated': record.command_saturated,
        'relay_enable_command': record.relay_enable_command,
        'relay_enable_feedback': record.relay_enable_feedback,
        'encoder_position_rad': record.encoder_position_rad,
        'encoder_sequence': record.encoder_sequence,
        'command_disposition': result.command_disposition,
        'encoder_disposition': result.encoder_disposition,
        'safety_disposition': result.safety_disposition,
    }


def fault_telemetry_mapping(
    result: AdapterResult,
    *,
    reset_reason: str = '',
) -> dict[str, Any]:
    record = result.record
    return {
        'sample_index': record.sample_index,
        'detected_faults': [
            fault.value for fault in record.detected_faults
        ],
        'latched_faults': [fault.value for fault in record.latched_faults],
        'active_raw_fault_sources': [
            fault.value for fault in record.active_raw_fault_sources
        ],
        'input_diagnostics': list(result.input_diagnostics),
        'reset_accepted': record.reset_accepted,
        'reset_rejected': record.reset_rejected,
        'reset_reason': reset_reason,
    }
