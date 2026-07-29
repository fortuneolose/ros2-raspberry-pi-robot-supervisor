"""Deterministic hardware-independent supervisor software-in-the-loop bench.

SIM-010 uses only sample-indexed simulated interfaces. All configured plant
values, operating limits, and fault thresholds are explicitly synthetic and
must not be interpreted as physical validation or hardware settings.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from numbers import Real
from typing import Any

import numpy as np

from models.dc_motor import StateSpaceModel
from models.robustness import quantize_encoder_position


class SupervisorState(str, Enum):
    """Deterministic supervisor states."""

    SAFE_STARTUP = "SAFE_STARTUP"
    READY = "READY"
    RUNNING = "RUNNING"
    FAULT_LATCHED = "FAULT_LATCHED"
    SAFE_SHUTDOWN = "SAFE_SHUTDOWN"


class FaultCode(str, Enum):
    """Safety faults detected by the synthetic supervisor."""

    EMERGENCY_STOP = "EMERGENCY_STOP"
    WATCHDOG_TIMEOUT = "WATCHDOG_TIMEOUT"
    ENCODER_STALE = "ENCODER_STALE"
    ENCODER_FAILURE = "ENCODER_FAILURE"
    ENCODER_INVALID = "ENCODER_INVALID"
    RELAY_FEEDBACK_FAILURE = "RELAY_FEEDBACK_FAILURE"
    UNDERVOLTAGE = "UNDERVOLTAGE"
    INVALID_SUPPLY_VOLTAGE = "INVALID_SUPPLY_VOLTAGE"
    INVALID_COMMAND = "INVALID_COMMAND"
    INVALID_RUNTIME_INPUT = "INVALID_RUNTIME_INPUT"


_FAULT_ORDER = tuple(FaultCode)


def _ordered_faults(values: set[FaultCode]) -> tuple[FaultCode, ...]:
    return tuple(code for code in _FAULT_ORDER if code in values)


def _evidence_float(value: float) -> float:
    """Normalize finite evidence values to 12 significant decimal digits."""

    return float(format(float(value), ".12g"))


def _strict_real(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a numeric synthetic value")
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise ValueError(
            f"{name} must be a representable synthetic value"
        ) from error
    if not np.isfinite(result):
        raise ValueError(f"{name} must be a finite synthetic value")
    return result


def _synthetic_positive_float(value: Any, name: str) -> float:
    result = _strict_real(value, name)
    if result <= 0.0:
        raise ValueError(f"{name} must be a finite synthetic value above zero")
    return result


def _synthetic_nonnegative_float(value: Any, name: str) -> float:
    result = _strict_real(value, name)
    if result < 0.0:
        raise ValueError(
            f"{name} must be a finite nonnegative synthetic value"
        )
    return result


def _synthetic_positive_int(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{name} must be a positive synthetic integer")
    return value


def _required_mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a configuration object")
    return value


def _required_text(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a nonempty string")
    return value


def _required_value(values: Mapping[str, Any], key: str, name: str) -> Any:
    if key not in values:
        raise ValueError(f"{name}.{key} is required")
    return values[key]


@dataclass(frozen=True)
class SyntheticSilConfig:
    """Explicitly synthetic SIM-010 interface and safety configuration."""

    test_bench_id: str
    status: str
    provenance: str
    synthetic_motor_parameter_file: str
    synthetic_motor_parameter_set_id: str
    synthetic_opposing_load_torque_nm: float
    synthetic_nominal_supply_voltage_v: float
    synthetic_normal_run_command_voltage_v: float
    synthetic_command_voltage_abs_max_v: float
    synthetic_undervoltage_threshold_v: float
    synthetic_safe_startup_healthy_samples: int
    synthetic_safe_shutdown_hold_samples: int
    synthetic_watchdog_missed_heartbeat_samples: int
    synthetic_encoder_stale_samples: int
    synthetic_relay_feedback_mismatch_samples: int
    synthetic_encoder_counts_per_revolution: int
    synthetic_telemetry_channel: str
    synthetic_fault_channel: str
    coefficient_freeze_readiness: str
    fixed_point_conversion_readiness: str

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "SyntheticSilConfig":
        """Build and validate a configuration from the versioned JSON shape."""

        root = _required_mapping(payload, "SIM-010 configuration")
        plant = _required_mapping(
            _required_value(root, "synthetic_plant", "configuration"),
            "synthetic_plant",
        )
        operating = _required_mapping(
            _required_value(
                root,
                "synthetic_operating_values",
                "configuration",
            ),
            "synthetic_operating_values",
        )
        limits = _required_mapping(
            _required_value(
                root,
                "synthetic_operating_limits",
                "configuration",
            ),
            "synthetic_operating_limits",
        )
        thresholds = _required_mapping(
            _required_value(
                root,
                "synthetic_fault_thresholds",
                "configuration",
            ),
            "synthetic_fault_thresholds",
        )
        interfaces = _required_mapping(
            _required_value(
                root,
                "synthetic_interfaces",
                "configuration",
            ),
            "synthetic_interfaces",
        )
        config = cls(
            test_bench_id=_required_text(
                _required_value(root, "test_bench_id", "configuration"),
                "test_bench_id",
            ),
            status=_required_text(
                _required_value(root, "status", "configuration"),
                "status",
            ),
            provenance=_required_text(
                _required_value(root, "provenance", "configuration"),
                "provenance",
            ),
            synthetic_motor_parameter_file=_required_text(
                _required_value(
                    plant,
                    "motor_parameter_file",
                    "synthetic_plant",
                ),
                "synthetic_motor_parameter_file",
            ),
            synthetic_motor_parameter_set_id=_required_text(
                _required_value(
                    plant,
                    "motor_parameter_set_id",
                    "synthetic_plant",
                ),
                "synthetic_motor_parameter_set_id",
            ),
            synthetic_opposing_load_torque_nm=(
                _synthetic_nonnegative_float(
                    _required_value(
                        plant,
                        "opposing_load_torque_nm",
                        "synthetic_plant",
                    ),
                    "synthetic_opposing_load_torque_nm",
                )
            ),
            synthetic_nominal_supply_voltage_v=_synthetic_positive_float(
                _required_value(
                    operating,
                    "nominal_supply_voltage_v",
                    "synthetic_operating_values",
                ),
                "synthetic_nominal_supply_voltage_v",
            ),
            synthetic_normal_run_command_voltage_v=(
                _synthetic_positive_float(
                    _required_value(
                        operating,
                        "normal_run_command_voltage_v",
                        "synthetic_operating_values",
                    ),
                    "synthetic_normal_run_command_voltage_v",
                )
            ),
            synthetic_command_voltage_abs_max_v=_synthetic_positive_float(
                _required_value(
                    limits,
                    "command_voltage_abs_max_v",
                    "synthetic_operating_limits",
                ),
                "synthetic_command_voltage_abs_max_v",
            ),
            synthetic_undervoltage_threshold_v=_synthetic_positive_float(
                _required_value(
                    limits,
                    "undervoltage_threshold_v",
                    "synthetic_operating_limits",
                ),
                "synthetic_undervoltage_threshold_v",
            ),
            synthetic_safe_startup_healthy_samples=_synthetic_positive_int(
                _required_value(
                    thresholds,
                    "safe_startup_healthy_samples",
                    "synthetic_fault_thresholds",
                ),
                "synthetic_safe_startup_healthy_samples",
            ),
            synthetic_safe_shutdown_hold_samples=_synthetic_positive_int(
                _required_value(
                    thresholds,
                    "safe_shutdown_hold_samples",
                    "synthetic_fault_thresholds",
                ),
                "synthetic_safe_shutdown_hold_samples",
            ),
            synthetic_watchdog_missed_heartbeat_samples=(
                _synthetic_positive_int(
                    _required_value(
                        thresholds,
                        "watchdog_missed_heartbeat_samples",
                        "synthetic_fault_thresholds",
                    ),
                    "synthetic_watchdog_missed_heartbeat_samples",
                )
            ),
            synthetic_encoder_stale_samples=_synthetic_positive_int(
                _required_value(
                    thresholds,
                    "encoder_stale_samples",
                    "synthetic_fault_thresholds",
                ),
                "synthetic_encoder_stale_samples",
            ),
            synthetic_relay_feedback_mismatch_samples=(
                _synthetic_positive_int(
                    _required_value(
                        thresholds,
                        "relay_feedback_mismatch_samples",
                        "synthetic_fault_thresholds",
                    ),
                    "synthetic_relay_feedback_mismatch_samples",
                )
            ),
            synthetic_encoder_counts_per_revolution=_synthetic_positive_int(
                _required_value(
                    interfaces,
                    "encoder_counts_per_revolution",
                    "synthetic_interfaces",
                ),
                "synthetic_encoder_counts_per_revolution",
            ),
            synthetic_telemetry_channel=_required_text(
                _required_value(
                    interfaces,
                    "telemetry_channel",
                    "synthetic_interfaces",
                ),
                "synthetic_telemetry_channel",
            ),
            synthetic_fault_channel=_required_text(
                _required_value(
                    interfaces,
                    "fault_channel",
                    "synthetic_interfaces",
                ),
                "synthetic_fault_channel",
            ),
            coefficient_freeze_readiness=_required_text(
                _required_value(
                    root,
                    "coefficient_freeze_readiness",
                    "configuration",
                ),
                "coefficient_freeze_readiness",
            ),
            fixed_point_conversion_readiness=_required_text(
                _required_value(
                    root,
                    "fixed_point_conversion_readiness",
                    "configuration",
                ),
                "fixed_point_conversion_readiness",
            ),
        )
        config.validate()
        return config

    def validate(self) -> None:
        """Reject internally inconsistent synthetic operating values."""

        _required_text(self.test_bench_id, "test_bench_id")
        _required_text(self.status, "status")
        _required_text(self.provenance, "provenance")
        _required_text(
            self.synthetic_motor_parameter_file,
            "synthetic_motor_parameter_file",
        )
        _required_text(
            self.synthetic_motor_parameter_set_id,
            "synthetic_motor_parameter_set_id",
        )
        _synthetic_nonnegative_float(
            self.synthetic_opposing_load_torque_nm,
            "synthetic_opposing_load_torque_nm",
        )
        _synthetic_positive_float(
            self.synthetic_nominal_supply_voltage_v,
            "synthetic_nominal_supply_voltage_v",
        )
        _synthetic_positive_float(
            self.synthetic_normal_run_command_voltage_v,
            "synthetic_normal_run_command_voltage_v",
        )
        _synthetic_positive_float(
            self.synthetic_command_voltage_abs_max_v,
            "synthetic_command_voltage_abs_max_v",
        )
        _synthetic_positive_float(
            self.synthetic_undervoltage_threshold_v,
            "synthetic_undervoltage_threshold_v",
        )
        _synthetic_positive_int(
            self.synthetic_safe_startup_healthy_samples,
            "synthetic_safe_startup_healthy_samples",
        )
        _synthetic_positive_int(
            self.synthetic_safe_shutdown_hold_samples,
            "synthetic_safe_shutdown_hold_samples",
        )
        _synthetic_positive_int(
            self.synthetic_watchdog_missed_heartbeat_samples,
            "synthetic_watchdog_missed_heartbeat_samples",
        )
        _synthetic_positive_int(
            self.synthetic_encoder_stale_samples,
            "synthetic_encoder_stale_samples",
        )
        _synthetic_positive_int(
            self.synthetic_relay_feedback_mismatch_samples,
            "synthetic_relay_feedback_mismatch_samples",
        )
        _synthetic_positive_int(
            self.synthetic_encoder_counts_per_revolution,
            "synthetic_encoder_counts_per_revolution",
        )
        _required_text(
            self.synthetic_telemetry_channel,
            "synthetic_telemetry_channel",
        )
        _required_text(
            self.synthetic_fault_channel,
            "synthetic_fault_channel",
        )
        _required_text(
            self.coefficient_freeze_readiness,
            "coefficient_freeze_readiness",
        )
        _required_text(
            self.fixed_point_conversion_readiness,
            "fixed_point_conversion_readiness",
        )
        if self.status != "synthetic_software_fixture":
            raise ValueError(
                "SIM-010 status must explicitly identify a synthetic fixture"
            )
        if not self.test_bench_id.endswith("-SYNTHETIC"):
            raise ValueError("SIM-010 identifier must explicitly say SYNTHETIC")
        if "physical" not in self.provenance.lower():
            raise ValueError("SIM-010 provenance must state its physical limitation")
        if (
            self.synthetic_undervoltage_threshold_v
            >= self.synthetic_nominal_supply_voltage_v
        ):
            raise ValueError(
                "synthetic undervoltage threshold must be below nominal supply"
            )
        if (
            self.synthetic_normal_run_command_voltage_v
            > self.synthetic_command_voltage_abs_max_v
        ):
            raise ValueError(
                "synthetic normal command must not exceed its synthetic limit"
            )
        if self.synthetic_encoder_counts_per_revolution < 4:
            raise ValueError(
                "synthetic encoder resolution must contain at least four counts"
            )
        if (
            self.synthetic_telemetry_channel
            != "in_memory_ordered_records"
        ):
            raise ValueError("unsupported synthetic telemetry channel")
        if self.synthetic_fault_channel != "in_memory_ordered_events":
            raise ValueError("unsupported synthetic fault channel")
        if self.coefficient_freeze_readiness != "HOLD":
            raise ValueError("SIM-010 must preserve coefficient freeze HOLD")
        if self.fixed_point_conversion_readiness != "HOLD":
            raise ValueError("SIM-010 must preserve fixed-point conversion HOLD")


@dataclass(frozen=True)
class SupervisorInputs:
    """One synchronous set of simulated supervisor inputs."""

    supply_voltage_v: Any
    requested_motor_voltage_v: Any = 0.0
    arm_request: Any = False
    run_request: Any = False
    shutdown_request: Any = False
    reset_request: Any = False
    watchdog_heartbeat: Any = True
    estop_active: Any = False
    encoder_stale: Any = False
    encoder_failed: Any = False
    encoder_invalid: Any = False
    relay_feedback_failed: Any = False


@dataclass(frozen=True)
class _ValidatedInputs:
    supply_voltage_v: float
    requested_motor_voltage_v: float
    arm_request: bool
    run_request: bool
    shutdown_request: bool
    reset_request: bool
    watchdog_heartbeat: bool
    estop_active: bool
    encoder_stale: bool
    encoder_failed: bool
    encoder_invalid: bool
    relay_feedback_failed: bool
    invalid_input_fields: tuple[str, ...]
    invalid_faults: tuple[FaultCode, ...]


@dataclass(frozen=True)
class EncoderTelemetry:
    """One deterministic encoder-channel sample."""

    position_rad: float
    sequence: int
    healthy: bool
    valid: bool


@dataclass(frozen=True)
class FaultEvent:
    """One ordered fault-channel event."""

    sample_index: int
    code: str
    action: str
    state: SupervisorState

    def to_mapping(self) -> dict[str, Any]:
        return {
            "sample_index": self.sample_index,
            "code": self.code,
            "action": self.action,
            "state": self.state.value,
        }


@dataclass(frozen=True)
class TelemetryRecord:
    """One ordered supervisor/plant telemetry-channel record."""

    sample_index: int
    state_before: SupervisorState
    state: SupervisorState
    arm_request: bool
    run_request: bool
    shutdown_request: bool
    reset_request: bool
    estop_active: bool
    watchdog_heartbeat: bool
    watchdog_missed_samples: int
    supply_voltage_v: float
    supply_healthy: bool
    encoder_position_rad: float
    encoder_sequence: int
    encoder_healthy: bool
    encoder_valid: bool
    encoder_liveness_established: bool
    encoder_stale_age_samples: int
    requested_motor_voltage_v: float
    motor_command_v: float
    command_saturated: bool
    relay_enable_command: bool
    relay_enable_feedback: bool
    relay_feedback_mismatch_samples: int
    active_raw_fault_sources: tuple[FaultCode, ...]
    invalid_input_fields: tuple[str, ...]
    detected_faults: tuple[FaultCode, ...]
    latched_faults: tuple[FaultCode, ...]
    reset_accepted: bool
    reset_rejected: bool
    motor_position_rad: float
    motor_speed_rad_s: float
    motor_current_a: float

    @property
    def safe_output(self) -> bool:
        return (
            not self.relay_enable_command
            and self.motor_command_v == 0.0
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "sample_index": self.sample_index,
            "state_before": self.state_before.value,
            "state": self.state.value,
            "arm_request": self.arm_request,
            "run_request": self.run_request,
            "shutdown_request": self.shutdown_request,
            "reset_request": self.reset_request,
            "estop_active": self.estop_active,
            "watchdog_heartbeat": self.watchdog_heartbeat,
            "watchdog_missed_samples": self.watchdog_missed_samples,
            "supply_voltage_v": _evidence_float(self.supply_voltage_v),
            "supply_healthy": self.supply_healthy,
            "encoder_position_rad": _evidence_float(
                self.encoder_position_rad
            ),
            "encoder_sequence": self.encoder_sequence,
            "encoder_healthy": self.encoder_healthy,
            "encoder_valid": self.encoder_valid,
            "encoder_liveness_established": (
                self.encoder_liveness_established
            ),
            "encoder_stale_age_samples": self.encoder_stale_age_samples,
            "requested_motor_voltage_v": _evidence_float(
                self.requested_motor_voltage_v
            ),
            "motor_command_v": _evidence_float(self.motor_command_v),
            "command_saturated": self.command_saturated,
            "relay_enable_command": self.relay_enable_command,
            "relay_enable_feedback": self.relay_enable_feedback,
            "relay_feedback_mismatch_samples": (
                self.relay_feedback_mismatch_samples
            ),
            "active_raw_fault_sources": [
                code.value for code in self.active_raw_fault_sources
            ],
            "invalid_input_fields": list(self.invalid_input_fields),
            "detected_faults": [code.value for code in self.detected_faults],
            "latched_faults": [code.value for code in self.latched_faults],
            "reset_accepted": self.reset_accepted,
            "reset_rejected": self.reset_rejected,
            "motor_position_rad": _evidence_float(
                self.motor_position_rad
            ),
            "motor_speed_rad_s": _evidence_float(
                self.motor_speed_rad_s
            ),
            "motor_current_a": _evidence_float(self.motor_current_a),
            "safe_output": self.safe_output,
        }


_BOOLEAN_INPUT_DEFAULTS: tuple[tuple[str, bool], ...] = (
    ("arm_request", False),
    ("run_request", False),
    ("shutdown_request", False),
    ("reset_request", False),
    ("watchdog_heartbeat", False),
    ("estop_active", True),
    ("encoder_stale", True),
    ("encoder_failed", True),
    ("encoder_invalid", True),
    ("relay_feedback_failed", True),
)


def _validate_runtime_inputs(inputs: Any) -> _ValidatedInputs:
    """Normalize malformed samples and return deterministic fault metadata."""

    invalid_fields: list[str] = []
    invalid_faults: set[FaultCode] = set()
    if not isinstance(inputs, SupervisorInputs):
        return _ValidatedInputs(
            supply_voltage_v=0.0,
            requested_motor_voltage_v=0.0,
            arm_request=False,
            run_request=False,
            shutdown_request=False,
            reset_request=False,
            watchdog_heartbeat=False,
            estop_active=True,
            encoder_stale=True,
            encoder_failed=True,
            encoder_invalid=True,
            relay_feedback_failed=True,
            invalid_input_fields=("inputs",),
            invalid_faults=(FaultCode.INVALID_RUNTIME_INPUT,),
        )

    try:
        supply_value = _strict_real(
            inputs.supply_voltage_v,
            "supply_voltage_v",
        )
    except ValueError:
        supply_value = -1.0
    if supply_value < 0.0:
        invalid_fields.append("supply_voltage_v")
        invalid_faults.add(FaultCode.INVALID_SUPPLY_VOLTAGE)
        supply_value = 0.0

    try:
        command_value = _strict_real(
            inputs.requested_motor_voltage_v,
            "requested_motor_voltage_v",
        )
    except ValueError:
        invalid_fields.append("requested_motor_voltage_v")
        invalid_faults.add(FaultCode.INVALID_COMMAND)
        command_value = 0.0

    boolean_values: dict[str, bool] = {}
    for field_name, safe_default in _BOOLEAN_INPUT_DEFAULTS:
        value = getattr(inputs, field_name)
        if not isinstance(value, bool):
            invalid_fields.append(field_name)
            invalid_faults.add(FaultCode.INVALID_RUNTIME_INPUT)
            boolean_values[field_name] = safe_default
        else:
            boolean_values[field_name] = value

    return _ValidatedInputs(
        supply_voltage_v=supply_value,
        requested_motor_voltage_v=command_value,
        invalid_input_fields=tuple(invalid_fields),
        invalid_faults=_ordered_faults(invalid_faults),
        **boolean_values,
    )


class SimulatedTelemetryChannel:
    """Ordered in-memory telemetry channel."""

    def __init__(self) -> None:
        self.records: list[TelemetryRecord] = []

    def publish(self, record: TelemetryRecord) -> None:
        self.records.append(record)


class SimulatedFaultChannel:
    """Ordered in-memory fault and reset-event channel."""

    def __init__(self) -> None:
        self.events: list[FaultEvent] = []

    def publish(self, event: FaultEvent) -> None:
        self.events.append(event)


class SimulatedDCMotor:
    """Floating-point DC motor driven by the existing discrete model."""

    def __init__(
        self,
        model: StateSpaceModel,
        synthetic_opposing_load_torque_nm: float,
    ) -> None:
        if not isinstance(model, StateSpaceModel):
            raise ValueError("SIM-010 motor model has an invalid type")
        if model.sample_period_s is None:
            raise ValueError("SIM-010 motor model must be discrete")
        _synthetic_positive_float(
            model.sample_period_s,
            "synthetic_motor_sample_period_s",
        )
        matrices = (model.a, model.b, model.c, model.d)
        if any(
            not isinstance(matrix, np.ndarray)
            or not np.all(np.isfinite(matrix))
            for matrix in matrices
        ):
            raise ValueError("SIM-010 motor matrices must be finite arrays")
        if model.a.shape != (3, 3) or model.b.shape != (3, 2):
            raise ValueError("SIM-010 expects the existing three-state motor model")
        if model.c.shape != (1, 3) or model.d.shape != (1, 2):
            raise ValueError("SIM-010 motor output matrices are invalid")
        self.model = model
        self.synthetic_opposing_load_torque_nm = (
            _synthetic_nonnegative_float(
                synthetic_opposing_load_torque_nm,
                "synthetic_opposing_load_torque_nm",
            )
        )
        self.state = np.zeros(3, dtype=np.float64)

    def step(self, motor_command_v: float) -> None:
        if not np.isfinite(motor_command_v):
            raise ValueError("simulated motor command must be finite")
        input_vector = np.array(
            [motor_command_v, self.synthetic_opposing_load_torque_nm],
            dtype=np.float64,
        )
        self.state = self.model.a @ self.state + self.model.b @ input_vector


class SimulatedEncoder:
    """Configurable quantised encoder telemetry interface."""

    def __init__(self, synthetic_counts_per_revolution: int) -> None:
        validated_counts = _synthetic_positive_int(
            synthetic_counts_per_revolution,
            "synthetic_encoder_counts_per_revolution",
        )
        if validated_counts < 4:
            raise ValueError("synthetic encoder must have at least four counts")
        self.synthetic_counts_per_revolution = validated_counts
        self._sequence = 0
        self._last_position_rad = 0.0

    def sample(
        self,
        motor_position_rad: float,
        *,
        stale: bool,
        failed: bool,
        invalid: bool,
    ) -> EncoderTelemetry:
        if not stale and not invalid:
            self._sequence += 1
            self._last_position_rad = quantize_encoder_position(
                motor_position_rad,
                self.synthetic_counts_per_revolution,
            )
        return EncoderTelemetry(
            position_rad=self._last_position_rad,
            sequence=self._sequence,
            healthy=not failed,
            valid=not invalid,
        )


class SimulatedHBridgeCommand:
    """Synthetic H-bridge command limiter and enable gate."""

    def __init__(self, synthetic_abs_limit_v: float) -> None:
        self.synthetic_abs_limit_v = _synthetic_positive_float(
            synthetic_abs_limit_v,
            "synthetic_command_voltage_abs_max_v",
        )

    def apply(
        self,
        requested_voltage_v: float,
        *,
        relay_enable_command: bool,
        relay_enable_feedback: bool,
    ) -> tuple[float, bool]:
        saturated = abs(requested_voltage_v) > self.synthetic_abs_limit_v
        if not relay_enable_command or not relay_enable_feedback:
            return 0.0, saturated
        return (
            float(
                np.clip(
                    requested_voltage_v,
                    -self.synthetic_abs_limit_v,
                    self.synthetic_abs_limit_v,
                )
            ),
            saturated,
        )


class SimulatedRelayEnable:
    """Relay command and synthetic feedback interface."""

    def __init__(self) -> None:
        self.command = False
        self.feedback = False

    def apply(self, command: bool, *, feedback_failed: bool) -> bool:
        self.command = bool(command)
        self.feedback = self.command and not feedback_failed
        return self.feedback


class SimulatedEmergencyStop:
    """Emergency-stop input interface."""

    @staticmethod
    def active(value: bool) -> bool:
        return bool(value)


class SimulatedWatchdog:
    """Missed-heartbeat counter with a synthetic sample threshold."""

    def __init__(self, synthetic_missed_heartbeat_samples: int) -> None:
        self.synthetic_missed_heartbeat_samples = _synthetic_positive_int(
            synthetic_missed_heartbeat_samples,
            "synthetic_watchdog_missed_heartbeat_samples",
        )
        self.missed_samples = 0

    def observe(self, heartbeat: bool) -> bool:
        if heartbeat:
            self.missed_samples = 0
        else:
            self.missed_samples += 1
        return (
            self.missed_samples
            < self.synthetic_missed_heartbeat_samples
        )


class SimulatedSupplyVoltageMonitor:
    """Supply monitor with an explicitly synthetic undervoltage threshold."""

    def __init__(self, synthetic_undervoltage_threshold_v: float) -> None:
        self.synthetic_undervoltage_threshold_v = _synthetic_positive_float(
            synthetic_undervoltage_threshold_v,
            "synthetic_undervoltage_threshold_v",
        )

    def healthy(self, supply_voltage_v: float) -> bool:
        return bool(
            np.isfinite(supply_voltage_v)
            and supply_voltage_v >= self.synthetic_undervoltage_threshold_v
        )


class SupervisorTestBench:
    """Synchronous SIM-010 supervisor and all simulated interfaces."""

    def __init__(
        self,
        config: SyntheticSilConfig,
        motor_model: StateSpaceModel,
    ) -> None:
        config.validate()
        self.config = config
        self.motor = SimulatedDCMotor(
            motor_model,
            config.synthetic_opposing_load_torque_nm,
        )
        self.encoder = SimulatedEncoder(
            config.synthetic_encoder_counts_per_revolution
        )
        self.hbridge = SimulatedHBridgeCommand(
            config.synthetic_command_voltage_abs_max_v
        )
        self.relay = SimulatedRelayEnable()
        self.estop = SimulatedEmergencyStop()
        self.watchdog = SimulatedWatchdog(
            config.synthetic_watchdog_missed_heartbeat_samples
        )
        self.supply_monitor = SimulatedSupplyVoltageMonitor(
            config.synthetic_undervoltage_threshold_v
        )
        self.telemetry = SimulatedTelemetryChannel()
        self.faults = SimulatedFaultChannel()
        self.state = SupervisorState.SAFE_STARTUP
        self.latched_faults: set[FaultCode] = set()
        self.sample_index = 0
        self._startup_healthy_samples = 0
        self._shutdown_hold_samples = 0
        self._last_encoder_sequence: int | None = None
        self._encoder_stale_age_samples = 0
        self._encoder_liveness_established = False
        self._relay_feedback_mismatch_samples = 0
        self._ready_disarmed_sample_seen = False

    def _publish_new_faults(self, faults: set[FaultCode]) -> None:
        for code in _ordered_faults(faults - self.latched_faults):
            self.faults.publish(
                FaultEvent(
                    sample_index=self.sample_index,
                    code=code.value,
                    action="LATCHED",
                    state=SupervisorState.FAULT_LATCHED,
                )
            )
        self.latched_faults.update(faults)

    def _publish_reset_event(self, action: str, state: SupervisorState) -> None:
        self.faults.publish(
            FaultEvent(
                sample_index=self.sample_index,
                code="RESET_SEQUENCE",
                action=action,
                state=state,
            )
        )

    def _update_encoder_age(self, sample: EncoderTelemetry) -> None:
        if not sample.healthy or not sample.valid:
            return
        if self._last_encoder_sequence is None:
            self._encoder_stale_age_samples = 0
        elif sample.sequence != self._last_encoder_sequence:
            self._encoder_stale_age_samples = 0
            self._encoder_liveness_established = True
        else:
            self._encoder_stale_age_samples += 1
        self._last_encoder_sequence = sample.sequence

    def _detect_primary_faults(
        self,
        inputs: _ValidatedInputs,
        encoder_sample: EncoderTelemetry,
        watchdog_healthy: bool,
        supply_healthy: bool,
    ) -> set[FaultCode]:
        detected = set(inputs.invalid_faults)
        if self.estop.active(inputs.estop_active):
            detected.add(FaultCode.EMERGENCY_STOP)
        if not watchdog_healthy:
            detected.add(FaultCode.WATCHDOG_TIMEOUT)
        if not encoder_sample.healthy:
            detected.add(FaultCode.ENCODER_FAILURE)
        if not encoder_sample.valid:
            detected.add(FaultCode.ENCODER_INVALID)
        if (
            self._encoder_stale_age_samples
            >= self.config.synthetic_encoder_stale_samples
        ):
            detected.add(FaultCode.ENCODER_STALE)
        if (
            FaultCode.INVALID_SUPPLY_VOLTAGE not in detected
            and not supply_healthy
        ):
            detected.add(FaultCode.UNDERVOLTAGE)
        return detected

    def _enter_ready(self) -> None:
        self.state = SupervisorState.READY
        self._ready_disarmed_sample_seen = False

    def _advance_nonfault_state(self, inputs: _ValidatedInputs) -> None:
        if self.state == SupervisorState.SAFE_STARTUP:
            if self._encoder_liveness_established:
                self._startup_healthy_samples += 1
            if (
                self._startup_healthy_samples
                >= self.config.synthetic_safe_startup_healthy_samples
            ):
                self._enter_ready()
        elif self.state == SupervisorState.READY:
            if inputs.shutdown_request:
                self.state = SupervisorState.SAFE_SHUTDOWN
                self._shutdown_hold_samples = 0
                self._ready_disarmed_sample_seen = False
            elif not inputs.arm_request and not inputs.run_request:
                self._ready_disarmed_sample_seen = True
            elif (
                self._ready_disarmed_sample_seen
                and inputs.arm_request
                and inputs.run_request
            ):
                self.state = SupervisorState.RUNNING
        elif self.state == SupervisorState.RUNNING:
            if (
                inputs.shutdown_request
                or not inputs.arm_request
                or not inputs.run_request
            ):
                self.state = SupervisorState.SAFE_SHUTDOWN
                self._shutdown_hold_samples = 0
                self._ready_disarmed_sample_seen = False
        elif self.state == SupervisorState.SAFE_SHUTDOWN:
            self._shutdown_hold_samples += 1
            if (
                self._shutdown_hold_samples
                >= self.config.synthetic_safe_shutdown_hold_samples
                and self._encoder_liveness_established
            ):
                self._enter_ready()

    def _process_fault_state(
        self,
        inputs: _ValidatedInputs,
        detected: set[FaultCode],
        active_raw_fault_sources: set[FaultCode],
    ) -> tuple[bool, bool]:
        reset_accepted = False
        reset_rejected = False
        self._publish_new_faults(detected)
        if inputs.reset_request:
            safe_request_state = (
                not inputs.arm_request and not inputs.run_request
            )
            if (
                not detected
                and not active_raw_fault_sources
                and safe_request_state
            ):
                self.latched_faults.clear()
                self.state = SupervisorState.SAFE_SHUTDOWN
                self._shutdown_hold_samples = 0
                self._ready_disarmed_sample_seen = False
                reset_accepted = True
                self._publish_reset_event("ACCEPTED", self.state)
            else:
                reset_rejected = True
                self._publish_reset_event("REJECTED", self.state)
        return reset_accepted, reset_rejected

    def step(self, inputs: SupervisorInputs) -> TelemetryRecord:
        """Advance exactly one synthetic sample and publish ordered evidence."""

        validated = _validate_runtime_inputs(inputs)
        state_before = self.state
        encoder_sample = self.encoder.sample(
            float(self.motor.state[0]),
            stale=validated.encoder_stale,
            failed=validated.encoder_failed,
            invalid=validated.encoder_invalid,
        )
        self._update_encoder_age(encoder_sample)
        watchdog_healthy = self.watchdog.observe(
            validated.watchdog_heartbeat
        )
        supply_healthy = self.supply_monitor.healthy(
            validated.supply_voltage_v
        )
        detected = self._detect_primary_faults(
            validated,
            encoder_sample,
            watchdog_healthy,
            supply_healthy,
        )
        active_raw_fault_sources: set[FaultCode] = set()
        if validated.relay_feedback_failed:
            active_raw_fault_sources.add(
                FaultCode.RELAY_FEEDBACK_FAILURE
            )
        reset_accepted = False
        reset_rejected = False

        if self.state == SupervisorState.FAULT_LATCHED:
            reset_accepted, reset_rejected = self._process_fault_state(
                validated,
                detected,
                active_raw_fault_sources,
            )
        elif detected:
            self._publish_new_faults(detected)
            self.state = SupervisorState.FAULT_LATCHED
        else:
            self._advance_nonfault_state(validated)

        relay_enable_command = self.state == SupervisorState.RUNNING
        relay_enable_feedback = self.relay.apply(
            relay_enable_command,
            feedback_failed=validated.relay_feedback_failed,
        )
        if relay_enable_command != relay_enable_feedback:
            self._relay_feedback_mismatch_samples += 1
        else:
            self._relay_feedback_mismatch_samples = 0

        if (
            self._relay_feedback_mismatch_samples
            >= self.config.synthetic_relay_feedback_mismatch_samples
        ):
            detected.add(FaultCode.RELAY_FEEDBACK_FAILURE)
            self._publish_new_faults(
                {FaultCode.RELAY_FEEDBACK_FAILURE}
            )
            self.state = SupervisorState.FAULT_LATCHED
            relay_enable_command = False
            relay_enable_feedback = self.relay.apply(
                False,
                feedback_failed=validated.relay_feedback_failed,
            )

        motor_command_v, hbridge_saturated = self.hbridge.apply(
            validated.requested_motor_voltage_v,
            relay_enable_command=relay_enable_command,
            relay_enable_feedback=relay_enable_feedback,
        )
        if detected or self.state != SupervisorState.RUNNING:
            motor_command_v = 0.0
            relay_enable_command = False
            relay_enable_feedback = self.relay.apply(
                False,
                feedback_failed=validated.relay_feedback_failed,
            )
        self.motor.step(motor_command_v)

        record = TelemetryRecord(
            sample_index=self.sample_index,
            state_before=state_before,
            state=self.state,
            arm_request=validated.arm_request,
            run_request=validated.run_request,
            shutdown_request=validated.shutdown_request,
            reset_request=validated.reset_request,
            estop_active=validated.estop_active,
            watchdog_heartbeat=validated.watchdog_heartbeat,
            watchdog_missed_samples=self.watchdog.missed_samples,
            supply_voltage_v=validated.supply_voltage_v,
            supply_healthy=supply_healthy,
            encoder_position_rad=encoder_sample.position_rad,
            encoder_sequence=encoder_sample.sequence,
            encoder_healthy=encoder_sample.healthy,
            encoder_valid=encoder_sample.valid,
            encoder_liveness_established=(
                self._encoder_liveness_established
            ),
            encoder_stale_age_samples=self._encoder_stale_age_samples,
            requested_motor_voltage_v=(
                validated.requested_motor_voltage_v
            ),
            motor_command_v=motor_command_v,
            command_saturated=hbridge_saturated,
            relay_enable_command=relay_enable_command,
            relay_enable_feedback=relay_enable_feedback,
            relay_feedback_mismatch_samples=(
                self._relay_feedback_mismatch_samples
            ),
            active_raw_fault_sources=_ordered_faults(
                active_raw_fault_sources
            ),
            invalid_input_fields=validated.invalid_input_fields,
            detected_faults=_ordered_faults(detected),
            latched_faults=_ordered_faults(self.latched_faults),
            reset_accepted=reset_accepted,
            reset_rejected=reset_rejected,
            motor_position_rad=float(self.motor.state[0]),
            motor_speed_rad_s=float(self.motor.state[1]),
            motor_current_a=float(self.motor.state[2]),
        )
        if detected and not record.safe_output:
            raise RuntimeError("detected SIM-010 fault did not force safe output")
        if self.state == SupervisorState.FAULT_LATCHED and not record.safe_output:
            raise RuntimeError("latched SIM-010 fault did not preserve safe output")
        self.telemetry.publish(record)
        self.sample_index += 1
        return record
