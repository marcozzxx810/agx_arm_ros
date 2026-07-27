"""Pure helpers for passive robot-alignment diagnostics.

This module deliberately has no ROS imports so it can be tested with mocked SDK
objects without creating a ROS graph or touching a CAN device.
"""

from __future__ import annotations

import math
from typing import Any


SEQUENCE_FRAME_PREFIX = "robot_alignment/sequence/"

REJECTION_NONE = 0
REJECTION_NOT_READY = 1
REJECTION_NOT_ENABLED = 2
REJECTION_GATE_CLOSED = 3
REJECTION_TEACH_MODE = 4
REJECTION_INVALID_COMMAND = 5
REJECTION_SDK_ERROR = 6


def parse_alignment_sequence(frame_id: object) -> int | None:
    """Return the reserved alignment sequence ID or None for normal commands."""

    text = str(frame_id)
    if not text.startswith(SEQUENCE_FRAME_PREFIX):
        return None
    suffix = text[len(SEQUENCE_FRAME_PREFIX) :]
    if not suffix or not suffix.isdecimal():
        return None
    value = int(suffix)
    return value if 0 <= value < 2**64 else None


def seconds_to_nanoseconds(value: object) -> int:
    numeric = float(value)
    if not math.isfinite(numeric) or numeric <= 0.0:
        return -1
    return int(round(numeric * 1_000_000_000))


class FreshnessTracker:
    """Track source timestamps without treating cached reads as new samples."""

    def __init__(self) -> None:
        self._last: dict[object, int] = {}

    def observe(self, source: object, timestamp_ns: int, *, valid: bool) -> bool:
        if not valid or timestamp_ns <= 0:
            return False
        previous = self._last.get(source)
        self._last[source] = int(timestamp_ns)
        return previous is None or previous != timestamp_ns


def _finite(value: object) -> float:
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError("SDK diagnostic value is not finite")
    return numeric


def motor_state_values(
    joint_index: int,
    sample: Any,
    receipt_ros_ns: int,
    receipt_monotonic_ns: int,
    freshness: FreshnessTracker,
) -> dict[str, Any]:
    result = {
        "joint_index": int(joint_index),
        "sdk_timestamp_ns": -1,
        "driver_receipt_ros_timestamp_ns": int(receipt_ros_ns),
        "driver_receipt_monotonic_ns": int(receipt_monotonic_ns),
        "position_rad": math.nan,
        "velocity_rad_s": math.nan,
        "current_a": math.nan,
        "torque_nm": math.nan,
        "fresh": False,
        "valid": False,
    }
    if sample is None:
        return result
    try:
        timestamp_ns = seconds_to_nanoseconds(sample.timestamp)
        values = (
            _finite(sample.msg.position),
            _finite(sample.msg.velocity),
            _finite(sample.msg.current),
            _finite(sample.msg.torque),
        )
        valid = timestamp_ns > 0
    except (AttributeError, TypeError, ValueError):
        return result
    result.update(
        {
            "sdk_timestamp_ns": timestamp_ns,
            "position_rad": values[0],
            "velocity_rad_s": values[1],
            "current_a": values[2],
            "torque_nm": values[3],
            "fresh": freshness.observe(
                ("motor", int(joint_index)), timestamp_ns, valid=valid
            ),
            "valid": valid,
        }
    )
    return result


def joint_angle_values(
    sample: Any,
    receipt_ros_ns: int,
    receipt_monotonic_ns: int,
    freshness: FreshnessTracker,
) -> dict[str, Any]:
    result = {
        "source_timestamp_ns": -1,
        "receipt_ros_timestamp_ns": int(receipt_ros_ns),
        "receipt_monotonic_ns": int(receipt_monotonic_ns),
        "qpos_rad": [math.nan] * 7,
        "fresh": False,
        "valid": False,
    }
    if sample is None:
        return result
    try:
        timestamp_ns = seconds_to_nanoseconds(sample.timestamp)
        qpos = [_finite(value) for value in sample.msg]
        valid = timestamp_ns > 0 and len(qpos) == 7
    except (AttributeError, TypeError, ValueError):
        return result
    if not valid:
        return result
    result.update(
        {
            "source_timestamp_ns": timestamp_ns,
            "qpos_rad": qpos,
            "fresh": freshness.observe("joint_angle", timestamp_ns, valid=True),
            "valid": True,
        }
    )
    return result


_DRIVER_FLAGS = (
    "voltage_too_low",
    "motor_overheating",
    "driver_overcurrent",
    "driver_overheating",
    "collision_status",
    "driver_error_status",
    "driver_enable_status",
    "stall_status",
)


def _driver_status_bits(message: Any) -> int:
    code = getattr(message, "foc_status_code", None)
    if code is not None:
        return int(code) & 0xFF
    status = message.foc_status
    bits = 0
    for bit, name in enumerate(_DRIVER_FLAGS):
        if bool(getattr(status, name)):
            bits |= 1 << bit
    return bits


def driver_state_values(
    joint_index: int,
    sample: Any,
    receipt_ros_ns: int,
    receipt_monotonic_ns: int,
    freshness: FreshnessTracker,
) -> dict[str, Any]:
    result = {
        "joint_index": int(joint_index),
        "sdk_timestamp_ns": -1,
        "receipt_ros_timestamp_ns": int(receipt_ros_ns),
        "receipt_monotonic_ns": int(receipt_monotonic_ns),
        "voltage_v": math.nan,
        "bus_current_a": math.nan,
        "foc_temperature_c": math.nan,
        "motor_temperature_c": math.nan,
        "status_bits": 0,
        "under_voltage": False,
        "motor_over_temperature": False,
        "driver_over_current": False,
        "driver_over_temperature": False,
        "collision": False,
        "driver_error": False,
        "enabled": False,
        "stall": False,
        "fresh": False,
        "valid": False,
    }
    if sample is None:
        return result
    try:
        timestamp_ns = seconds_to_nanoseconds(sample.timestamp)
        message = sample.msg
        bits = _driver_status_bits(message)
        numeric = (
            _finite(message.vol),
            _finite(message.bus_current),
            _finite(message.foc_temp),
            _finite(message.motor_temp),
        )
        valid = timestamp_ns > 0
    except (AttributeError, TypeError, ValueError):
        return result
    result.update(
        {
            "sdk_timestamp_ns": timestamp_ns,
            "voltage_v": numeric[0],
            "bus_current_a": numeric[1],
            "foc_temperature_c": numeric[2],
            "motor_temperature_c": numeric[3],
            "status_bits": bits,
            "under_voltage": bool(bits & (1 << 0)),
            "motor_over_temperature": bool(bits & (1 << 1)),
            "driver_over_current": bool(bits & (1 << 2)),
            "driver_over_temperature": bool(bits & (1 << 3)),
            "collision": bool(bits & (1 << 4)),
            "driver_error": bool(bits & (1 << 5)),
            "enabled": bool(bits & (1 << 6)),
            "stall": bool(bits & (1 << 7)),
            "fresh": freshness.observe(
                ("driver", int(joint_index)), timestamp_ns, valid=valid
            ),
            "valid": valid,
        }
    )
    return result


def arm_status_values(
    sample: Any,
    receipt_ros_ns: int,
    receipt_monotonic_ns: int,
    freshness: FreshnessTracker,
) -> dict[str, Any]:
    result = {
        "source_timestamp_ns": -1,
        "receipt_ros_timestamp_ns": int(receipt_ros_ns),
        "receipt_monotonic_ns": int(receipt_monotonic_ns),
        "ctrl_mode": 0,
        "arm_status": 0,
        "mode_feedback": 0,
        "teach_status": 0,
        "motion_status": 0,
        "trajectory_num": 0,
        "error_code": 0,
        "joint_angle_limit": [False] * 7,
        "communication_status_joint": [False] * 7,
        "fresh": False,
        "valid": False,
    }
    if sample is None:
        return result
    try:
        timestamp_ns = seconds_to_nanoseconds(sample.timestamp)
        message = sample.msg
        error = message.err_status
        limits = [
            bool(getattr(error, f"joint_{index}_angle_limit"))
            for index in range(1, 8)
        ]
        communication = [
            bool(getattr(error, f"communication_status_joint_{index}"))
            for index in range(1, 8)
        ]
        error_code = int(getattr(message, "err_code", 0))
        valid = timestamp_ns > 0
    except (AttributeError, TypeError, ValueError):
        return result
    result.update(
        {
            "source_timestamp_ns": timestamp_ns,
            "ctrl_mode": int(message.ctrl_mode),
            "arm_status": int(message.arm_status),
            "mode_feedback": int(message.mode_feedback),
            "teach_status": int(message.teach_status),
            "motion_status": int(message.motion_status),
            "trajectory_num": int(message.trajectory_num),
            "error_code": error_code,
            "joint_angle_limit": limits,
            "communication_status_joint": communication,
            "fresh": freshness.observe("arm_status", timestamp_ns, valid=valid),
            "valid": valid,
        }
    )
    return result
