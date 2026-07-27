from types import SimpleNamespace

from agx_arm_ctrl.alignment_diagnostics import (
    FreshnessTracker,
    arm_status_values,
    driver_state_values,
    motor_state_values,
    parse_alignment_sequence,
)


def test_alignment_sequence_is_reserved_and_strict():
    assert parse_alignment_sequence("robot_alignment/sequence/42") == 42
    assert parse_alignment_sequence("robot_alignment/sequence/not-a-number") is None
    assert parse_alignment_sequence("base_link") is None


def test_cached_motor_timestamp_is_not_fresh_and_values_are_preserved():
    tracker = FreshnessTracker()
    sample = SimpleNamespace(
        timestamp=100.5,
        msg=SimpleNamespace(
            position=1.0,
            velocity=2.0,
            current=3.0,
            torque=4.0,
        ),
    )
    first = motor_state_values(4, sample, 10, 20, tracker)
    cached = motor_state_values(4, sample, 11, 21, tracker)
    assert first["sdk_timestamp_ns"] == 100_500_000_000
    assert first["fresh"] is True
    assert cached["fresh"] is False
    assert cached["current_a"] == 3.0
    assert cached["torque_nm"] == 4.0


def test_driver_status_bits_are_preserved_and_decoded():
    tracker = FreshnessTracker()
    sample = SimpleNamespace(
        timestamp=1.0,
        msg=SimpleNamespace(
            vol=48.0,
            bus_current=0.5,
            foc_temp=40.0,
            motor_temp=35.0,
            foc_status_code=(1 << 0) | (1 << 2) | (1 << 6),
        ),
    )
    values = driver_state_values(1, sample, 10, 20, tracker)
    assert values["status_bits"] == 69
    assert values["under_voltage"]
    assert values["driver_over_current"]
    assert values["enabled"]
    assert not values["stall"]


def test_full_arm_status_keeps_per_joint_flags():
    tracker = FreshnessTracker()
    flags = {
        **{f"joint_{index}_angle_limit": index == 3 for index in range(1, 8)},
        **{
            f"communication_status_joint_{index}": index == 6
            for index in range(1, 8)
        },
    }
    sample = SimpleNamespace(
        timestamp=2.0,
        msg=SimpleNamespace(
            ctrl_mode=1,
            arm_status=0,
            mode_feedback=6,
            teach_status=0,
            motion_status=0,
            trajectory_num=0,
            err_code=12,
            err_status=SimpleNamespace(**flags),
        ),
    )
    values = arm_status_values(sample, 10, 20, tracker)
    assert values["valid"]
    assert values["error_code"] == 12
    assert values["joint_angle_limit"][2]
    assert values["communication_status_joint"][5]
