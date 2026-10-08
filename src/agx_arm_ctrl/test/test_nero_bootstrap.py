from types import SimpleNamespace

import pytest

from agx_arm_ctrl import agx_arm_ctrl_single_node as driver_module
from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


class _Logger:
    def __init__(self):
        self.messages = []

    def warn(self, message):
        self.messages.append(("warn", message))

    def info(self, message):
        self.messages.append(("info", message))

    def error(self, message):
        self.messages.append(("error", message))


def test_persisted_leader_recovery_uses_only_mode_and_enable(monkeypatch):
    events = []

    class OldArm:
        def disconnect(self):
            events.append("disconnect_old")

    class RecoveryArm:
        def connect(self):
            events.append("connect_recovery")

        def set_follower_mode(self):
            events.append("set_follower_mode")

        def get_arm_status(self):
            events.append("get_arm_status")
            return SimpleNamespace(msg=SimpleNamespace(ctrl_mode=1))

    recovery_arm = RecoveryArm()
    expected_config = {"firmeware_version": "v112"}

    def create_config(**kwargs):
        events.append(("create_config", kwargs))
        assert kwargs["firmeware_version"] == driver_module.NeroFW.V112
        return expected_config

    monkeypatch.setattr(driver_module, "create_agx_arm_config", create_config)
    monkeypatch.setattr(
        driver_module.AgxArmFactory,
        "create_arm",
        lambda config: recovery_arm,
    )

    subject = SimpleNamespace(
        arm_type="nero",
        can_port="can0",
        enable_timeout=5.0,
        agx_arm=OldArm(),
        get_logger=lambda: _Logger(),
    )

    def enable(enable, timeout):
        events.append(("enable", enable, timeout))
        return True

    subject._enable_arm = enable

    result = AgxArmRosNode._recover_persisted_nero_leader(subject)

    assert result is expected_config
    assert events == [
        "disconnect_old",
        (
            "create_config",
            {
                "robot": "nero",
                "comm": "can",
                "channel": "can0",
                "firmeware_version": driver_module.NeroFW.V112,
            },
        ),
        "connect_recovery",
        "set_follower_mode",
        ("enable", True, 5.0),
        "set_follower_mode",
        "get_arm_status",
    ]


def test_nero_firmware_driver_mapping():
    subject = SimpleNamespace()

    cases = {
        "1.10": driver_module.NeroFW.DEFAULT,
        "1.11": driver_module.NeroFW.V111,
        "1.12": driver_module.NeroFW.V112,
        "1.20": driver_module.NeroFW.V120,
        "1.21": driver_module.NeroFW.V121,
        "1.22": driver_module.NeroFW.V121,
        "1.25-beta": driver_module.NeroFW.V121,
    }
    for firmware, expected_driver in cases.items():
        assert AgxArmRosNode._nero_firmware_driver(
            subject, firmware
        ) == expected_driver


def test_leader_mode_is_verified_by_leader_joint_stream():
    events = []

    class Arm:
        def set_leader_mode(self):
            events.append("set_leader_mode")

        def get_leader_joint_angles(self):
            events.append("get_leader_joint_angles")
            return SimpleNamespace(hz=200.0)

        def get_arm_status(self):
            raise AssertionError("leader mode disables normal status push")

    subject = SimpleNamespace(
        physical_mode="leader",
        enable_flag=True,
        enable_timeout=0.1,
        agx_arm=Arm(),
        get_logger=lambda: _Logger(),
    )

    AgxArmRosNode._configure_physical_mode(subject)

    assert events == ["set_leader_mode", "get_leader_joint_angles"]


def test_follower_mode_is_verified_by_control_status():
    events = []

    class Arm:
        def set_follower_mode(self):
            events.append("set_follower_mode")

        def get_arm_status(self):
            events.append("get_arm_status")
            return SimpleNamespace(msg=SimpleNamespace(ctrl_mode=1))

    subject = SimpleNamespace(
        physical_mode="follower",
        enable_flag=True,
        enable_timeout=0.1,
        agx_arm=Arm(),
        get_logger=lambda: _Logger(),
    )

    def enable(enable, timeout):
        events.append(("enable", enable, timeout))
        return True

    subject._enable_arm = enable

    AgxArmRosNode._configure_physical_mode(subject)

    assert events == [
        "set_follower_mode",
        ("enable", True, 0.1),
        "get_arm_status",
    ]


def test_leader_gripper_setup_enters_confirmed_follower_mode():
    events = []

    statuses = iter([
        SimpleNamespace(timestamp=10.0),
        SimpleNamespace(hz=200.0, timestamp=10.0, msg=SimpleNamespace(ctrl_mode=1)),
        SimpleNamespace(hz=200.0, timestamp=11.0, msg=SimpleNamespace(ctrl_mode=1)),
    ])

    class Arm:
        def set_follower_mode(self):
            events.append("set_follower_mode")

        def get_arm_status(self):
            events.append("get_arm_status")
            return next(statuses)

    subject = SimpleNamespace(
        agx_arm=Arm(),
        enable_timeout=0.1,
        get_logger=lambda: _Logger(),
    )

    def enable(enable, timeout):
        events.append(("enable", enable, timeout))
        return True

    subject._enable_arm = enable

    AgxArmRosNode._enter_follower_mode_for_leader_gripper_setup(subject)

    assert events == [
        "get_arm_status",
        "set_follower_mode",
        ("enable", True, 0.1),
        "get_arm_status",
        "get_arm_status",
    ]


def test_leader_gripper_range_is_preserved_when_already_configured():
    events = []
    gripper = SimpleNamespace(
        get_teaching_param=lambda **kwargs: (
            events.append(("get", kwargs))
            or SimpleNamespace(
                teaching_range_per=100,
                max_range_config=0.1,
                teaching_friction=3,
            )
        ),
        set_teaching_param=lambda **kwargs: events.append(("set", kwargs)),
    )
    subject = SimpleNamespace(
        is_nero=True,
        physical_mode="leader",
        leader_gripper_max_range_m=0.1,
        leader_gripper_teaching_friction=0,
        gripper=gripper,
        enable_timeout=5.0,
        leader_gripper_reset_on_start=False,
        get_logger=lambda: _Logger(),
    )

    AgxArmRosNode._prepare_leader_gripper(subject)

    assert events == [("get", {"timeout": 2.0, "min_interval": 0.0})]


def test_leader_gripper_range_is_restored_before_leader_mode():
    events = []
    gripper = SimpleNamespace(
        get_teaching_param=lambda **kwargs: SimpleNamespace(
            teaching_range_per=150,
            max_range_config=0.0,
            teaching_friction=7,
        ),
        set_teaching_param=lambda **kwargs: events.append(kwargs) or True,
    )
    subject = SimpleNamespace(
        is_nero=True,
        physical_mode="leader",
        leader_gripper_max_range_m=0.1,
        leader_gripper_teaching_friction=0,
        gripper=gripper,
        enable_timeout=5.0,
        leader_gripper_reset_on_start=False,
        get_logger=lambda: _Logger(),
    )

    AgxArmRosNode._prepare_leader_gripper(subject)

    assert events == [
        {
            "teaching_range_per": 150,
            "max_range_config": 0.1,
            "teaching_friction": 7,
            "timeout": 2.0,
        }
    ]


def test_leader_gripper_teaching_friction_is_restored():
    events = []
    gripper = SimpleNamespace(
        get_teaching_param=lambda **kwargs: SimpleNamespace(
            teaching_range_per=100,
            max_range_config=0.1,
            teaching_friction=1,
        ),
        set_teaching_param=lambda **kwargs: events.append(kwargs) or True,
    )
    subject = SimpleNamespace(
        is_nero=True,
        physical_mode="leader",
        leader_gripper_max_range_m=0.1,
        leader_gripper_teaching_friction=10,
        leader_gripper_reset_on_start=False,
        gripper=gripper,
        enable_timeout=5.0,
        get_logger=lambda: _Logger(),
    )

    AgxArmRosNode._prepare_leader_gripper(subject)

    assert events == [
        {
            "teaching_range_per": 100,
            "max_range_config": 0.1,
            "teaching_friction": 10,
            "timeout": 2.0,
        }
    ]


def test_leader_gripper_parameter_readback_accepts_missing_legacy_ack():
    events = []
    teaching = iter(
        [
            SimpleNamespace(
                teaching_range_per=100,
                max_range_config=0.1,
                teaching_friction=1,
            ),
            SimpleNamespace(
                teaching_range_per=100,
                max_range_config=0.1,
                teaching_friction=10,
            ),
        ]
    )
    gripper = SimpleNamespace(
        get_teaching_param=lambda **kwargs: next(teaching),
        set_teaching_param=lambda **kwargs: events.append(kwargs) or False,
    )
    logger = _Logger()
    subject = SimpleNamespace(
        is_nero=True,
        physical_mode="leader",
        leader_gripper_max_range_m=0.1,
        leader_gripper_teaching_friction=10,
        leader_gripper_reset_on_start=False,
        gripper=gripper,
        enable_timeout=5.0,
        get_logger=lambda: logger,
    )

    AgxArmRosNode._prepare_leader_gripper(subject)

    assert len(events) == 1
    assert any(
        level == "warn" and "verified by read-back" in message
        for level, message in logger.messages
    )


def test_leader_gripper_reset_waits_for_disabled_feedback():
    events = []
    statuses = iter(
        [
            None,
            SimpleNamespace(
                hz=200.0,
                timestamp=10.0,
                driver_enable_status=True,
                voltage_too_low=False,
                motor_overheating=False,
                driver_overcurrent=False,
                driver_overheating=False,
                sensor_status=False,
                driver_error_status=False,
                width=0.042,
            ),
            # A cached disabled frame must not complete reset.
            SimpleNamespace(
                hz=200.0,
                timestamp=10.0,
                driver_enable_status=False,
            ),
            SimpleNamespace(
                hz=200.0,
                timestamp=11.0,
                driver_enable_status=False,
            ),
            # A cached disabled frame must not complete re-enable.
            SimpleNamespace(
                hz=200.0,
                timestamp=11.0,
                driver_enable_status=False,
            ),
            SimpleNamespace(
                hz=200.0,
                timestamp=12.0,
                driver_enable_status=True,
            ),
        ]
    )
    gripper = SimpleNamespace(
        get_status=lambda: next(statuses),
        reset=lambda: events.append("reset"),
        move=lambda **kwargs: events.append(("move", kwargs)) or True,
    )
    logger = _Logger()
    subject = SimpleNamespace(
        gripper=gripper,
        enable_timeout=5.0,
        gripper_default_effort=1.0,
        get_logger=lambda: logger,
    )

    AgxArmRosNode._reset_leader_gripper_before_mode_switch(subject)

    assert events == [
        "reset",
        ("move", {"width": 0.042, "force": 1.0}),
    ]
    assert any(
        level == "info" and "reset and re-enabled" in message
        for level, message in logger.messages
    )


@pytest.mark.parametrize("width", [float("nan"), float("inf"), -float("inf")])
def test_leader_gripper_reset_rejects_invalid_width_without_commands(width):
    status = SimpleNamespace(hz=200.0, width=width, voltage_too_low=False,
        motor_overheating=False, driver_overcurrent=False, driver_overheating=False,
        sensor_status=False, driver_error_status=False)
    def unexpected_command(*args, **kwargs):
        raise AssertionError("invalid feedback must not trigger gripper commands")
    subject = SimpleNamespace(enable_timeout=0.1, gripper=SimpleNamespace(
        get_status=lambda: status, reset=unexpected_command, move=unexpected_command))
    with pytest.raises(RuntimeError, match="width is invalid"):
        AgxArmRosNode._reset_leader_gripper_before_mode_switch(subject)
