from types import SimpleNamespace

from agx_arm_ctrl import agx_arm_ctrl_single_node as driver_module
from agx_arm_ctrl.agx_arm_ctrl_single_node import AgxArmRosNode


class _Logger:
    def __init__(self):
        self.messages = []

    def warn(self, message):
        self.messages.append(("warn", message))

    def info(self, message):
        self.messages.append(("info", message))


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
        "1.25-beta": driver_module.NeroFW.V120,
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

    AgxArmRosNode._configure_physical_mode(subject)

    assert events == ["set_follower_mode", "get_arm_status"]
