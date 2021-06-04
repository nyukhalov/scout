from typing import List
from scout.ros.joystick.input import DualShockInput, JoystickProfile
from sensor_msgs.msg import Joy
import json
import os
import unittest

dir_path = os.path.dirname(os.path.realpath(__file__))
config_path = os.path.join(dir_path, "..", "..", "..", "config")

class TestDualShockInput(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestDualShockInput, self).__init__(*args, **kwargs)
        self._profile = None
        with open(os.path.join(config_path, "joystick.json"), "r") as f:
            joystick_config_json = json.load(f)
            for profile_json in joystick_config_json["profiles"]:
                if profile_json["name"] == "DesktopDualShock4":
                    self._profile = JoystickProfile.from_json(profile_json)
        if self._profile is None:
            raise Exception("Unable to find joystick configuration")

    def _make_msg_from_btns(self, btns: List[int]) -> Joy:
        assert len(btns) == 13
        msg = Joy()
        msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0]
        msg.buttons = btns
        return msg

    def _make_msg_from_axes(self, axes: List[float]) -> Joy:
        assert len(axes) == 8
        msg = Joy()
        msg.axes = axes
        msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        return msg

    def test_btn_l1_pressed(self):
        msg = self._make_msg_from_btns([0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
        ds = DualShockInput(self._profile)
        ds.handle_message(msg)
        self.assertTrue(ds.is_steering_offset_dec())
        self.assertFalse(ds.is_steering_offset_inc())

    def test_btn_r1_pressed(self):
        msg = self._make_msg_from_btns([0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0])
        ds = DualShockInput(self._profile)
        ds.handle_message(msg)
        self.assertTrue(ds.is_steering_offset_inc())
        self.assertFalse(ds.is_steering_offset_dec())

    def test_ax_l2_fully_released_when_not_initialized(self):
        msg = self._make_msg_from_axes([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0])
        ds = DualShockInput(self._profile)
        ds.handle_message(msg)
        self.assertEquals(0.0, ds.braking())

    def test_ax_l2_fully_released(self):
        msg = self._make_msg_from_axes([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.0, -0.0])
        ds = DualShockInput(self._profile)
        ds.handle_message(msg)
        self.assertEquals(0.0, ds.braking())

    def test_ax_l2_fully_pressed(self):
        msg = self._make_msg_from_axes([0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -0.0, -0.0])
        ds = DualShockInput(self._profile)
        ds.handle_message(msg)
        self.assertEquals(1.0, ds.braking())

    def test_ax_r2_fully_released_when_not_initialized(self):
        msg = self._make_msg_from_axes([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0])
        ds = DualShockInput(self._profile)
        ds.handle_message(msg)
        self.assertEquals(0.0, ds.throttle())

    def test_ax_r2_fully_released(self):
        msg = self._make_msg_from_axes([0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0])
        ds = DualShockInput(self._profile)
        ds.handle_message(msg)
        self.assertEquals(0.0, ds.throttle())

    def test_ax_r2_fully_pressed(self):
        msg = self._make_msg_from_axes([0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.0, -0.0])
        ds = DualShockInput(self._profile)
        ds.handle_message(msg)
        self.assertEquals(1.0, ds.throttle())
