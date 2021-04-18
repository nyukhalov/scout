from typing import List
from scout.ros.joystick import DualShockInput
from sensor_msgs.msg import Joy
import unittest


class TestDualShockInput(unittest.TestCase):
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
        ds = DualShockInput()
        ds.handle_message(msg)
        self.assertTrue(ds.is_steering_offset_dec())
        self.assertFalse(ds.is_steering_offset_inc())

    def test_btn_r1_pressed(self):
        msg = self._make_msg_from_btns([0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0])
        ds = DualShockInput()
        ds.handle_message(msg)
        self.assertTrue(ds.is_steering_offset_inc())
        self.assertFalse(ds.is_steering_offset_dec())

    def test_ax_l2_fully_released_when_not_initialized(self):
        msg = self._make_msg_from_axes([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0])
        ds = DualShockInput()
        ds.handle_message(msg)
        self.assertEquals(0.0, ds.braking())

    def test_ax_l2_fully_released(self):
        msg = self._make_msg_from_axes([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -0.0, -0.0])
        ds = DualShockInput()
        ds.handle_message(msg)
        self.assertEquals(0.0, ds.braking())

    def test_ax_l2_fully_pressed(self):
        msg = self._make_msg_from_axes([0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -0.0, -0.0])
        ds = DualShockInput()
        ds.handle_message(msg)
        self.assertEquals(1.0, ds.braking())

    def test_ax_r2_fully_released_when_not_initialized(self):
        msg = self._make_msg_from_axes([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0, -0.0])
        ds = DualShockInput()
        ds.handle_message(msg)
        self.assertEquals(0.0, ds.throttle())

    def test_ax_r2_fully_released(self):
        msg = self._make_msg_from_axes([0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -0.0, -0.0])
        ds = DualShockInput()
        ds.handle_message(msg)
        self.assertEquals(0.0, ds.throttle())

    def test_ax_r2_fully_pressed(self):
        msg = self._make_msg_from_axes([0.0, 0.0, 0.0, 0.0, 0.0, -1.0, -0.0, -0.0])
        ds = DualShockInput()
        ds.handle_message(msg)
        self.assertEquals(1.0, ds.throttle())
