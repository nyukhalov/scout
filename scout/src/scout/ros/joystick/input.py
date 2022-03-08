import abc
from typing import Dict
from sensor_msgs.msg import Joy


class JoystickInput:
    @abc.abstractmethod
    def handle_message(self, msg: Joy) -> None:
        pass

    @abc.abstractmethod
    def is_steering_offset_dec(self) -> bool:
        pass

    @abc.abstractmethod
    def is_steering_offset_inc(self) -> bool:
        pass

    @abc.abstractmethod
    def steering(self) -> float:
        """Return the steering input in the range [0..1]"""
        pass

    @abc.abstractmethod
    def throttle(self) -> float:
        """Return the throttle input in the range [0..1]"""
        pass

    @abc.abstractmethod
    def braking(self) -> float:
        """Return the braking input in the range [0..1]"""
        pass

    @abc.abstractmethod
    def is_activate_auto_pressed(self) -> bool:
        """Return true if the activate auto button is pressed"""
        pass


class JoystickProfile:

    @staticmethod
    def from_json(json: Dict) -> "JoystickProfile":
        return JoystickProfile(
            json["name"],
            json["btn_down"],
            json["btn_right"],
            json["btn_top"],
            json["btn_left"],
            json["left_bumper"],
            json["right_bumper"],
            json["lsb_horizontal"],
            json["lsb_vertical"],
            json["rsb_horizontal"],
            json["rsb_vertical"],
            json["left_trigger"],
            json["right_trigger"]
        )

    def __init__(
            self,
            name: str,
            btn_down: int,
            btn_right: int,
            btn_top: int,
            btn_left: int,
            left_bumper: int,
            right_bumper: int,
            lsb_horizontal: int,
            lsb_vertical: int,
            rsb_horizontal: int,
            rsb_vertical: int,
            left_trigger: int,
            right_trigger: int):
        self.name = name
        self.btn_down = btn_down
        self.btn_right = btn_right
        self.btn_top = btn_top
        self.btn_left = btn_left
        self.left_bumper = left_bumper
        self.right_bumper = right_bumper
        self.lsb_horizontal = lsb_horizontal
        self.lsb_vertical = lsb_vertical
        self.rsb_horizontal = rsb_horizontal
        self.rsb_vertical = rsb_vertical
        self.left_trigger = left_trigger
        self.right_trigger = right_trigger

    def __str__(self) -> str:
        return str(vars(self))

    def __repr__(self) -> str:
        return self.__str__()


class DualShockInput(JoystickInput):
    def __init__(self, profile: JoystickProfile):
        self._profile = profile
        self._msg = None
        self._prev_buttons = None
        self._left_trigger_initialized = False
        self._right_trigger_initialized = False

    def handle_message(self, msg: Joy) -> None:
        if self._msg:
            self._prev_buttons = self._msg.buttons
        self._msg = msg

    def is_steering_offset_dec(self) -> bool:
        btn = self._profile.left_bumper
        return self._btn_updated(btn) and self._btn_pressed(btn)

    def is_steering_offset_inc(self) -> bool:
        btn = self._profile.right_bumper
        return self._btn_updated(btn) and self._btn_pressed(btn)

    def steering(self) -> float:
        return -self._ax_value(self._profile.lsb_horizontal)

    def throttle(self) -> float:
        val = self._ax_value(self._profile.right_trigger)
        assert -1.0 <= val <= 1.0
        if not self._right_trigger_initialized and val != 0.0:
            self._right_trigger_initialized = True
        if self._right_trigger_initialized:
            return (1 - val) / 2  # 0 .. 1
        return 0

    def braking(self) -> float:
        val = self._ax_value(self._profile.left_trigger)
        assert -1.0 <= val <= 1.0
        if not self._left_trigger_initialized and val != 0.0:
            self._left_trigger_initialized = True
        if self._left_trigger_initialized:
            return (1 - val) / 2  # 0 .. 1
        return 0

    def is_activate_auto_pressed(self) -> bool:
        return self._btn_pressed(self._profile.btn_top)

    def _btn_pressed(self, btn: int) -> bool:
        if self._msg is None:
            raise Exception("Input has not been initialized")
        return self._msg.buttons[btn] > 0

    def _btn_updated(self, btn: int) -> bool:
        if not self._prev_buttons:
            return True
        return self._prev_buttons[btn] != self._msg.buttons[btn]

    def _ax_value(self, ax: int) -> float:
        if self._msg is None:
            raise Exception("Input has not been initialized")
        return self._msg.axes[ax]
