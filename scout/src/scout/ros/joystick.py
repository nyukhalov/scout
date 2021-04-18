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
        pass

    @abc.abstractmethod
    def throttle(self) -> float:
        pass

    @abc.abstractmethod
    def braking(self) -> float:
        pass


class JoystickProfile:

    @staticmethod
    def from_json(json: Dict) -> "JoystickProfile":
        return JoystickProfile(
            json["name"],
            json["btn_x"],
            json["btn_o"],
            json["btn_triangle"],
            json["btn_rect"],
            json["btn_l1"],
            json["btn_r1"],
            json["ax_left_horizontal"],
            json["ax_left_vertical"],
            json["ax_right_horizontal"],
            json["ax_right_vertical"],
            json["ax_l2"],
            json["ax_r2"]
        )

    def __init__(
            self,
            name: str,
            btn_x: int,
            btn_o: int,
            btn_triangle: int,
            btn_rect: int,
            btn_l1: int,
            btn_r1: int,
            ax_left_horizontal: int,
            ax_left_vertical: int,
            ax_right_horizontal: int,
            ax_right_vertical: int,
            ax_l2: int,
            ax_r2: int):
        self.name = name
        self.btn_x = btn_x
        self.btn_o = btn_o
        self.btn_triangle = btn_triangle
        self.btn_rect = btn_rect
        self.btn_l1 = btn_l1
        self.btn_r1 = btn_r1
        self.ax_left_horizontal = ax_left_horizontal
        self.ax_left_vertical = ax_left_vertical
        self.ax_right_horizontal = ax_right_horizontal
        self.ax_right_vertical = ax_right_vertical
        self.ax_l2 = ax_l2
        self.ax_r2 = ax_r2


class DualShockInput(JoystickInput):
    def __init__(self, profile: JoystickProfile):
        self._profile = profile
        self._msg = None
        self._l2_initialized = False
        self._r2_initialized = False

    def handle_message(self, msg: Joy) -> None:
        self._msg = msg

    def is_steering_offset_dec(self) -> bool:
        return self._btn_pressed(self._profile.btn_l1)

    def is_steering_offset_inc(self) -> bool:
        return self._btn_pressed(self._profile.btn_r1)

    def steering(self) -> float:
        return self._ax_value(self._profile.ax_left_horizontal)

    def throttle(self) -> float:
        val = self._ax_value(self._profile.ax_r2)
        assert -1.0 <= val <= 1.0
        if not self._r2_initialized and val != 0.0:
            self._r2_initialized = True
        if self._r2_initialized:
            return (1 - val) / 2 # 0 .. 1
        return 0

    def braking(self) -> float:
        val = self._ax_value(self._profile.ax_l2)
        assert -1.0 <= val <= 1.0
        if not self._l2_initialized and val != 0.0:
            self._l2_initialized = True
        if self._l2_initialized:
            return (1 - val) / 2 # 0 .. 1
        return 0

    def _btn_pressed(self, btn: int) -> bool:
        if self._msg is None:
            raise Exception("Input has not been initialized")
        return self._msg.buttons[btn] > 0

    def _ax_value(self, ax: int) -> float:
        if self._msg is None:
            raise Exception("Input has not been initialized")
        return self._msg.axes[ax]
