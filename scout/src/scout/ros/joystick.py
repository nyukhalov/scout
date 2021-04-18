import abc
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


class DualShockInput(JoystickInput):
    # button indices
    BTN_X = 0
    BTN_O = 1
    BTN_TRIANGLE = 2
    BTN_RECT = 3
    BTN_L1 = 4
    BTN_R1 = 5

    # axes indices
    AX_LEFT_LEFT_RIGHT = 0
    AX_LEFT_UP_DOWN = 1
    AX_L2 = 2
    AX_RIGHT_LEFT_RIGHT = 3
    AX_RIGHT_UP_DOWN = 4
    AX_R2 = 5

    def __init__(self):
        self._msg = None
        self._l2_initialized = False
        self._r2_initialized = False

    def handle_message(self, msg: Joy) -> None:
        self._msg = msg

    def is_steering_offset_dec(self) -> bool:
        return self._btn_pressed(self.BTN_L1)

    def is_steering_offset_inc(self) -> bool:
        return self._btn_pressed(self.BTN_R1)

    def steering(self) -> float:
        return self._ax_value(self.AX_LEFT_LEFT_RIGHT)

    def throttle(self) -> float:
        val = self._ax_value(self.AX_R2)
        assert -1.0 <= val <= 1.0
        if not self._r2_initialized and val != 0.0:
            self._r2_initialized = True
        if self._r2_initialized:
            return (1 - val) / 2 # 0 .. 1
        return 0

    def braking(self) -> float:
        val = self._ax_value(self.AX_L2)
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
