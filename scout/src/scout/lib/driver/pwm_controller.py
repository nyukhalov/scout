from scout.lib.driver import maestro as m


class PwmController:
    PWM_MIN = 3000
    PWM_MAX = 9000
    PWM_MID = 6000

    def __init__(self, ctrl: m.Controller, channel: int, speed: int, accel: int, min_val: int, max_val: int):
        assert ctrl is not None
        assert 0 <= channel <= 5, f"channel value {channel} is out of range"
        assert 0 <= speed, f"speed value {speed} is out of range"
        assert 0 <= accel <= 255, f"accel value {accel} is out of range"
        assert self.PWM_MIN <= min_val <= self.PWM_MID, f"min_val value {min_val} is out of range"
        assert self.PWM_MID <= max_val <= self.PWM_MAX, f"max_val value {max_val} is out of range"

        self.ctrl = ctrl
        self.channel = channel
        self.min_val = min_val
        self.max_val = max_val

        # Speed is measured as 0.25microseconds/10milliseconds
        self.ctrl.setSpeed(channel, speed)
        # Valid values are from 0 to 255. 0=unrestricted, 1 is slowest start.
        # A value of 1 will take the servo about 3s to move between 1ms to 2ms range
        self.ctrl.setAccel(channel, accel)

    def set_target_by_factor(self, factor: float) -> None:
        assert -1.0 <= factor <= 1.0, f"factor value {factor} is out of range"
        max_offset = (self.max_val - self.PWM_MID) if factor >= 0 else (self.PWM_MID - self.min_val)
        target = self.PWM_MID + int(factor * max_offset)
        self.set_target(target)

    def set_target(self, target: int) -> None:
        assert self.PWM_MIN <= target <= self.PWM_MAX, f"target value {target} is out of range"
        # For servos, target represents the pulse width in of quarter-microseconds
        # Servo center is at 1500 microseconds, or 6000 quarter-microseconds
        # Typcially valid servo range is 3000 to 9000 quarter-microsecond
        self.ctrl.setTarget(self.channel, target)
