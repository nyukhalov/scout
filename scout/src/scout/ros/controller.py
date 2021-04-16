import time
import traceback
from typing import List, Optional
import json

import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import Joy
from scout.lib.driver import maestro as m
from scout.lib.driver.pwm_controller import PwmController


class PwmConfig:
    @staticmethod
    def from_json(json: str) -> "PwmConfig":
        channel = int(json["channel"])
        speed = int(json["speed"])
        accel = int(json["accel"])
        min_val = int(json["min_val"])
        max_val = int(json["max_val"])
        return PwmConfig(channel, speed, accel, min_val, max_val)

    def __init__(self, channel: int, speed: int, accel: int, min_val: int, max_val: int):
        self.channel = channel
        self.speed = speed
        self.accel = accel
        self.min_val = min_val
        self.max_val = max_val


class ControllerConfig:
    @staticmethod
    def from_json(json: str) -> "ControllerConfig":
        config = ControllerConfig()
        throttle_cfg_json = json["throttle"]
        steering_cfg_json = json["steering"]
        config.pwm_device = str(json["device"])
        config.throttle = PwmConfig.from_json(throttle_cfg_json)
        config.steering = PwmConfig.from_json(steering_cfg_json)
        return config

    def __init__(self):
        # default values
        self.pwm_device = "/dev/ttyACM0"
        self.throttle = PwmConfig(channel=5, speed=0, accel=0, min_val=5300, max_val=6500)
        self.steering = PwmConfig(channel=0, speed=50, accel=0, min_val=5000, max_val=7000)


class RosController:

    @staticmethod
    def make_pwm_controller(servo: m.Controller, config: PwmConfig) -> PwmController:
        return PwmController(
            servo,
            channel=config.channel,
            speed=config.speed,
            accel=config.accel,
            min_val=config.min_val,
            max_val=config.max_val
        )

    def __init__(self):
        config_file = rospy.get_param("~controller_config", None)
        config = self._load_config(config_file)
        rospy.loginfo(f"Using the configuration: {vars(config)}")

        self.servo = m.Controller(config.pwm_device)
        self.steering_ctrl = self.make_pwm_controller(self.servo, config.steering)
        self.throttle_ctrl = self.make_pwm_controller(self.servo, config.throttle)

        self._joy_sub = rospy.Subscriber(
            f"/j0/joy",
            Joy,
            self._on_joy_input
        )

        self._steering_joy_val = None
        self._throttle_joy_initialized = False
        self._throttle_joy_val = None
        self._reverse_joy_initialized = False
        self._reverse_joy_val = None

    def run(self) -> None:
        rospy.loginfo(f"Starting controller")
        try:
            rate = rospy.Rate(1)  # ROS Rate at 1Hz
            while not rospy.is_shutdown():
                self._do_something()
                rate.sleep()
        except Exception as e:
            rospy.logerror('The node has been interrupted by exception', e)
            traceback.print_exc()
        finally:
            self._destroy()

    def _load_config(self, config_file: Optional[str]) -> ControllerConfig:
        config = ControllerConfig()
        if config_file:
            with open(config_file, "r") as f:
                config_json = json.load(f)
                config = ControllerConfig.from_json(config_json("pwm"))
        return config

    def _destroy(self) -> None:
        self.steering_ctrl.set_target_by_factor(0)
        self.throttle_ctrl.set_target_by_factor(0)

    def _on_joy_input(self, msg: Joy) -> None:
        ax_left_left_right = 0
        ax_left_up_down = 1
        ax_right_left_right = 2
        ax_l2 = 3
        ax_r2 = 4
        ax_right_up_down = 5

        btn_rect = 0
        btn_x = 1
        btn_circle = 2
        btn_triangle = 3
        btn_l1 = 4
        btn_r1 = 5

        new_steering_val = msg.axes[ax_left_left_right]
        if self._steering_joy_val is None or new_steering_val != self._steering_joy_val:
            self._steering_joy_val = new_steering_val
            assert -1.0 <= self._steering_joy_val <= 1.0
            self.steering_ctrl.set_target_by_factor(-self._steering_joy_val)

        new_throttle_val = msg.axes[ax_r2]
        if self._throttle_joy_val is None or new_throttle_val != self._throttle_joy_val:
            self._throttle_joy_val = new_throttle_val
            if not self._throttle_joy_initialized and self._throttle_joy_val != 0:
                self._throttle_joy_initialized = True

            if self._throttle_joy_initialized:
                assert -1.0 <= self._throttle_joy_val <= 1.0
                factor = (1 - self._throttle_joy_val) / 2 # 0 .. 1
                self.throttle_ctrl.set_target_by_factor(factor)

        new_reverse_val = msg.axes[ax_l2]
        if self._reverse_joy_val is None or new_reverse_val != self._reverse_joy_val:
            self._reverse_joy_val = new_reverse_val
            if not self._reverse_joy_initialized and self._reverse_joy_val != 0:
                self._reverse_joy_initialized = True

            if self._reverse_joy_initialized:
                assert -1.0 <= self._reverse_joy_val <= 1.0
                factor = (1 - self._reverse_joy_val) / 2 # 0 .. 1
                self.throttle_ctrl.set_target_by_factor(-factor)

    def _do_something(self) -> None:
        pass

