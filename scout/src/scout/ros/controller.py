import time
import traceback
from typing import List, Optional
import json

import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import Joy
from scout.lib.driver import maestro as m
from scout.lib.driver.pwm_controller import PwmController
from scout.ros.config import PwmConfig, ControllerConfig
from scout.ros.joystick import DualShockInput


class RosController:

    @staticmethod
    def make_pwm_controller(servo: m.Controller, config: PwmConfig) -> PwmController:
        controller = PwmController(
            servo,
            channel=config.channel,
            speed=config.speed,
            accel=config.accel,
            min_val=config.min_val,
            max_val=config.max_val
        )
        controller.set_offset(config.offset)
        return controller

    def __init__(self):
        config_file = rospy.get_param("~controller_config", None)
        config = self._load_config(config_file)
        rospy.loginfo(f"Using the configuration: {vars(config)}")

        self.servo = m.Controller(config.device)
        self.throttle_ctrl = self.make_pwm_controller(self.servo, config.throttle)
        self.steering_ctrl = self.make_pwm_controller(self.servo, config.steering)

        self._joy_sub = rospy.Subscriber(
            f"/j0/joy",
            Joy,
            self._on_joy_input
        )

        self._pwm_steering_offset = config.steering.offset

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
                config = ControllerConfig.from_json(config_json["pwm"])
        return config

    def _destroy(self) -> None:
        self.steering_ctrl.set_target_by_factor(0)
        self.throttle_ctrl.set_target_by_factor(0)

    def _on_joy_input(self, msg: Joy) -> None:
        joy_input = DualShockInput(msg)

        if joy_input.is_steering_offset_dec():
            self._pwm_steering_offset -= 1
            rospy.loginfo(f"Setting new PWM steering offset: {self._pwm_steering_offset}")
            self.steering_ctrl.set_offset(self._pwm_steering_offset)
        elif joy_input.is_steering_offset_inc():
            self._pwm_steering_offset += 1
            rospy.loginfo(f"Setting new PWM steering offset: {self._pwm_steering_offset}")
            self.steering_ctrl.set_offset(self._pwm_steering_offset)

        steering_val = joy_input.steering()
        throttle_val = joy_input.throttle()
        reverse_val = joy_input.braking()
        assert -1.0 <= steering_val <= 1.0
        assert 0.0 <= throttle_val <= 1.0
        assert 0.0 <= reverse_val <= 1.0
        self.steering_ctrl.set_target_by_factor(-steering_val)
        self.throttle_ctrl.set_target_by_factor(throttle_val)
        if reverse_val > 0:
            self.throttle_ctrl.set_target_by_factor(-reverse_val)

    def _do_something(self) -> None:
        pass

