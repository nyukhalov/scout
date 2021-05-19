import time
import traceback
from typing import List, Optional
import json

import rospy
from nav_msgs.msg import Path
from scout.lib.driver import maestro as m
from scout.lib.driver.pwm_controller import PwmController
from scout.ros.config import PwmConfig, ControllerConfig


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
        rospy.init_node("scout_controller", anonymous=False, log_level=rospy.INFO)
        rospy.loginfo("Initializing node")

        controller_config_file = rospy.get_param("~controller_config", None)
        controller_config = self._load_controller_config(controller_config_file)
        rospy.loginfo(f"Controller configuration: {controller_config}")

        self.servo = m.Controller(controller_config.device)
        self.throttle_ctrl = self.make_pwm_controller(self.servo, controller_config.throttle)
        self.steering_ctrl = self.make_pwm_controller(self.servo, controller_config.steering)

    def run(self) -> None:
        rospy.loginfo("Starting controller")
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

    def _load_controller_config(self, config_file: Optional[str]) -> ControllerConfig:
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
        self._joy_input.handle_message(msg)

        new_offset = None
        if self._joy_input.is_steering_offset_dec():
            new_offset = self.steering_ctrl.get_offset() - 1
        elif self._joy_input.is_steering_offset_inc():
            new_offset = self.steering_ctrl.get_offset() + 1
        if new_offset is not None:
            rospy.loginfo(f"Setting new PWM steering offset: {new_offset}")
            self.steering_ctrl.set_offset(new_offset)

        steering_val = self._joy_input.steering()
        throttle_val = self._joy_input.throttle()
        reverse_val = self._joy_input.braking()
        self.steering_ctrl.set_target_by_factor(-steering_val)
        self.throttle_ctrl.set_target_by_factor(throttle_val)
        if reverse_val > 0:
            self.throttle_ctrl.set_target_by_factor(-reverse_val)

    def _do_something(self) -> None:
        pass

