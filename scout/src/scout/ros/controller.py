import time
import traceback
from typing import List, Optional
import json

import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import Joy
from scout.lib.driver import maestro as m
from scout.lib.driver.pwm_controller import  PwmController


class ControllerConfig:
    def __init__(self):
        self.pwm_device = "/dev/ttyACM0"
        # throttle
        self.pwm_throttle_channel = 5
        self.pwm_throttle_speed = 0
        self.pwm_throttle_accel = 0
        self.pwm_throttle_min_val = 5300
        self.pwm_throttle_max_val = 6500
        # steering
        self.pwm_steering_channel = 0
        self.pwm_steering_speed = 50
        self.pwm_steering_accel = 0
        self.pwm_steering_min_val = 5000
        self.pwm_steering_max_val = 7000


class RosController:
    def __init__(self):
        config_file = rospy.get_param("~controller_config", None)
        config = self._load_config(config_file)
        rospy.loginfo(f"Using the configuration: {vars(config)}")

        self.servo = m.Controller(config.pwm_device)
        self.steering_ctrl = PwmController(
                self.servo,
                channel=config.pwm_steering_channel,
                speed=config.pwm_steering_speed,
                accel=config.pwm_steering_accel,
                min_val=config.pwm_steering_min_val,
                max_val=config.pwm_steering_max_val
        )
        self.throttle_ctrl = PwmController(
                self.servo,
                channel=config.pwm_throttle_channel,
                speed=config.pwm_throttle_speed,
                accel=config.pwm_throttle_accel,
                min_val=config.pwm_throttle_min_val,
                max_val=config.pwm_throttle_max_val
        )
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
                config.pwm_device = str(config_json["pwm"]["device"])
                # throttle
                config.pwm_throttle_channel = int(config_json["pwm"]["throttle"]["channel"])
                config.pwm_throttle_speed = int(config_json["pwm"]["throttle"]["speed"])
                config.pwm_throttle_accel = int(config_json["pwm"]["throttle"]["accel"])
                config.pwm_throttle_min_val = int(config_json["pwm"]["throttle"]["min_val"])
                config.pwm_throttle_max_val = int(config_json["pwm"]["throttle"]["max_val"])
                # steering
                config.pwm_steering_channel = int(config_json["pwm"]["steering"]["channel"])
                config.pwm_steering_speed = int(config_json["pwm"]["steering"]["speed"])
                config.pwm_steering_accel = int(config_json["pwm"]["steering"]["accel"])
                config.pwm_steering_min_val = int(config_json["pwm"]["steering"]["min_val"])
                config.pwm_steering_max_val = int(config_json["pwm"]["steering"]["max_val"])
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

