import time
import traceback
from typing import List, Optional
import json

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Path
from scout.lib.driver import maestro as m
from scout.lib.driver.pwm_controller import PwmController
from scout.ros.controller.config import PwmConfig, ControllerConfig
from scout.msg import CarControlStamped


class RosControllerNode:

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


        ctrl_topic = rospy.get_param("~car_control_topic", "/car/control")
        controller_config_file = rospy.get_param("~controller_config", None)
        controller_config = self._load_controller_config(controller_config_file)
        rospy.loginfo(f"Controller configuration: {controller_config}")

        self._ctrl_sub = rospy.Subscriber(ctrl_topic, CarControlStamped, self._on_ctrl_msg)

        self.servo = m.Controller(controller_config.device)
        self.throttle_ctrl = self.make_pwm_controller(self.servo, controller_config.throttle)
        self.steering_ctrl = self.make_pwm_controller(self.servo, controller_config.steering)

        self._pub = rospy.Publisher("/debug/steer", Float32, queue_size=1)

    def run(self) -> None:
        rospy.loginfo("Starting node...")
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerror("The node has been interrupted by exception", e)
            traceback.print_exc()
        finally:
            rospy.loginfo("Shutting down...")
            self._destroy()

    def _load_controller_config(self, config_file: Optional[str]) -> ControllerConfig:
        config = ControllerConfig()
        if config_file:
            with open(config_file, "r") as f:
                config_json = json.load(f)
                config = ControllerConfig.from_json(config_json["pwm"])
        return config

    def _destroy(self) -> None:
        #self.steering_ctrl.set_target_by_factor(0)
        self.throttle_ctrl.set_target_by_factor(0)

    def _on_ctrl_msg(self, msg: CarControlStamped) -> None:
        throttle = msg.control.actuators.gas
        brake = msg.control.actuators.brake
        steer = msg.control.actuators.steer
        steer_offset_inc = msg.control.calibration.steer_offset_inc

        debug_msg = Float32()
        debug_msg.data = steer
        self._pub.publish(debug_msg)

        if steer_offset_inc != 0:
            rospy.loginfo(f"Setting new PWM steering offset: {new_offset}")
            new_offset = self.steering_ctrl.get_offset() + steer_offset_inc
            self.steering_ctrl.set_offset(new_offset)

        self.steering_ctrl.set_target_by_factor(steer)
        if brake > 0:
            self.throttle_ctrl.set_target_by_factor(-brake)
        else:
            self.throttle_ctrl.set_target_by_factor(throttle)

