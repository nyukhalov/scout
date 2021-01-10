import time
import traceback
from typing import List

import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import Joy
from scout.lib.driver import maestro as m
from scout.lib.driver.pwm_controller import  PwmController


class RosController:
    def __init__(self):
        self.servo = m.Controller('/dev/ttyACM0')
        self.steering_ctrl = PwmController(self.servo, channel=0, speed=50, accel=0, min_val=5000, max_val=7000)
        self.throttle_ctrl = PwmController(self.servo, channel=5, speed=0, accel=0, min_val=5000, max_val=6400)

        self._steering_target = 6000
        self._throttle_target = 6000

        self._joy_sub = rospy.Subscriber(
            f"/j0/joy",
            Joy,
            self._on_joy_input
        )

        self._steering_joy_val = None
        self._throttle_joy_initialized = False
        self._throttle_joy_val = None

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

    def _destroy(self) -> None:
        self.steering_ctrl.set_target_by_factor(0)
        self.throttle_ctrl.set_target_by_factor(0)

    def _on_joy_input(self, msg: Joy) -> None:
        #rospy.loginfo(msg.buttons)
        #rospy.loginfo(msg.axes)

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

    def _do_something(self) -> None:
        pass

