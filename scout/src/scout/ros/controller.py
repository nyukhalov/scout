import time
import traceback
from typing import List

import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import Joy
from scout.lib.driver import maestro as m


class RosController:
    def __init__(self):
        self._ch_steer = 0
        self._ch_throttle = 5

        self.servo = m.Controller('/dev/ttyACM0')

        # Speed is measured as 0.25microseconds/10milliseconds
        self.servo.setSpeed(self._ch_steer, 50)
        # Valid values are from 0 to 255. 0=unrestricted, 1 is slowest start.
        # A value of 1 will take the servo about 3s to move between 1ms to 2ms range
        self.servo.setAccel(self._ch_steer, 0)

        self.servo.setSpeed(self._ch_throttle, 0)
        self.servo.setAccel(self._ch_throttle, 0)

        self._steering_target = 6000
        self._throttle_target = 6000

        self._joy_sub = rospy.Subscriber(
            f"/j1/joy",
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
        self.servo.setTarget(self._ch_steer, 6000)
        self.servo.setTarget(self._ch_throttle, 6000)

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
            # For servos, target represents the pulse width in of quarter-microseconds
            # Servo center is at 1500 microseconds, or 6000 quarter-microseconds
            # Typcially valid servo range is 3000 to 9000 quarter-microsecond
            assert -1.0 <= self._steering_joy_val <= 1.0
            self._steering_target = int(6000 - self._steering_joy_val * 1000)
            self.servo.setTarget(self._ch_steer, self._steering_target)

        new_throttle_val = msg.axes[ax_r2]
        if self._throttle_joy_val is None or new_throttle_val != self._throttle_joy_val:
            self._throttle_joy_val = new_throttle_val
            if not self._throttle_joy_initialized and self._throttle_joy_val != 0:
                self._throttle_joy_initialized = True

            if self._throttle_joy_initialized:
                assert -1.0 <= self._throttle_joy_val <= 1.0
                factor = (1 - self._throttle_joy_val) / 2 # 0 .. 1
                rospy.loginfo(factor)
                self._throttle_target = int(6000 + factor * 400)
                self.servo.setTarget(self._ch_throttle, self._throttle_target)

    def _do_something(self) -> None:
        pass

