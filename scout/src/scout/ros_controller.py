import time
import traceback
from typing import List

import rospy

from nav_msgs.msg import Path


class RosController:
    def __init__(self):
        pass

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
        pass

    def _do_something(self) -> None:
        rospy.loginfo("Test")
        pass
