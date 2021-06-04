import rospy
from sensor_msgs.msg import LaserScan


class RosWallFollowerNode:
    def __init__(self):
        rospy.init_node("scout_wall_follower", anonymous=False, log_level=rospy.INFO)
        rospy.loginfo("Initializing node")

        scan_topic = rospy.get_param("~scan_topic", "/scan")
        self._scan_sub = rospy.Subscriber(scan_topic, LaserScan, self._on_scan_msg)

    def run(self) -> None:
        rospy.loginfo("Starting node...")
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerror("The node has been interrupted by exception", e)
            traceback.print_exc()
        finally:
            rospy.loginfo("Shutting down...")

    def _on_scan_msg(self, msg: LaserScan) -> None:
        # TODO: normalize all angles to [-PI, PI]
        num_rays = len(msg.ranges)
        assert num_rays == int((msg.angle_max - msg.angle_min) / msg.angle_increment)

        target_baselink_angle = -3.14 / 2  # -90 degrees
        target_angle = ???
        idx = (target_angle - msg.angle_min) / msg.angle_increment
        target_range = msg.ranges[idx]

        for i, r in enumerate(msg.ranges):
            if msg.range_min <= r <= msg.range_max:
                # do something
                pass

