import rospy
import math
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
        num_rays = len(msg.ranges)
        assert num_rays == int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        if num_rays == 0:
            rospy.logwarn(f"num_rays must be positive, but got {num_rays}")
            return

        z_rot = math.pi  # TODO: get from TF
        target_angle = 1.5 * math.pi # 270 (-90) degrees in the base_link frame
        amin = msg.angle_min + z_rot
        amax = msg.angle_max + z_rot
        assert amin <= target_angle <= amax

        idx = int((target_angle - amin) // msg.angle_increment)
        target_range = msg.ranges[idx]

        if msg.range_min <= target_range <= msg.range_max:
            # TODO: do someting
            rospy.loginfo(f"Range={target_range}")
        else:
            rospy.logwarn(f"Range {target_range} is not within allowed range [{msg.range_min}, {msg.range_max}]")

