import rospy
import math
from sensor_msgs.msg import LaserScan
from scout.msg import CarControlStamped, CarControl, Actuators, Calibration
from scout.lib.pid import PID


class RosWallFollowerNode:
    def __init__(self):
        rospy.init_node("scout_wall_follower", anonymous=False, log_level=rospy.DEBUG)
        rospy.loginfo("Initializing node")

        scan_topic = rospy.get_param("~scan_topic", "/scan")
        ctrl_topic = rospy.get_param("~car_control_topic", "/car/control/auto")

        rospy.loginfo(f"Subscribing to {scan_topic} for laser scans")
        self._scan_sub = rospy.Subscriber(scan_topic, LaserScan, self._on_scan_msg)

        rospy.loginfo(f"Publishing control messages to {ctrl_topic}")
        self._ctrl_pub = rospy.Publisher(ctrl_topic, CarControlStamped, queue_size=1)
        self._seq = 0

        self._theta = math.pi / 8
        self._z_rot = math.pi  # TODO: get from TF
        self._pid = PID(14, 0, 0.09, setpoint=0.3)  # setpoint of 30 cm

    def run(self) -> None:
        rospy.loginfo("Starting node...")
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerror("The node has been interrupted by exception", e)
            traceback.print_exc()
        finally:
            rospy.loginfo("Shutting down...")

    def _get_distance_to_wall(self, a: float, b: float) -> float:
        """
        Return the distance from the lidar origin to the wall on the right
        Args:
            a: range value of the lidar ray at angle theta
            b: range value of the lidar ray at 0 degrees
        """
        alpha = math.atan((b / a - math.sin(self._theta)) / math.cos(self._theta))
        return b * math.cos(alpha)

    def _on_scan_msg(self, msg: LaserScan) -> None:
        num_rays = 1 + int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        assert num_rays == len(msg.ranges)

        target_angle_b = 1.5 * math.pi # 270 (-90) degrees in the base_link frame
        target_angle_a = target_angle_b + self._theta

        amin = msg.angle_min + self._z_rot
        amax = msg.angle_max + self._z_rot
        assert amin <= target_angle_a <= amax
        assert amin <= target_angle_b <= amax

        idx_a = int((target_angle_a - amin) // msg.angle_increment)
        idx_b = int((target_angle_b - amin) // msg.angle_increment)
        range_a = msg.ranges[idx_a]
        range_b = msg.ranges[idx_b]

        if msg.range_min <= range_a <= msg.range_max and msg.range_min <= range_b <= msg.range_max:
            dist = self._get_distance_to_wall(range_a, range_b)
            steer = self._pid.update(dist)
            steer = max(-1, min(1, steer))
            self._send_ctrl_msg(steer)
            rospy.logdebug(f"range_a={range_a}, range_b={range_b}, dist={dist}, steer={steer}")
        else:
            rospy.logdebug(f"range_a={range_a} or range_b={range_b} is not within allowed range [{msg.range_min}, {msg.range_max}]")

    def _send_ctrl_msg(self, steer: float) -> None:
        assert -1 <= steer <= 1
        self._seq += 1
        msg = CarControlStamped()
        msg.header.seq = self._seq
        msg.header.stamp = rospy.get_rostime()
        msg.control = CarControl()
        msg.control.activate_auto = False
        msg.control.actuators = Actuators()
        msg.control.actuators.gas = 0.3
        msg.control.actuators.brake = 0
        msg.control.actuators.steer = steer
        self._ctrl_pub.publish(msg)
