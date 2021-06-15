import rospy
from scout.msg import CarControlStamped, CarControl, Actuators, Calibration

class RosControlMultiplexerNode:

    def __init__(self):
        rospy.init_node("scout_ctrl_multiplexer", anonymous=False, log_level=rospy.INFO)
        rospy.loginfo("Initializing node")

        joy_ctrl_topic = rospy.get_param("~car_joystick_control_topic", "/car/control/joystick")
        auto_ctrl_topic = rospy.get_param("~car_auto_control_topic", "/car/control/auto")
        final_ctrl_topic = rospy.get_param("~car_final_control_topic", "/car/control")

        # subscribers
        rospy.loginfo(f"Subscribing to {joy_ctrl_topic} for Joystick controls")
        self._joy_ctrl_sub = rospy.Subscriber(joy_ctrl_topic, CarControlStamped, self._on_joy_ctrl)
        rospy.loginfo(f"Subscribing to {auto_ctrl_topic} for auto controls")
        self._auto_ctrl_sub = rospy.Subscriber(auto_ctrl_topic, CarControlStamped, self._on_auto_ctrl)

        # publishers
        self._ctrl_pub = rospy.Publisher(final_ctrl_topic, CarControlStamped, queue_size=1)

        # keep track of last ctrl messages
        self._last_joy_ctrl_msg = None
        self._last_auto_ctrl_msg = None

    def run(self) -> None:
        rospy.loginfo("Starting node...")
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerror("The node has been interrupted by exception", e)
            traceback.print_exc()
        finally:
            rospy.loginfo("Shutting down...")

    def _compose_and_publish_ctrl_msg(self) -> None:
        if self._last_joy_ctrl_msg is None:
            rospy.logwarn("Have not received a joystick control message yet")
            return
        if self._last_auto_ctrl_msg is None:
            rospy.logwarn("Have not received an auto control message yet")
            return

        # override gas/braking using joystick values
        self._last_auto_ctrl_msg.control.actuators.gas = self._last_joy_ctrl_msg.control.actuators.gas
        self._last_auto_ctrl_msg.control.actuators.brake = self._last_joy_ctrl_msg.control.actuators.brake
        self._ctrl_pub.publish(self._last_auto_ctrl_msg)

    def _on_joy_ctrl(self, msg: CarControlStamped) -> None:
        self._last_joy_ctrl_msg = msg
        self._compose_and_publish_ctrl_msg()

    def _on_auto_ctrl(self, msg: CarControlStamped) -> None:
        self._last_auto_ctrl_msg = msg
        self._compose_and_publish_ctrl_msg()
