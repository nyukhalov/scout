import rospy
from scout.msg import CarControlStamped, CarControl, Actuators, Calibration

class RosControlMultiplexerNode:

    def __init__(self):
        rospy.init_node("scout_ctrl_multiplexer", anonymous=False, log_level=rospy.INFO)
        rospy.loginfo("Initializing node")

        joy_ctrl_topic = rospy.get_param("~car_joystick_control_topic", "/car/control/joystick")
        auto_ctrl_topic = rospy.get_param("~car_auto_control_topic", "/car/control/auto")
        final_ctrl_topic = rospy.get_param("~car_final_control_topic", "/car/control")

        # start in manual mode
        self._is_auto = False

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
        # first we ensure there was at least one joy control message
        if self._last_joy_ctrl_msg is None:
            rospy.logwarn("Have not received a joystick control message yet")
            return

        # We want to take over when the user steer using the Joystick.
        # The threshold is used to avoid false positives when the stick slightly jitters
        is_steer_override = abs(self._last_joy_ctrl_msg.control.actuators.steer) > 0.02
        is_takeover = self._is_auto and is_steer_override
        if is_takeover:
            rospy.logwarn("Activating MANUAL mode")
            self._is_auto = False

        # republish the joy message when in manual mode
        if not self._is_auto:
            self._ctrl_pub.publish(self._last_joy_ctrl_msg)
            return

        if self._last_auto_ctrl_msg is None:
            rospy.logwarn("Have not received an auto control message yet")
            return

        # override gas/braking using joystick values
        self._last_auto_ctrl_msg.control.actuators.gas = self._last_joy_ctrl_msg.control.actuators.gas
        self._last_auto_ctrl_msg.control.actuators.brake = self._last_joy_ctrl_msg.control.actuators.brake
        self._ctrl_pub.publish(self._last_auto_ctrl_msg)

    def _on_joy_ctrl(self, msg: CarControlStamped) -> None:
        if (msg.control.activate_auto and not self._is_auto):
            rospy.logwarn("Activating AUTO mode")
            self._is_auto = True
        self._last_joy_ctrl_msg = msg
        self._compose_and_publish_ctrl_msg()

    def _on_auto_ctrl(self, msg: CarControlStamped) -> None:
        self._last_auto_ctrl_msg = msg
        self._compose_and_publish_ctrl_msg()
