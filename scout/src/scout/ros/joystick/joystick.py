import rospy
import json
from sensor_msgs.msg import Joy
from scout.msg import CarControlStamped, CarControl, Actuators, Calibration
from scout.ros.joystick.input import DualShockInput
from scout.ros.joystick.config import JoystickConfig

class RosJoystickNode:
    def __init__(self):
        self._seq = 0

        rospy.init_node("scout_joystick", anonymous=False, log_level=rospy.INFO)
        rospy.loginfo("Initializing node")

        joy_topic = rospy.get_param("~joy_topic", "/joy")
        joystick_profile = rospy.get_param("~joystick_profile", "JetsonDualShock4")
        ctrl_topic = rospy.get_param("~car_control_topic", "/car/control/joystick")

        joystick_config_file = rospy.get_param("~joystick_config", None)
        joystick_config = self._load_joystick_config(joystick_config_file, joystick_profile)
        rospy.loginfo(f"Joystick configuration: {joystick_config}")

        self._joy_input = DualShockInput(joystick_config.active_profile)

        rospy.loginfo(f"Listening Joy messages from {joy_topic}")
        self._joy_sub = rospy.Subscriber(joy_topic, Joy, self._on_joy_input)

        rospy.loginfo(f"Publishing control messages to {ctrl_topic}")
        self._ctrl_pub = rospy.Publisher(ctrl_topic, CarControlStamped, queue_size=1)

    def run(self) -> None:
        rospy.loginfo("Starting joystick node")
        rospy.spin()

    def _load_joystick_config(self, config_file: str, profile: str) -> JoystickConfig:
        rospy.loginfo(f"Loading joystick config (profile={profile})")
        with open(config_file, "r") as f:
            config_json = json.load(f)
            return JoystickConfig.from_json(config_json, profile)

    def _on_joy_input(self, msg: Joy) -> None:
        self._joy_input.handle_message(msg)

        steer_offset_inc = 0
        if self._joy_input.is_steering_offset_dec():
            steer_offset_inc = - 1
        elif self._joy_input.is_steering_offset_inc():
            steer_offset_inc = 1

        self._seq += 1
        msg = CarControlStamped()
        msg.header.seq = self._seq
        msg.header.stamp = rospy.get_rostime()
        msg.control = CarControl()
        msg.control.actuators = Actuators()
        msg.control.actuators.gas = self._joy_input.throttle()
        msg.control.actuators.brake = self._joy_input.braking()
        msg.control.actuators.steer = self._joy_input.steering()
        msg.control.calibration = Calibration()
        msg.control.calibration.steer_offset_inc = steer_offset_inc
        self._ctrl_pub.publish(msg)

