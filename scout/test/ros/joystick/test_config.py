import json
from scout.ros.joystick.config import JoystickConfig
import unittest


class TestJoystickCOnfig(unittest.TestCase):
    def test_from_json(self):
        json_str = """
        {
            "profiles": [
                {
                    "name": "my_config",
                    "btn_down": 1,
                    "btn_right": 2,
                    "btn_top": 3,
                    "btn_left": 4,
                    "left_bumper": 5,
                    "right_bumper": 6,
                    "lsb_horizontal": 7,
                    "lsb_vertical": 8,
                    "rsb_horizontal": 9,
                    "rsb_vertical": 10,
                    "left_trigger": 11,
                    "right_trigger": 12
                }
            ]
        }
        """
        config_json = json.loads(json_str)
        config = JoystickConfig.from_json(config_json, "my_config")
        self.assertEquals("my_config", config.active_profile.name)
        self.assertEquals(1, config.active_profile.btn_down)
        self.assertEquals(2, config.active_profile.btn_right)
        self.assertEquals(3, config.active_profile.btn_top)
        self.assertEquals(4, config.active_profile.btn_left)
        self.assertEquals(5, config.active_profile.left_bumper)
        self.assertEquals(6, config.active_profile.right_bumper)
        self.assertEquals(7, config.active_profile.lsb_horizontal)
        self.assertEquals(8, config.active_profile.lsb_vertical)
        self.assertEquals(9, config.active_profile.rsb_horizontal)
        self.assertEquals(10, config.active_profile.rsb_vertical)
        self.assertEquals(11, config.active_profile.left_trigger)
        self.assertEquals(12, config.active_profile.right_trigger)

