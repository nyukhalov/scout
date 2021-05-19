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
                    "btn_x": 1,
                    "btn_o": 2,
                    "btn_triangle": 3,
                    "btn_rect": 4,
                    "btn_l1": 5,
                    "btn_r1": 6,
                    "ax_left_horizontal": 7,
                    "ax_left_vertical": 8,
                    "ax_right_horizontal": 9,
                    "ax_right_vertical": 10,
                    "ax_l2": 11,
                    "ax_r2": 12
                }
            ]
        }
        """
        config_json = json.loads(json_str)
        config = JoystickConfig.from_json(config_json, "my_config")
        self.assertEquals("my_config", config.active_profile.name)
        self.assertEquals(1, config.active_profile.btn_x)
        self.assertEquals(2, config.active_profile.btn_o)
        self.assertEquals(3, config.active_profile.btn_triangle)
        self.assertEquals(4, config.active_profile.btn_rect)
        self.assertEquals(5, config.active_profile.btn_l1)
        self.assertEquals(6, config.active_profile.btn_r1)
        self.assertEquals(7, config.active_profile.ax_left_horizontal)
        self.assertEquals(8, config.active_profile.ax_left_vertical)
        self.assertEquals(9, config.active_profile.ax_right_horizontal)
        self.assertEquals(10, config.active_profile.ax_right_vertical)
        self.assertEquals(11, config.active_profile.ax_l2)
        self.assertEquals(12, config.active_profile.ax_r2)

