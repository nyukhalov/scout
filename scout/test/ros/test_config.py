import json
from scout.ros.config import PwmConfig, ControllerConfig, JoystickConfig
import unittest


class TestPwmConfig(unittest.TestCase):
    def test_from_json(self):
        json_str = """
        {
            "channel": 1,
            "speed": 2,
            "accel": 3,
            "min_val": 4,
            "max_val": 5,
            "offset": 6
        }
        """
        config_json = json.loads(json_str)
        config = PwmConfig.from_json(config_json)
        self.assertEquals(1, config.channel)
        self.assertEquals(2, config.speed)
        self.assertEquals(3, config.accel)
        self.assertEquals(4, config.min_val)
        self.assertEquals(5, config.max_val)
        self.assertEquals(6, config.offset)


class TestControllerConfig(unittest.TestCase):
    def test_from_json(self):
        json_str = """
        {
            "device": "device",
            "throttle": {
                "channel": 1,
                "speed": 2,
                "accel": 3,
                "min_val": 4,
                "max_val": 5,
                "offset": -1
            },
            "steering": {
                "channel": 6,
                "speed": 7,
                "accel": 8,
                "min_val": 9,
                "max_val": 10,
                "offset": -2
            }
        }
        """
        config_json = json.loads(json_str)
        config = ControllerConfig.from_json(config_json)
        self.assertEquals("device", config.device)
        self.assertEquals(1, config.throttle.channel)
        self.assertEquals(2, config.throttle.speed)
        self.assertEquals(3, config.throttle.accel)
        self.assertEquals(4, config.throttle.min_val)
        self.assertEquals(5, config.throttle.max_val)
        self.assertEquals(-1, config.throttle.offset)
        self.assertEquals(6, config.steering.channel)
        self.assertEquals(7, config.steering.speed)
        self.assertEquals(8, config.steering.accel)
        self.assertEquals(9, config.steering.min_val)
        self.assertEquals(10, config.steering.max_val)
        self.assertEquals(-2, config.steering.offset)


class TestJoystickCOnfig(unittest.TestCase):
    def test_from_json(self):
        json_str = """
        {
            "active_profile": "my_config",
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
        config = JoystickConfig.from_json(config_json)
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

