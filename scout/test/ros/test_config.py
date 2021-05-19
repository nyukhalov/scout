import json
from scout.ros.config import PwmConfig, ControllerConfig
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

