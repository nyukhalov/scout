from typing import Dict


class PwmConfig:
    @staticmethod
    def from_json(json: Dict) -> "PwmConfig":
        channel = int(json["channel"])
        speed = int(json["speed"])
        accel = int(json["accel"])
        min_val = int(json["min_val"])
        max_val = int(json["max_val"])
        return PwmConfig(channel, speed, accel, min_val, max_val)

    def __init__(self, channel: int, speed: int, accel: int, min_val: int, max_val: int):
        self.channel = channel
        self.speed = speed
        self.accel = accel
        self.min_val = min_val
        self.max_val = max_val


class ControllerConfig:
    @staticmethod
    def from_json(json: Dict) -> "ControllerConfig":
        config = ControllerConfig()
        throttle_cfg_json = json["throttle"]
        steering_cfg_json = json["steering"]
        config.pwm_device = str(json["device"])
        config.throttle = PwmConfig.from_json(throttle_cfg_json)
        config.steering = PwmConfig.from_json(steering_cfg_json)
        return config

    def __init__(self):
        # default values
        self.pwm_device = "/dev/ttyACM0"
        self.throttle = PwmConfig(channel=5, speed=0, accel=0, min_val=5300, max_val=6500)
        self.steering = PwmConfig(channel=0, speed=50, accel=0, min_val=5000, max_val=7000)

