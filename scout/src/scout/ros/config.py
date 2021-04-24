from typing import Dict
from scout.ros.joystick import JoystickProfile


class PwmConfig:
    @staticmethod
    def from_json(json: Dict) -> "PwmConfig":
        channel = int(json["channel"])
        speed = int(json["speed"])
        accel = int(json["accel"])
        min_val = int(json["min_val"])
        max_val = int(json["max_val"])
        offset = int(json["offset"])
        return PwmConfig(channel, speed, accel, min_val, max_val, offset)

    def __init__(self, channel: int, speed: int, accel: int, min_val: int, max_val: int, offset: int):
        self.channel = channel
        self.speed = speed
        self.accel = accel
        self.min_val = min_val
        self.max_val = max_val
        self.offset = offset

    def __str__(self) -> str:
        return str(vars(self))

    def __repr__(self) -> str:
        return self.__str__()


class ControllerConfig:
    @staticmethod
    def from_json(json: Dict) -> "ControllerConfig":
        config = ControllerConfig()
        throttle_cfg_json = json["throttle"]
        steering_cfg_json = json["steering"]
        config.device = str(json["device"])
        config.throttle = PwmConfig.from_json(throttle_cfg_json)
        config.steering = PwmConfig.from_json(steering_cfg_json)
        return config

    def __init__(self):
        # default values
        self.device = "/dev/ttyACM0"
        self.throttle = PwmConfig(channel=5, speed=0, accel=0, min_val=5300, max_val=6500, offset=0)
        self.steering = PwmConfig(channel=0, speed=50, accel=0, min_val=5000, max_val=7000, offset=0)

    def __str__(self) -> str:
        return str(vars(self))

    def __repr__(self) -> str:
        return self.__str__()


class JoystickConfig:
    @staticmethod
    def from_json(json: Dict) -> "JoystickConfig":
        active_profile_name = json["active_profile"]
        for profile_json in json["profiles"]:
            if profile_json["name"] == active_profile_name:
                active_profile = JoystickProfile.from_json(profile_json)
                return JoystickConfig(active_profile)
        raise Exception(f"Unable to load active profile: {active_profile_name}")

    def __init__(self, active_profile: JoystickProfile):
        self.active_profile = active_profile

    def __str__(self) -> str:
        return str(vars(self))

    def __repr__(self) -> str:
        return self.__str__()

