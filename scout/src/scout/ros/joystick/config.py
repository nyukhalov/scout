from typing import Dict
from scout.ros.joystick.input import JoystickProfile


class JoystickConfig:
    @staticmethod
    def from_json(json: Dict, profile: str) -> "JoystickConfig":
        for profile_json in json["profiles"]:
            if profile_json["name"] == profile:
                active_profile = JoystickProfile.from_json(profile_json)
                return JoystickConfig(active_profile)
        raise Exception(f"Unable to load profile: {profile}")

    def __init__(self, active_profile: JoystickProfile):
        self.active_profile = active_profile

    def __str__(self) -> str:
        return str(vars(self))

    def __repr__(self) -> str:
        return self.__str__()

