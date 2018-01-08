# This file controls the human-robot interface
# There should be functions such as get_turn_command and do_lift
# This is a singleton class, init() is called once at the beginning
# From then on, .get() is called to get the instance and methods are called on the instance
from wpilib import Joystick, XboxController

_instance = None


def init():
    global _instance
    if _instance is None:
        _instance = _OI()


def get():
    if _instance is not None:
        return _instance
    raise ValueError("OI not yet initialized")


class _OI:
    def __init__(self):
        self.left_joystick = Joystick(0)
        self.right_joystick = Joystick(1)
        self.gamepad = XboxController(2)

    def get_speed_command(self):
        return self.left_joystick.getY()

    def get_turn_command(self):
        return self.right_joystick.getX()
