# This file controls the human-robot interface
# There should be functions such as get_turn_command and do_lift
# This is a singleton class, init() is called once at the beginning
# From then on, .get() is called to get the instance and methods are called on the instance
from wpilib import Joystick, XboxController

_instance = None


class OnCondition(object):

    def __init__(self, oi, condition):
        self.condition = condition
        self.oi = oi

    def __call__(self, f):
        self.oi.add_action_listener(self.condition, f)
        return f


class OnClick(OnCondition):
    def __init__(self, oi, joystick, button):
        super().__init__(oi, lambda: joystick.getButton(button))


class _OI:
    def __init__(self):
        self.left_joystick = Joystick(0)
        self.right_joystick = Joystick(1)
        self.gamepad = XboxController(2)

        self._action_listeners = []
        OnClick(self, self.left_joystick, 1)(self.move_elevator_to_bottom)

    def add_action_listener(self, condition, action):
        self._action_listeners.append((condition, action))

    def get_speed_command(self):
        return self.left_joystick.getY()

    def get_turn_command(self):
        return self.right_joystick.getX()

    def move_elevator_to_bottom(self):
        pass  # This is an example action

    def get_elevator_manual_command(self):
        return self.gamepad.getRawAxis(0)

    def intake_is_active(self):
        return self.gamepad.getYButton()

    def outtake_is_active(self):
        return self.gamepad.getBButton()

    def intake_is_open(self):
        return self.gamepad.getXButton()

def get() -> _OI:
    global _instance
    if _instance is None:
        _instance = _OI()
    return _instance


def init():
    get()