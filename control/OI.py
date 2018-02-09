# This file controls the human-robot interface
# There should be functions such as get_turn_command and do_lift
# This is a singleton class, init() is called once at the beginning
# From then on, .get() is called to get the instance and methods are called on the instance
from typing import Callable

import typing
from wpilib import Joystick, XboxController
from wpilib.command import Command

_instance = None


class OnCondition(object):

    def __init__(self, oi, condition):
        self.condition = condition
        self.oi = oi

    def __call__(self, action):
        self.oi.add_action_listener(self.condition, action)
        return action


class OnClick(OnCondition):
    def __init__(self, oi, joystick, button):
        super().__init__(oi, lambda: joystick.getButton(button))


class _OI:
    def __init__(self):
        self.gamepad = XboxController(0)
        self.gamepad1 = XboxController(1)

        self._action_listeners = []
        self._while_listeners = []

    # Utility
    def exec_while_condition(self, condition: Callable, cmd: Command):
        self._while_listeners.append((condition, cmd))

    def update(self):
        for condition, action in self._action_listeners:
            if condition():
                action()
        for condition, command in self._while_listeners:
            command = typing.cast(Command, command)
            cond_result = condition()
            if command.isRunning() and not cond_result:
                command.cancel()
            elif not command.isRunning() and cond_result:
                command.start()

    def add_action_listener(self, condition, action):
        self._action_listeners.append((condition, action))

    # OpDrive
    def get_left_power(self):
        return -self.gamepad.getY(XboxController.Hand.kLeft)

    def get_right_power(self):
        return -self.gamepad.getY(XboxController.Hand.kRight)

    def get_turn_command(self):
        return -self.gamepad.getX(XboxController.Hand.kRight)

    # Intake
    def intake_is_active(self):
        return self.gamepad1.getYButton()

    def outtake_is_active(self):
        return self.gamepad1.getBButton()

    def arm_is_down(self):
        return self.gamepad1.getTriggerAxis(XboxController.Hand.kRight) > 0.75

    # Elevator
    def get_elevator_manual_command(self):
        return -self.gamepad1.getY(XboxController.Hand.kLeft)

    def elevator_is_manual_control(self):
        return self.gamepad1.getTriggerAxis(XboxController.Hand.kLeft) > 0.75

    def elevator_move_to_top(self):
        return self.gamepad1.getBumper(XboxController.Hand.kRight)

    def elevator_move_to_bottom(self):
        return self.gamepad1.getBumper(XboxController.Hand.kLeft)

    def elevator_zero(self):
        return False


def get() -> _OI:
    global _instance
    if _instance is None:
        _instance = _OI()
    return _instance


def init():
    get()


class OIUpdateCommand(Command):
    def __init__(self):
        super().__init__()
        self.setRunWhenDisabled(True)

    def initialize(self):
        init()

    def execute(self):
        get().update()

    def isFinished(self):
        return False
