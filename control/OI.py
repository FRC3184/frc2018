# This file controls the human-robot interface
# There should be functions such as get_turn_command and do_lift
# This is a singleton class, init() is called once at the beginning
# From then on, .get() is called to get the instance and methods are called on the instance
import typing
from typing import Callable

import wpilib
from wpilib import XboxController
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
        self.drive_gamepad = XboxController(0)
        self.op_gamepad = XboxController(1)

        self._action_listeners = []
        self._while_listeners = []

        self.arm_down = LambdaToggle(lambda: self.op_gamepad.getTriggerAxis(XboxController.Hand.kRight) > 0.75)

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
        return -self.drive_gamepad.getY(XboxController.Hand.kLeft)

    def get_right_power(self):
        return -self.drive_gamepad.getY(XboxController.Hand.kRight)

    def get_turn_command(self):
        return -self.drive_gamepad.getX(XboxController.Hand.kRight)

    def get_fine_left_turn(self):
        return self.drive_gamepad.getXButton()

    def get_fine_right_turn(self):
        return self.drive_gamepad.getBButton()

    def get_fine_forward(self):
        return self.drive_gamepad.getPOV() == 0

    def get_fine_backward(self):
        return self.drive_gamepad.getPOV() == -180

    def get_spot_turn(self):
        return self.drive_gamepad.getBumper(XboxController.Hand.kLeft)

    # Intake
    def outtake_is_active(self):
        return self.op_gamepad.getBumper(XboxController.Hand.kRight)

    def intake_is_active(self):
        return self.op_gamepad.getYButton()

    def get_outtake_command(self):
        return self.op_gamepad.getY(XboxController.Hand.kRight)

    def arm_is_down(self):
        return self.arm_down.get()

    def arm_is_open(self):
        return self.op_gamepad.getBumper(XboxController.Hand.kLeft)

    def rumble_op(self):
        rumble_val = 0.75
        self.op_gamepad.setRumble(XboxController.RumbleType.kLeftRumble, rumble_val)
        self.op_gamepad.setRumble(XboxController.RumbleType.kRightRumble, rumble_val)

    def unrumble_op(self):
        self.op_gamepad.setRumble(XboxController.RumbleType.kLeftRumble, 0)
        self.op_gamepad.setRumble(XboxController.RumbleType.kRightRumble, 0)

    # Elevator
    def get_elevator_manual_command(self):
        return -self.op_gamepad.getY(XboxController.Hand.kLeft)

    def elevator_is_manual_control(self):
        return self.op_gamepad.getTriggerAxis(XboxController.Hand.kLeft) > 0.75

    def elevator_move_to_top(self):
        return self.op_gamepad.getPOV() == 0

    def elevator_move_to_bottom(self):
        return self.op_gamepad.getPOV() == 180

    def elevator_zero(self):
        return False

    # Climber
    def do_drop_forks(self):
        return self.op_gamepad.getBackButton()

    def do_climb(self):
        return self.op_gamepad.getStartButton()


def get() -> _OI:
    global _instance
    if _instance is None:
        _instance = _OI()
    return _instance


def init():
    get()


class LambdaToggle:
    """Similar to pyfrc toggle, but takes an arbitrary function as input

    Usage::

        foo = Toggle(f)

        if foo:
            toggleFunction()

        if foo.on:
            onToggle()

        if foo.off:
            offToggle()
    """
    class _SteadyDebounce:
        """
            Similar to ButtonDebouncer, but the output stays steady for
            the given periodic_filter. E.g, if you set the period to 2
            and press the button, the value will return true for 2 seconds.

            Steady debounce will return true for the given period, allowing it to be
            used with Toggle
        """

        def __init__(self, f: callable, period: float):
            """
            :param joystick:  Joystick object
            :type  joystick:  :class:`wpilib.Joystick`
            :param button: Number of button to retrieve
            :type  button: int
            :param period:    Period of time (in seconds) to wait before allowing new button
                              presses. Defaults to 0.5 seconds.
            :type  period:    float
            """
            self.function = f

            self.debounce_period = float(period)
            self.latest = - self.debounce_period # Negative latest prevents get from returning true until joystick is presed for the first time
            self.enabled = False

        def get(self):
            """
            :returns: The value of the joystick button. Once the button is pressed,
            the return value will be `True` until the time expires
            """

            now = wpilib.Timer.getFPGATimestamp()
            if now - self.latest < self.debounce_period:
                return True

            if self.function():
                self.latest = now
                return True
            else:
                return False

    def __init__(self, f: callable, debounce_period: float=None):
        """
        :param f: Function to debounce
        :param debounce_period: Period in seconds to wait before registering a new button press.
        """

        if debounce_period is not None:
            self.joystickget = LambdaToggle._SteadyDebounce(f, debounce_period).get
        else:
            self.joystickget = f

        self.released = False
        self.toggle = False
        self.state = False

    def get(self):
        """
         :return: State of toggle
         :rtype: bool
         """
        current_state = self.joystickget()

        if current_state and not self.released:
            self.released = True
            self.toggle = not self.toggle
            self.state = not self.state # Toggles between 1 and 0.

        elif not current_state and self.released:
            self.released = False

        return self.toggle

    @property
    def on(self):
        """
        Equates to true if toggle is in the 'on' state
        """
        self.get()
        return self.state

    @property
    def off(self):
        """
        Equates to true if toggle is in the 'off' state
        """
        self.get()
        return not self.state

    __bool__ = get


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
