import typing

from wpilib.command import Command

import systems
from control import OI
from systems.Elevator import Elevator


class OpElevatorCommand(Command):
    def __init__ (self, robot):
        super().__init__ ()
        self.requires(systems.elevator)
        self.robot = robot

    def initialize(self):
        pass

    def execute(self):
        oi = OI.get()
        elevator = typing.cast(systems.elevator, Elevator)

        power = oi.get_elevator_manual_command()
        elevator.manual_set(power)

    def finish(self):
        pass

    def isFinished(self):
        return False