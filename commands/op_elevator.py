import typing

from wpilib.command import Command

import systems
from control import OI
from systems.elevator import Elevator


class OpElevatorManualCommand(Command):
    def __init__ (self, elevator: Elevator):
        super().__init__ ()
        self.requires(elevator)
        self.elevator = elevator

    def initialize(self):
        pass

    def execute(self):
        oi = OI.get()
        elevator = self.elevator

        power = oi.get_elevator_manual_command()
        elevator.set_power(power)

    def finish(self):
        self.elevator.set_power(0)

    def cancel(self):
        super().cancel()
        self.elevator.set_power(0)

    def isFinished(self):
        return False