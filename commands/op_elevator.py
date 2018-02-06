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
        if abs(power) < 0.05:
            power = 0
        elevator.set_power(power)
        print(elevator.get_elevator_position())

    def end(self):
        self.elevator.set_power(0)
        # self.elevator.hold()

    def isFinished(self):
        return False