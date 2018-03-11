from wpilib.command import Command

from systems.climber import Climber
from systems.elevator import Elevator


class Climb(Command):
    def __init__(self, elevator: Elevator, climber: Climber):
        super().__init__(name="Climb")
        self.climber = climber
        self.elevator = elevator
        self.requires(climber)
        self.power = 0

    def initialize(self):
        self.power = 0

    def execute(self):
        self.climber.active_climber(self.power)
        if self.power < 1:
            self.power += 0.025

    def end(self):
        self.climber.inactive_climber()
        self.elevator.hold()

    def isFinished(self):
        return False