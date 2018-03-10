from wpilib.command import Command

from systems.climber import Climber


class DropForkliftCommand(Command):
    def __init__(self, forklift: Climber):
        super().__init__(name="DropForklift command")
        self.forklift = forklift
        self.requires(forklift)

    def initialize(self):
        self.forklift.open_gate()

    def isFinished(self):
        return True
