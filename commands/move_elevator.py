from wpilib.command import Command

import systems
from systems.elevator import Elevator


class MoveElevatorCommand(Command):
    def __init__(self, elevator: Elevator, target_pos):
        super().__init__()

        self.elevator = elevator
        self.requires(self.elevator)
        self.target_pos = target_pos

    def initialize(self):
        self.elevator.init_profile(self.target_pos)

    def execute(self):
        # Keep trying to start the profile. Even if it has started we don't care
        self.elevator.start_profile()

    def isFinished(self):
        return self.elevator.has_finished_profile()

    def finish(self):
        pass