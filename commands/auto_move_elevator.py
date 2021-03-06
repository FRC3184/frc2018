from wpilib.command import Command

from systems.elevator import Elevator


class ProfileMoveElevatorCommand(Command):
    def __init__(self, elevator: Elevator, target_pos):
        super().__init__()

        self.elevator = elevator
        self.requires(self.elevator)
        self.target_pos = target_pos

        self.failed_init = False

    def initialize(self):
        # If we couldn't initialize the profile, there's no point in moving forward
        self.failed_init = not self.elevator.init_profile(self.target_pos)
        if self.failed_init:
            print("Elevator FAILED INIT")

    def execute(self):
        if not self.failed_init:
            # Keep trying to start the profile. Even if it has started we don't care
            self.elevator.start_profile()

    def isFinished(self):
        return self.elevator.has_finished_profile() or self.failed_init

    def end(self):
        print("Finished profile")
        self.elevator.finish_profile()


class MoveElevatorCommand(Command):
    def __init__(self, elevator: Elevator, target_pos):
        super().__init__()

        self.elevator = elevator
        self.requires(self.elevator)
        self.target_pos = target_pos

    def initialize(self):
        pass

    def execute(self):
        self.elevator.move_to(self.target_pos)

    def isFinished(self):
        return abs(self.elevator.get_elevator_position() - self.target_pos) < 1

    def end(self):
        print("Finished profile")