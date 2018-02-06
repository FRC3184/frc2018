from wpilib.command import Command

from systems.elevator import Elevator


class ElevatorZeroCommand(Command):
    def __init__(self, elevator: Elevator):
        super().__init__("Elevator Zero Command")
        self.elevator = elevator
        self._state = 0
        self.is_done = False

    def initialize(self):
        self._state = 0
        self.is_done = False

    def execute(self):
        if self._state == 0:
            if self.elevator.is_at_bottom():
                self.elevator.set_power(0)
                self._state = 1
            else:
                self.elevator.set_power(-0.2)
        elif self._state == 1:
            self.elevator.start_zero_position()
            if self.elevator.is_done_zeroing():
                self.is_done = True

    def end(self):
        self.elevator.finish_zero_position()

    def isFinished(self):
        return self.is_done
