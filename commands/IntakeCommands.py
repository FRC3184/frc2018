from wpilib import Timer
from wpilib.command import Command, TimedCommand

from systems.intake import Intake, ArmState


class MoveIntakeCommand(Command):
    def __init__(self, intake: Intake, new_state: ArmState):
        super().__init__("MoveIntakeCommand")
        self.intake = intake
        self.new_state = new_state
        self.old_state = None
        self.timer = Timer()
        self.requires(intake)

    def initialize(self):
        self.old_state = self.intake.state
        self.intake.set_arm_state(self.new_state)
        self.timer.reset()
        self.timer.start()

    def isFinished(self):
        return self.new_state == self.old_state or \
               (self.new_state == ArmState.DOWN and self.timer.get() > 0.2) or \
               (self.new_state == ArmState.UP and self.timer.get() > 0.5)

    def end(self):
        self.timer.stop()


class TimedRunIntakeCommand(TimedCommand):
    def __init__(self, intake: Intake, power, time=0):
        super().__init__("TimedRunIntake", time)
        self.power = power
        self.intake = intake
        self.requires(intake)

    def execute(self):
        self.intake.run_intake(self.power)

    def end(self):
        self.intake.run_intake(0)

