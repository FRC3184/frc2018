from wpilib import Timer
from wpilib.command import Command, TimedCommand

from systems.drivetrain import Drivetrain
from systems.intake import Intake, ArmState, GrabState


class MoveIntakeCommand(Command):
    def __init__(self, intake: Intake, new_state: ArmState):
        super().__init__("MoveIntakeCommand")
        self.intake = intake
        self.new_state = new_state
        self.old_state = None
        self.timer = Timer()
        self.requires(intake)

    def initialize(self):
        self.old_state = self.intake.get_arm_state()
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


class AcquireCube(Command):
    def __init__(self, drive_speed, timeout, drive: Drivetrain, intake: Intake):
        super().__init__("AcquireCube", timeout)
        self.drive_speed = drive_speed
        self.drive = drive
        self.intake = intake
        self.requires(intake)
        self.requires(drive)
        self.state = 0
        self.timer = Timer()

    def initialize(self):
        self.state = 0
        self.intake.set_arm_state(ArmState.DOWN)
        self.intake.set_grab_state(GrabState.OUT)
        self.timer.start()

    def execute(self):
        if self.state == 0:
            self.drive.arcade_drive(0, 0)
            if self.timer.get() > 0.2:
                self.state = 1
        elif self.state == 1:
            self.intake.intake()
            self.drive.arcade_drive(self.drive_speed, 0)

    def isFinished(self):
        return self.intake.has_acquired_cube()

    def end(self):
        self.intake.set_grab_state(GrabState.IN)
        self.intake.set_arm_state(ArmState.UP)
        self.drive.arcade_drive(0, 0)