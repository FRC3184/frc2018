import math

from wpilib.command import Command

from systems.drivetrain import Drivetrain


class TurnToAngle(Command):
    def __init__(self, drive: Drivetrain, angle: float, margin: float = 5, delta: bool = True):
        super().__init__("TurnToAngle")
        self.requires(drive)
        self.drive = drive
        self.angle = angle
        self.delta = delta
        self.base_angle = 0
        self.margin = margin

    def get_cur_angle(self):
        return self.drive.robotdrive.get_heading()

    def initialize(self):
        self.base_angle = self.get_cur_angle()

    def execute(self):
        err = self.get_cur_angle() - self.angle
        print(err)
        if self.delta:
            err -= self.base_angle

        min_output = 0.2
        gain = (0.01/180) * err
        if abs(gain) < min_output:
            gain = math.copysign(min_output, gain)

        self.drive.arcade_drive(0, -gain)

    def isFinished(self):
        return abs(self.get_cur_angle() - self.angle) < self.margin

    def end(self):
        self.drive.arcade_drive(0, 0)