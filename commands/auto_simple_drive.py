from wpilib.command import TimedCommand, Command

from control import pose
from systems.drivetrain import Drivetrain


class TimeDriveCommand(TimedCommand):
    def __init__(self, drive: Drivetrain, time, power):
        super().__init__("TimeDrive", time)
        self.drive = drive
        self.requires(self.drive)
        self.power = power

    def execute(self):
        self.drive.arcade_drive(self.power, 0)

    def end(self):
        self.drive.arcade_drive(0, 0)


class DistanceDriveCommand(Command):
    def __init__(self, drive: Drivetrain, power, distance):
        super().__init__("DistanceDrive")
        self.drive = drive
        self.requires(self.drive)
        self.power = power
        self.distance = distance

    def initialize(self):
        self.start_pose = pose.get_current_pose().copy()

    def execute(self):
        self.drive.arcade_drive(self.power, 0)

    def isFinished(self):
        return pose.get_current_pose().distance(self.start_pose) > self.distance

    def end(self):
        self.drive.arcade_drive(0, 0)