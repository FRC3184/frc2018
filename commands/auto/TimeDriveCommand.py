from wpilib.command import TimedCommand

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