from wpilib.command import Command

import systems
from control import OI
from systems.drivetrain import Drivetrain


class OpDriveCommand(Command):
    def __init__ (self, drivetrain: Drivetrain):
        super().__init__ ()
        self.requires(drivetrain)
        self.drivetrain = drivetrain

    def initialize(self):
        pass

    def execute(self):
        oi = OI.get()
        drive = self.drivetrain

        left = oi.get_right_power()
        right = oi.get_right_power()

        if left > 0:
            left = left **2
        elif left < 0:
            left = -(left **2)
        else:
            left = 0

        if right > 0:
            right = right ** 2
        elif right < 0:
            right = -(right ** 2)
        else:
            right = 0

        drive.tank_drive(left, right)

    def end(self):
        pass

    def isFinished(self):
        return False
