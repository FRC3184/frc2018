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

        speed = oi.get_speed_command()
        turn = oi.get_turn_command()

        if speed > 0:
            speed = speed **2
        elif speed < 0:
            speed = -(speed **2)
        else:
            speed = 0

        if turn > 0:
            turn = turn ** 2
        elif turn < 0:
            turn = -(turn ** 2)
        else:
            turn = 0

        drive.arcade_drive(-speed, -turn)

    def end(self):
        pass

    def isFinished(self):
        return False
