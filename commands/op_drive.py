from wpilib.command import Command

import mathutils
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

        power = mathutils.signed_power(oi.get_left_power(), 2)
        turn = mathutils.signed_power(oi.get_turn_command(), 2)

        drive.arcade_drive(power, turn * 0.5)

    def end(self):
        pass

    def isFinished(self):
        return False
