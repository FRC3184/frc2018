from wpilib.command import Command

import mathutils
from control import OI
from systems.drivetrain import Drivetrain
from systems.elevator import Elevator
from systems.elevator import TOP_EXTENT


class OpDriveCommand(Command):
    def __init__ (self, drivetrain: Drivetrain, elevator: Elevator):
        super().__init__ ()
        self.requires(drivetrain)
        self.drivetrain = drivetrain
        self.elevator = elevator

    def initialize(self):
        pass

    def execute(self):
        oi = OI.get()
        drive = self.drivetrain
        elevator = self.elevator

        power = self.get_speed_scale(elevator.get_elevator_position())*mathutils.signed_power(oi.get_left_power(), 2)
        turn = mathutils.signed_power(oi.get_turn_command(), 2)

        drive.arcade_drive(power, turn * 0.5)

    def end(self):
        pass

    def isFinished(self):
        return False

    def get_speed_scale(self, current_height):
        speed_scale = (-0.8/(TOP_EXTENT))*current_height+1

        return speed_scale