from ctre import ControlMode
from wpilib import Talon
from wpilib.command import Subsystem


class intake(Subsystem):
    def __init__(self):
        super().__init__("Intake")

        self.talon_left = Talon(0)
        self.talon_right = Talon(1)

    #positive power runs motors in
    #negative power runs motors out
    def run_intake(self, power):
        self.talon_left.set(self, power)
        self.talon_right.set(self, (-power))

    def open_intake(self):
        pass

    def close_intake(self):
        pass