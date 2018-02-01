from ctre import ControlMode
from ctre.talonsrx import TalonSRX
from wpilib.command import Subsystem


class Elevator(Subsystem):
    def __init__(self):
        super().__init__("Elevator")

        self.talon_master = TalonSRX(4)
        self.talon_slave = TalonSRX(5)
        self.talon_slave.follow(self.talon_master)

    def manual_set(self, power):
        self.talon_master.set(ControlMode.PercentOutput, power)