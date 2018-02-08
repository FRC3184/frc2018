from ctre import ControlMode
from wpilib import Talon, DoubleSolenoid
from wpilib.command import Subsystem


class ArmState:
    DOWN = 0
    UP = 1


class Intake(Subsystem):
    def __init__(self):
        super().__init__("Intake")

        self.talon_left = Talon(0)
        self.talon_right = Talon(1)

        self.solenoid_lift = DoubleSolenoid(0, 1)
        self.set_arm_state(ArmState.UP)

        self.power = .5

    def run_intake(self, power):
        """
        positive power runs motors in
        negative power runs motors out
        :param power: The power to run the intake motors at
        :return:
        """
        self.talon_left.set(power)
        self.talon_right.set(-power)

    def intake(self):
        self.run_intake(self.power)

    def eject(self):
        self.run_intake(-self.power)

    def set_arm_state(self, state):
        if state == ArmState.DOWN:
            self.solenoid_lift.set(DoubleSolenoid.Value.kForward)
        else:
            self.solenoid_lift.set(DoubleSolenoid.Value.kReverse)