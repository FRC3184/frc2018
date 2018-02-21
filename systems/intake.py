from ctre import ControlMode
from wpilib import Talon, DoubleSolenoid, AnalogInput, Solenoid
from wpilib.command import Subsystem

from dashboard import dashboard2


class ArmState:
    DOWN = 0
    UP = 1


class GrabState:
    IN = 0
    OUT = 1


class Intake(Subsystem):
    def __init__(self):
        super().__init__("Intake")

        self.talon_left = Talon(0)
        self.talon_right = Talon(1)

        self.solenoid_lift = DoubleSolenoid(0, 1)
        self.solenoid_grab = DoubleSolenoid(2, 3)
        self.set_arm_state(ArmState.UP)

        self.power = .75

        self.in_sensor = AnalogInput(0)
        dashboard2.add_graph("Ultrasonic", self.get_reported_distance)

    def get_reported_distance(self):
        return self.in_sensor.getVoltage() * 254 / 5

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

    def get_arm_state(self):
        return ArmState.DOWN if self.solenoid_lift.get() == DoubleSolenoid.Value.kReverse else ArmState.UP

    def set_arm_state(self, state):
        if state == ArmState.DOWN:
            self.solenoid_lift.set(DoubleSolenoid.Value.kReverse)
        else:
            self.solenoid_lift.set(DoubleSolenoid.Value.kForward)

    def get_grab_state(self):
        return GrabState.OUT if self.solenoid_grab.get() == DoubleSolenoid.Value.kForward else GrabState.IN

    def set_grab_state(self, state):
        if state == GrabState.OUT:
            self.solenoid_grab.set(DoubleSolenoid.Value.kForward)
        else:
            self.solenoid_grab.set(DoubleSolenoid.Value.kReverse)

    def has_acquired_cube(self):
        return self.get_reported_distance() < 10