from wpilib import Servo, VictorSP
from wpilib.command import Subsystem


class Climber(Subsystem):
    def __init__(self):
        super().__init__(name="Climber")

        self.trigger = Servo(4)
        self.trigger.set(0)

        self.climb_victor1 = VictorSP(2)
        self.climb_victor2 = VictorSP(3)

    def close_gate(self):
        self.trigger.set(0)

    def open_gate(self):
        self.trigger.set(0.5)

    def active_climber(self, climb_power):
        self.climb_victor1.set(climb_power)
        self.climb_victor2.set(climb_power)

    def inactive_climber(self):
        self.climb_victor1.set(0)
        self.climb_victor2.set(0)

