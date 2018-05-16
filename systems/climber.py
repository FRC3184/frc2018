from wpilib import Servo, VictorSP, PowerDistributionPanel
from wpilib.command import Subsystem


class Climber(Subsystem):
    def __init__(self):
        super().__init__(name="Climber")

        self.trigger = Servo(4)
        self.trigger.set(0)

        self.climb_victor1 = VictorSP(2)
        self.climb_victor2 = VictorSP(3)

        self.climb_pdp_port1 = 2
        self.climb_pdp_port2 = 3

        self.pdp = PowerDistributionPanel()

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

    def check_continuous_faults(self):
        """
        Check if the system is faulted.

        For the climber, we are interested in whether the versa dual-input has failed.
        :return:
        """

        master_current = self.pdp.getCurrent(self.climb_pdp_port1)
        slave_current = self.pdp.getCurrent(self.climb_pdp_port2)
        ref = master_current
        if ref == 0:
            ref = slave_current
        if ref == 0:
            ref = 1
        eps = 1e-1
        return False  # (master_current > eps or slave_current > eps) and abs(master_current - slave_current) / ref < 1