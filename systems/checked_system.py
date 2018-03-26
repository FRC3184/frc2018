from typing import Collection, List

from wpilib import DriverStation
from wpilib.command import Subsystem, Command


class FaultableSystem(Subsystem):
    """
    Represents a system that can have mechanical failures detectable by sensors.
    """
    def check_continuous_faults(self):
        """
        Check if the system is faulted.
        :return: True if the system is in a fault condition, else False
        """
        return False


class CheckFaults(Command):
    def __init__(self, systems: List[FaultableSystem]):
        super().__init__("CheckFaults command")
        self.systems = systems
        self.time_accumulator = 0

    def initialize(self):
        self.time_accumulator = 0

    def execute(self):
        self.time_accumulator += 1
        if self.time_accumulator == 20:
            self.time_accumulator = 0
            for system in self.systems:
                if system.check_continuous_faults():
                    DriverStation.getInstance().reportWarning(f"!!! {system.getName()} is faulted !!!", False)