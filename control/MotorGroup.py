from typing import Union, List

from ctre import ControlMode
from ctre.talonsrx import TalonSRX
from ctre.victorspx import VictorSPX
from wpilib import PWM, PWMSpeedController
from wpilib.interfaces import SpeedController


class SmartMotorGroup:
    def __init__(self, leader: TalonSRX, followers: List[Union[TalonSRX, VictorSPX]] = ()):
        self._leader = leader
        self._followers = followers

        self.setup_followers()

    def setup_followers(self):
        for controller in self._followers:
            controller.follow(self._leader)

    def add_follower(self, follower: Union[TalonSRX, VictorSPX]):
        self._followers.append(follower)
        follower.follow(self._leader)

    @property
    def leader(self) -> TalonSRX:
        return self._leader


class MotorGroup(SpeedController):
    def __init__(self, group: List[SpeedController]):
        super().__init__()
        self.group = group
        self.speed = 0
        self.isInverted = False

    def set(self, speed):
        self.speed = speed
        for sc in self.group:
            sc.set(speed * (-1 if self.isInverted else 1))

    def get(self):
        return self.speed

    def setInverted(self, isInverted):
        self.isInverted = isInverted

    def getInverted(self):
        return self.isInverted

    def disable(self):
        for sc in self.group:
            sc.disable()

    def stopMotor(self):
        self.set(0)

    def pidWrite(self, output):
        self.set(output)



