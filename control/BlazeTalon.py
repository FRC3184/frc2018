import hal
from ctre import TalonSRX


class BlazeTalon(TalonSRX):
    def __init__(self, deviceNumber: int):
        super().__init__(deviceNumber)
        self.phase = False
        self.setSensorPhase(self.phase)

    def setSensorPhase(self, PhaseSensor: bool):
        super().setSensorPhase(PhaseSensor)
        self.phase = PhaseSensor

    def getQuadraturePosition(self):
        # 1/-1 for simulation and practice robot, -1/1 on comp bot
        reverse = 1 if hal.isSimulation() else -1
        return (reverse if self.phase else -reverse) * super().getQuadraturePosition()