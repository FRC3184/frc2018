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
        return (1 if self.phase else -1) * super().getQuadraturePosition()