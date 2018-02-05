from wpilib.command import Command

import systems
from control import OI
from systems.intake import Intake


class OpIntakeCommand(Command):
    def __init__ (self, intake: Intake):
        super().__init__ ()
        self.requires(intake)
        self.intake = intake

    def initialize(self):
        pass

    def execute(self):
        oi = OI.get()
        intake = self.intake

        if oi.intake_is_active():
            intake.run_intake(1.0)
        elif oi.outtake_is_active():
            intake.run_intake(-1.0)
        else:
            intake.run_intake(0)

    def finish(self):
        pass

    def isFinished(self):
        return False