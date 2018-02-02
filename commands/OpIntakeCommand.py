from wpilib.command import Command

import systems
from control import OI


class OpInputCommand (Command):
    def __init__ (self, robot):
        super().__init__ ()
        self.requires(systems.intake)
        self.robot = robot

    def initialize(self):
        pass

    def execute(self):
        oi = OI.get()
        intake = systems.intake

        if oi.intake_is_active() :
            intake.run_intake(self, 1.0)
        elif oi.outtake_is_active() :
            intake.run_intake(self, -1.0)

    def finish(self):
        pass

    def isFinished(self):
        return False