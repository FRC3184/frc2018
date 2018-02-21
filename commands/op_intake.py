from wpilib.command import Command

import systems
from control import OI
from systems.intake import Intake, ArmState, GrabState


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
            intake.intake()
        elif oi.outtake_is_active():
            intake.eject()
        else:
            intake.run_intake(0)

        if oi.arm_is_down():
            intake.set_arm_state(ArmState.DOWN)
        else:
            intake.set_arm_state(ArmState.UP)

        if oi.arm_is_open():
            intake.set_grab_state(GrabState.OUT)
        else:
            intake.set_grab_state(GrabState.IN)

    def end(self):
        pass

    def isFinished(self):
        return False