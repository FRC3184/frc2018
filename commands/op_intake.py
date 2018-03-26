from wpilib.command import Command

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

        if oi.arm_is_down():
            intake.set_arm_state(ArmState.DOWN)
        else:
            intake.set_arm_state(ArmState.UP)

        if oi.arm_is_open():
            intake.set_grab_state(GrabState.OUT)
            intake.intake()
            if intake.has_acquired_cube():
                oi.rumble_op()
            else:
                oi.unrumble_op()
        else:
            oi.unrumble_op()
            intake.set_grab_state(GrabState.IN)
            if oi.outtake_is_active():
                pwr = oi.get_outtake_command()
                if abs(pwr) < 0.05:
                    pwr = 0
                intake.run_intake(pwr)
            else:
                intake.run_intake(0)

    def end(self):
        OI.get().unrumble_op()

    def isFinished(self):
        return False