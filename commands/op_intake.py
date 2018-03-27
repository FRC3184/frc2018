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

        if oi.intake_is_active():
            # intake.set_grab_state(GrabState.OUT)
            pwr = oi.get_outtake_command()
            if abs(pwr) < 0.05:
                pwr = 0
            max_intake_power = 0.5
            if pwr > max_intake_power:
                pwr = max_intake_power
            intake.run_intake(pwr)
            if intake.has_acquired_cube() and pwr > 0:
                oi.rumble_op()
            else:
                oi.unrumble_op()
        else:
            intake.run_intake(0)
        if oi.arm_is_open():
            intake.set_grab_state(GrabState.OUT)
        else:
            intake.set_grab_state(GrabState.IN)

    def end(self):
        OI.get().unrumble_op()

    def isFinished(self):
        return False