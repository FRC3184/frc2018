from wpilib.command import CommandGroup

from commands.auto_intake import TimedRunIntakeCommand, MoveIntakeCommand
from commands.pursuit_drive import PursuitDriveCommand
from control import pose
from control.pose import Pose
from mathutils import Vector2
from systems.drivetrain import Drivetrain
from systems.elevator import Elevator
from systems.intake import Intake, ArmState


class VaultOnly(CommandGroup):
    def __init__(self, drive: Drivetrain, intake: Intake):
        super().__init__("VaultOnly command")
        drive_path_waypoints = [Vector2(1.5, 0), Vector2(6, 0), Vector2(6, 2.75), Vector2(3, 2.75)]
        cruise = 0.4
        acc = 1
        lookahead = 1
        drive_path = PursuitDriveCommand(acc=acc, cruise_speed=cruise,
                                         waypoints=drive_path_waypoints,
                                         drive=drive, lookahead_base=lookahead)
        intake_out = MoveIntakeCommand(intake, ArmState.DOWN)
        drop_cube = TimedRunIntakeCommand(intake, time=0.5, power=-intake.power)

        self.addParallel(intake_out)
        self.addSequential(drive_path)
        self.addSequential(drop_cube)

    def initialize(self):
        pose.set_new_pose(Pose(1.5, 0, 0))
