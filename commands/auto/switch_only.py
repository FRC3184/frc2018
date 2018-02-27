import hal
from wpilib.command import CommandGroup, ConditionalCommand

from commands.auto_intake import MoveIntakeCommand, TimedRunIntakeCommand
from commands.auto_move_elevator import MoveElevatorCommand
from commands.pursuit_drive import PursuitDriveCommand
from control import game_data, pursuit, pose
from control.pose import Pose
from mathutils import Vector2
from systems.drivetrain import Drivetrain
from systems.elevator import Elevator, ElevatorPositions
from systems.intake import Intake, ArmState


class SwitchOnly(CommandGroup):
    def __init__(self, drive: Drivetrain, elevator: Elevator, intake: Intake):
        super().__init__("SwitchOnly command")
        drive_path_waypoints = [Vector2(1.5, 0), Vector2(4.5, 0), Vector2(4.5, 4), Vector2(9.5, 4)]
        cruise = 0.6
        acc = 0.6
        lookahead = 3
        drive_path_left = PursuitDriveCommand(acc=acc, cruise_speed=cruise,
                                               waypoints=drive_path_waypoints, drive=drive, lookahead_base=lookahead)
        drive_path_right = PursuitDriveCommand(acc=acc, cruise_speed=cruise,
                                              waypoints=pursuit.flip_waypoints_y(drive_path_waypoints),
                                              drive=drive, lookahead_base=lookahead)
        drive_path_chooser = ConditionalCommand("SwitchOnlySideCondition")
        drive_path_chooser.onFalse = drive_path_right
        drive_path_chooser.onTrue = drive_path_left
        drive_path_chooser.condition = lambda: game_data.get_own_switch_side() == game_data.Side.LEFT

        elevator_to_height = MoveElevatorCommand(elevator, ElevatorPositions.SWITCH)
        intake_out = MoveIntakeCommand(intake, ArmState.DOWN)
        drop_cube = TimedRunIntakeCommand(intake, time=0.5, power=-intake.power)

        if not hal.isSimulation():
            self.addParallel(elevator_to_height)
        self.addSequential(drive_path_chooser)

        self.addSequential(intake_out)
        self.addSequential(drop_cube)

    def initialize(self):
        pose.set_new_pose(Pose(1.5, 0, 0))