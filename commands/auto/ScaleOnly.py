import commandbased.flowcontrol as fc
import hal
from wpilib.command import CommandGroup, ConditionalCommand, PrintCommand

from commands.IntakeCommands import MoveIntakeCommand, TimedRunIntakeCommand
from commands.auto.WaitUntilCondition import WaitUntilConditionCommand
from commands.auto.reset_pose import ResetPoseCommand
from commands.move_elevator import MoveElevatorCommand
from commands.pursuit_drive import PursuitDriveCommand
from control import GameData, pursuit, pose
from control.pose import Pose
from mathutils import Vector2
from systems.drivetrain import Drivetrain
from systems.elevator import Elevator, ElevatorPositions
from systems.intake import Intake, ArmState


class ScaleOnly(CommandGroup):
    def __init__(self, drive: Drivetrain, elevator: Elevator, intake: Intake):
        super().__init__("ScaleOnly command")
        close_waypoints = [Vector2(0, 0), Vector2(16, 0), Vector2(22, 3)]
        far_waypoints = [Vector2(0, 0), Vector2(18, 0), Vector2(18, 17), Vector2(20.5, 16.5)]
        cruise = 0.6
        acc = 0.6
        margin = 3/12
        lookahead = 2
        drive_path_far = PursuitDriveCommand(acc=acc, cruise_speed=cruise,
                                             waypoints=far_waypoints, drive=drive, dist_margin=margin,
                                             lookahead_base=lookahead)
        drive_path_close = PursuitDriveCommand(acc=acc, cruise_speed=cruise,
                                               waypoints=close_waypoints,
                                               drive=drive, dist_margin=margin, lookahead_base=lookahead)
        drive_path_far_flipped = PursuitDriveCommand(acc=acc, cruise_speed=cruise,
                                                     waypoints=pursuit.flip_waypoints_y(far_waypoints), drive=drive,
                                                     dist_margin=margin,
                                                     lookahead_base=lookahead)
        drive_path_close_flipped = PursuitDriveCommand(acc=acc, cruise_speed=cruise,
                                                       waypoints=pursuit.flip_waypoints_y(close_waypoints),
                                                       drive=drive, dist_margin=margin, lookahead_base=lookahead)

        drive_path_chooser = ConditionalCommand("StandardScaleOnlySideCondition")
        drive_path_chooser.onFalse = drive_path_far
        drive_path_chooser.onTrue = drive_path_close
        drive_path_chooser.condition = lambda: GameData.get_own_switch_side() == GameData.Side.RIGHT

        drive_path_flip_chooser = ConditionalCommand("FlipScaleOnlySideCondition")
        drive_path_flip_chooser.onFalse = drive_path_close_flipped
        drive_path_flip_chooser.onTrue = drive_path_far_flipped
        drive_path_flip_chooser.condition = lambda: GameData.get_own_switch_side() == GameData.Side.RIGHT

        drive_flip_chooser = ConditionalCommand("DriveFlipCondition")
        drive_flip_chooser.condition = lambda: GameData.get_robot_side() != GameData.Side.RIGHT
        drive_flip_chooser.onTrue = drive_path_chooser
        drive_flip_chooser.onFalse = drive_path_flip_chooser

        elevator_condition = WaitUntilConditionCommand(lambda: pose.get_current_pose().x > 16)
        elevator_to_height = MoveElevatorCommand(elevator, ElevatorPositions.SWITCH)
        elev_group = CommandGroup()
        elev_group.addSequential(elevator_condition)
        if not hal.isSimulation():
            elev_group.addSequential(elevator_to_height)
        else:
            elev_group.addSequential(PrintCommand("Elevator moving"))

        intake_out = MoveIntakeCommand(intake, ArmState.DOWN)
        drop_cube = TimedRunIntakeCommand(intake, time=0.5, power=-intake.power)

        reset_pose = ResetPoseCommand()
        drive_back = PursuitDriveCommand(drive=drive, waypoints=[Vector2(0, 0), Vector2(-2, 0)],
                                                 cruise_speed=0.6, acc=0.2, reverse=True)

        self.addParallel(elev_group)
        self.addSequential(drive_flip_chooser)

        self.addSequential(intake_out)
        self.addSequential(drop_cube)

        self.addSequential(reset_pose)
        self.addSequential(drive_back)

    def initialize(self):
        pass  # pose.set_new_pose(Pose(x=0, y=-10, heading=0))
