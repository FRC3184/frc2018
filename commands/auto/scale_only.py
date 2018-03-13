import hal
from wpilib.command import CommandGroup, ConditionalCommand, PrintCommand, TimedCommand, Command

from commands.auto_intake import MoveIntakeCommand, TimedRunIntakeCommand, OpenIntakeCommand
from commands.auto_move_elevator import MoveElevatorCommand
from commands.auto_simple_drive import DistanceDriveCommand
from commands.pursuit_drive import PursuitDriveCommand
from commands.wait_until import WaitUntilConditionCommand
from control import game_data, pursuit, pose
from control.game_data import Side
from control.pose import Pose
from mathutils import Vector2
from systems.drivetrain import Drivetrain
from systems.elevator import Elevator, ElevatorPositions
from systems.intake import Intake, ArmState, GrabState


class ScaleOnly(CommandGroup):
    def __init__(self, drive: Drivetrain, elevator: Elevator, intake: Intake):
        super().__init__("ScaleOnly command")
        close_waypoints = [Vector2(0, -10), Vector2(16.5, -10), Vector2(22.5, -8)]
        far_waypoints = [Vector2(0, -10), Vector2(17, -10), Vector2(17, 7.5), Vector2(22.5, 8)]
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
        drive_path_chooser.condition = lambda: game_data.get_scale_side() == game_data.Side.LEFT

        drive_path_flip_chooser = ConditionalCommand("FlipScaleOnlySideCondition")
        drive_path_flip_chooser.onFalse = drive_path_close_flipped
        drive_path_flip_chooser.onTrue = drive_path_far_flipped
        drive_path_flip_chooser.condition = lambda: game_data.get_scale_side() == game_data.Side.RIGHT

        drive_flip_chooser = ConditionalCommand("DriveFlipCondition")
        drive_flip_chooser.condition = lambda: game_data.get_robot_side() == game_data.Side.RIGHT
        drive_flip_chooser.onTrue = drive_path_chooser
        drive_flip_chooser.onFalse = drive_path_flip_chooser

        wait_short = TimedCommand(name="ShortTimeout", timeoutInSeconds=1.25)
        wait_long = TimedCommand(name="LongTimeout", timeoutInSeconds=4)
        elevator_condition = ConditionalCommand("Elevator Wait Condition")
        elevator_condition.onTrue = wait_short
        elevator_condition.onFalse = wait_long
        elevator_condition.condition = lambda: game_data.get_robot_side() == game_data.get_scale_side()

        elevator_to_height = MoveElevatorCommand(elevator, 80)
        elev_group = CommandGroup()
        elev_group.addSequential(elevator_condition)
        if not hal.isSimulation():
            elev_group.addSequential(elevator_to_height)
        else:
            elev_group.addSequential(PrintCommand("Elevator moving"))

        # intake_out = MoveIntakeCommand(intake, ArmState.DOWN)
        drop_cube = OpenIntakeCommand(intake, GrabState.OUT)

        # reset_pose = ResetPoseCommand()
        drive_back = DistanceDriveCommand(drive=drive, power=-0.5, distance=2)

        self.addParallel(elev_group)
        self.addSequential(drive_flip_chooser)

        # self.addSequential(intake_out)
        self.addSequential(drop_cube)

        # self.addSequential(reset_pose)
        self.addSequential(drive_back)

    def initialize(self):
        pose.set_new_pose(Pose(x=1.5, y=-10 * (1 if game_data.get_robot_side() == Side.RIGHT else -1),
                               heading=0))

class ScaleOnlyChooser(Command):
    def __init__(self, drive: Drivetrain, elevator: Elevator, intake: Intake):
        super().__init__("ScaleChooser")
        self.drive = drive
        self.elevator = elevator
        self.intake = intake
        self.scale_command = ScaleOnly(drive, elevator, intake)
        self.drive_command = PursuitDriveCommand(acc=0.6, cruise_speed=0.6,
                                                 waypoints=[Vector2(0, 0), Vector2(10, 0)],
                                                 drive=drive)


    def initialize(self):
        print(f"{game_data.get_robot_side()} {game_data.get_scale_side()}")
        if game_data.get_robot_side() == game_data.get_scale_side():
            self.scale_command.start()
        else:
            self.drive_command.start()

    def isFinished(self):
        return True