import hal
from py_pursuit_pathing import pursuit
from wpilib.command import CommandGroup, ConditionalCommand, PrintCommand

from commands.auto_intake import MoveIntakeCommand, TimedRunIntakeCommand
from commands.auto_move_elevator import MoveElevatorCommand
from commands.auto_simple_drive import TimeDriveCommand
from commands.pursuit_drive import PursuitDriveCommand
from commands.turn_to_angle import TurnToAngle
from commands.wait_until import WaitUntilConditionCommand
from control import game_data, pose_estimator
from control.pose_estimator import Pose
from mathutils import Vector2
from systems.drivetrain import Drivetrain
from systems.elevator import Elevator, ElevatorPositions
from systems.intake import Intake, ArmState


class SwitchAndScale(CommandGroup):
    def __init__(self, drive: Drivetrain, elevator: Elevator, intake: Intake):
        super().__init__("ScaleOnly command")
        close_waypoints = [Vector2(0, -10), Vector2(16, -10), Vector2(23, -7.5)]
        far_waypoints = [Vector2(0, -10), Vector2(20, -10), Vector2(20, 7), Vector2(23, 7.5)]
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
        drive_path_chooser.condition = lambda: game_data.get_scale_side() == game_data.Side.RIGHT

        drive_path_flip_chooser = ConditionalCommand("FlipScaleOnlySideCondition")
        drive_path_flip_chooser.onFalse = drive_path_close_flipped
        drive_path_flip_chooser.onTrue = drive_path_far_flipped
        drive_path_flip_chooser.condition = lambda: game_data.get_scale_side() == game_data.Side.RIGHT

        drive_flip_chooser = ConditionalCommand("DriveFlipCondition")
        drive_flip_chooser.condition = lambda: game_data.get_robot_side() == game_data.Side.RIGHT
        drive_flip_chooser.onTrue = drive_path_chooser
        drive_flip_chooser.onFalse = drive_path_flip_chooser

        elevator_condition = WaitUntilConditionCommand(lambda: pose_estimator.get_current_pose().x > 16)
        elevator_to_height = MoveElevatorCommand(elevator, ElevatorPositions.SWITCH)
        elev_group = CommandGroup()
        elev_group.addSequential(elevator_condition)
        if not hal.isSimulation():
            elev_group.addSequential(elevator_to_height)
        else:
            elev_group.addSequential(PrintCommand("Elevator moving"))

        intake_out = MoveIntakeCommand(intake, ArmState.DOWN)
        drop_cube = TimedRunIntakeCommand(intake, time=0.5, power=-intake.power)

        drive_back = TimeDriveCommand(drive, time=0.2, power=-0.5)

        spin_chooser = ConditionalCommand("SpinChooser")
        spin_chooser.condition = lambda: game_data.get_scale_side() == game_data.Side.RIGHT
        spin_chooser.onTrue = TurnToAngle(drive, 180, delta=False)
        spin_chooser.onFalse = TurnToAngle(drive, -180, delta=False)

        switch_path_close = [Vector2(20, -10), Vector2(19, -8)]
        switch_path_far = [Vector2(20, -10), Vector2(18, 7)]

        switch_path_chooser = ConditionalCommand("SwitchPathChooser")
        switch_path_chooser.condition = lambda: game_data.get_own_switch_side() == game_data.get_robot_side()
        switch_path_chooser.onTrue = PursuitDriveCommand(drive, switch_path_close, cruise, acc)
        switch_path_chooser.onFalse = PursuitDriveCommand(drive, switch_path_far, cruise, acc)

        self.addParallel(elev_group)
        self.addSequential(drive_flip_chooser)

        self.addSequential(intake_out)
        self.addSequential(drop_cube)

        self.addSequential(drive_back)
        if not hal.isSimulation():
            self.addParallel(MoveElevatorCommand(elevator, 0))
        else:
            self.addParallel(PrintCommand("Elevator moving"))
        self.addSequential(spin_chooser)
        self.addSequential(switch_path_chooser)


    def initialize(self):
        pose_estimator.set_new_pose(Pose(x=1.5, y=-10, heading=0))
