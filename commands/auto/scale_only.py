import hal
import math

from py_pursuit_pathing import pursuit
from py_pursuit_pathing.pursuit import InterpolationStrategy
from wpilib.command import CommandGroup, ConditionalCommand, PrintCommand, TimedCommand, Command

from commands.auto_intake import MoveIntakeCommand, TimedRunIntakeCommand, SetIntakeCommand, AcquireCube
from commands.auto_move_elevator import MoveElevatorCommand
from commands.auto_simple_drive import DistanceDriveCommand
from commands.pursuit_drive import PursuitDriveCommand
from commands.spline_drive import SplineDriveCommand
from commands.turn_to_angle import TurnToLookat
from commands.wait_until import WaitUntilConditionCommand
from control import game_data, pose_estimator
from control.game_data import Side
from control.pose_estimator import Pose
from mathutils import Vector2
from systems.drivetrain import Drivetrain
from systems.elevator import Elevator, ElevatorPositions
from systems.intake import Intake, ArmState, GrabState


close_drive = None
far_drive = None
close_drive_flipped = None
far_drive_flipped = None


def init_paths(drive):
    global close_drive, far_drive, close_drive_flipped, far_drive_flipped
    cruise = 6
    acc = 10
    margin = 3 / 12
    lookahead = 2

    if None not in (close_drive, far_drive, far_drive_flipped, close_drive_flipped):
        return
    close_waypoints = [Pose(x=1.5, y=-10.0, heading=0.0),
                           Pose(x=16.5, y=-10.0, heading=0.0),
                           Pose(x=23.5, y=-8.0, heading=0.0)]
    far_waypoints = [Pose(x=1.5, y=-10.0, heading=0.0), Pose(x=16.0, y=-10.0, heading=0.0), Pose(x=20.0, y=-6.0, heading=1.5707963267948966), Pose(x=20.0, y=5.0, heading=1.5707963267948966), Pose(x=24.0, y=7.0, heading=0.0)]


    strategy = InterpolationStrategy.BIARC

    close_drive = PursuitDriveCommand(drive=drive, waypoints=close_waypoints,
                                      cruise_speed=cruise, acc=acc,
                                      dist_margin=margin, lookahead_base=lookahead,
                                      interpol_strat=strategy)
    far_drive = PursuitDriveCommand(drive=drive, waypoints=far_waypoints,
                                    cruise_speed=cruise, acc=acc,
                                    dist_margin=margin, lookahead_base=lookahead,
                                    interpol_strat=strategy)
    close_drive_flipped = PursuitDriveCommand(drive=drive,
                                              waypoints=pursuit.flip_waypoints_y(close_waypoints),
                                              cruise_speed=cruise, acc=acc,
                                              dist_margin=margin, lookahead_base=lookahead,
                                              interpol_strat=strategy)
    far_drive_flipped = PursuitDriveCommand(drive=drive, waypoints=pursuit.flip_waypoints_y(far_waypoints),
                                            cruise_speed=cruise, acc=acc,
                                            dist_margin=margin, lookahead_base=lookahead,
                                            interpol_strat=strategy)

def get_scale_only_group(drive, elevator, intake):
    group = CommandGroup()
    is_close = game_data.get_robot_side() == game_data.get_scale_side()

    if game_data.get_robot_side() == Side.LEFT:
        if is_close:
            drive_command = close_drive_flipped
        else:
            drive_command = far_drive_flipped
    else:
        if is_close:
            drive_command = close_drive
        else:
            drive_command = far_drive

    elev_wait = TimedCommand(name="Elev Timeout", timeoutInSeconds=(0.5 if is_close else 2))

    elevator_to_height = MoveElevatorCommand(elevator, 60)
    elev_group = CommandGroup()
    elev_group.addSequential(elev_wait)
    if not hal.isSimulation():
        elev_group.addSequential(elevator_to_height)
    else:
        elev_group.addSequential(PrintCommand("Elevator moving"))

    drop_cube = SetIntakeCommand(intake, GrabState.OUT)
    drive_back = DistanceDriveCommand(drive=drive, power=-0.2, distance=3)

    group.addParallel(elev_group)
    group.addSequential(drive_command)

    group.addSequential(drop_cube)

    group.addSequential(drive_back)
    return group


class ScaleOnly(Command):
    def __init__(self, drive: Drivetrain, elevator: Elevator, intake: Intake):
        super().__init__("ScaleChooser")
        self.drive = drive
        self.elevator = elevator
        self.intake = intake
        self.group = None
        init_paths(drive)

    def initialize(self):
        pose_estimator.set_new_pose(Pose(x=1.5, y=-10 * (1 if game_data.get_robot_side() == Side.RIGHT else -1),
                                         heading=0))
        self.group = get_scale_only_group(self.drive, self.elevator, self.intake)

        self.group.start()

    def isFinished(self):
        return self.group.isFinished()

    def end(self):
        print("Done")


class DoubleScale(Command):
    def __init__(self, drive: Drivetrain, elevator: Elevator, intake: Intake):
        super().__init__("DoubleScale")
        self.drive = drive
        self.elevator = elevator
        self.intake = intake
        self.group = None
        init_paths(drive)

    def initialize(self):
        pose_estimator.set_new_pose(Pose(x=1.5, y=-10 * (1 if game_data.get_robot_side() == Side.RIGHT else -1),
                                         heading=0))


        self.group = get_scale_only_group(self.drive, self.elevator, self.intake)
        self.group.addSequential(PrintCommand("Done with only scale"))

        if not hal.isSimulation():
            self.group.addParallel(MoveElevatorCommand(self.elevator, 0))
        else:
            self.group.addParallel(PrintCommand("Elevator moving"))
        on_left = game_data.get_scale_side() == Side.LEFT
        self.group.addSequential(TurnToLookat(self.drive, lookat=Vector2(16, 5 * (1 if on_left else -1))))

        self.group.addSequential(AcquireCube(drive=self.drive, drive_speed=0.4, intake=self.intake, timeout=0.7))
        if not hal.isSimulation():
            self.group.addParallel(MoveElevatorCommand(self.elevator, 80))
        else:
            self.group.addParallel(PrintCommand("Elevator moving"))
        self.group.addSequential(DistanceDriveCommand(drive=self.drive, power=-0.5, distance=2))
        self.group.addSequential(TurnToLookat(self.drive, lookat=Vector2(23, 6 * (1 if on_left else -1))))
        self.group.addSequential(DistanceDriveCommand(drive=self.drive, power=0.5, distance=2))
        self.group.addSequential(SetIntakeCommand(intake=self.intake, new_state=GrabState.OUT))

        self.group.start()

    def isFinished(self):
        return self.group.isFinished()