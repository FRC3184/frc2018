import hal
import math
from wpilib.command import CommandGroup, ConditionalCommand, PrintCommand, TimedCommand, Command

from commands.auto_intake import MoveIntakeCommand, TimedRunIntakeCommand, OpenIntakeCommand, AcquireCube
from commands.auto_move_elevator import MoveElevatorCommand
from commands.auto_simple_drive import DistanceDriveCommand
from commands.pursuit_drive import PursuitDriveCommand
from commands.turn_to_angle import TurnToLookat
from commands.wait_until import WaitUntilConditionCommand
from control import game_data, pursuit, pose
from control.game_data import Side
from control.pose import Pose
from mathutils import Vector2
from systems.drivetrain import Drivetrain
from systems.elevator import Elevator, ElevatorPositions
from systems.intake import Intake, ArmState, GrabState


def get_scale_only_group(drive, elevator, intake):
    group = CommandGroup()
    is_close = game_data.get_robot_side() == game_data.get_scale_side()

    close_waypoints = [Vector2(0, -10), Vector2(16.5, -10), Vector2(22.5, -8)]
    far_waypoints = [Vector2(0, -10), Vector2(20, -10), Vector2(20, 7)]

    if is_close:
        path = close_waypoints
    else:
        path = far_waypoints

    if game_data.get_robot_side() == Side.LEFT:
        path = pursuit.flip_waypoints_y(path)
    cruise = 0.6
    acc = 1.2
    margin = 3 / 12
    lookahead = 2
    drive_command = PursuitDriveCommand(drive=drive, waypoints=path, cruise_speed=cruise, acc=acc,
                                        dist_margin=margin, lookahead_base=lookahead)

    elev_wait = TimedCommand(name="Elev Timeout", timeoutInSeconds=(0.5 if is_close else 2))

    elevator_to_height = MoveElevatorCommand(elevator, 60)
    elev_group = CommandGroup()
    elev_group.addSequential(elev_wait)
    if not hal.isSimulation():
        elev_group.addSequential(elevator_to_height)
    else:
        elev_group.addSequential(PrintCommand("Elevator moving"))

    drop_cube = OpenIntakeCommand(intake, GrabState.OUT)
    drive_back = DistanceDriveCommand(drive=drive, power=-0.5, distance=2)

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

    def initialize(self):
        pose.set_new_pose(Pose(x=1.5, y=-10 * (1 if game_data.get_robot_side() == Side.RIGHT else -1),
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

    def initialize(self):
        pose.set_new_pose(Pose(x=1.5, y=-10 * (1 if game_data.get_robot_side() == Side.RIGHT else -1),
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
        self.group.addSequential(OpenIntakeCommand(intake=self.intake, new_state=GrabState.OUT))

        self.group.start()

    def isFinished(self):
        return self.group.isFinished()