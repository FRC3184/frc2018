import hal
from wpilib.command import CommandGroup, ConditionalCommand, Command, PrintCommand

from commands.auto_intake import MoveIntakeCommand, TimedRunIntakeCommand, OpenIntakeCommand
from commands.auto_move_elevator import MoveElevatorCommand
from commands.auto_simple_drive import TimeDriveCommand
from commands.pursuit_drive import PursuitDriveCommand
from commands.turn_to_angle import TurnToAngle
from control import game_data, pursuit, pose
from control.game_data import Side
from control.pose import Pose
from mathutils import Vector2
from systems.drivetrain import Drivetrain
from systems.elevator import Elevator, ElevatorPositions
from systems.intake import Intake, ArmState, GrabState


class SwitchOnlyCenter(CommandGroup):
    def __init__(self, drive: Drivetrain, elevator: Elevator, intake: Intake):
        super().__init__("SwitchOnly command")
        drive_path_waypoints = [Pose(x=1.5, y=-1.0, heading=0.0),
                                Pose(x=5.0, y=2.5, heading=0.7853981633974483),
                                Pose(x=9.0, y=4.0, heading=0.0)]
        flipped_path = [Pose(x=1.5, y=-1.0, heading=0.0),
                        Pose(x=5.0, y=-2.5, heading=-1.0471975511965976),
                        Pose(x=9.0, y=-4.0, heading=0.0)]
        cruise = 0.8
        acc = 3
        lookahead = 3
        drive_path_left = PursuitDriveCommand(acc=acc, cruise_speed=cruise,
                                              waypoints=drive_path_waypoints, drive=drive, lookahead_base=3)
        drive_path_right = PursuitDriveCommand(acc=acc, cruise_speed=cruise,
                                              waypoints=flipped_path,
                                              drive=drive, lookahead_base=lookahead)
        drive_path_chooser = ConditionalCommand("SwitchOnlySideCondition")
        drive_path_chooser.onFalse = drive_path_right
        drive_path_chooser.onTrue = drive_path_left
        drive_path_chooser.condition = lambda: game_data.get_own_switch_side() == game_data.Side.LEFT

        elevator_to_height = MoveElevatorCommand(elevator, ElevatorPositions.SWITCH)
        intake_out = MoveIntakeCommand(intake, ArmState.DOWN)
        drop_cube = OpenIntakeCommand(intake, GrabState.OUT)

        if not hal.isSimulation():
            self.addParallel(elevator_to_height)
        self.addSequential(drive_path_chooser)

        self.addSequential(intake_out)
        self.addSequential(drop_cube)

    def initialize(self):
        pose.set_new_pose(Pose(1.5, -1, 0))
        print("started switch center")


class SwitchOnlySideStraight(CommandGroup):
    def __init__(self, drive: Drivetrain, elevator: Elevator, intake: Intake):
        super().__init__("SwitchOnlyFromSide")

        drive_path_waypoints = [Pose(1.5, -10, 0), Pose(13, -10, 0)]
        cruise = 0.8
        acc = 1
        lookahead = 2

        drive_path= PursuitDriveCommand(acc=acc, cruise_speed=cruise,
                                        waypoints=drive_path_waypoints, drive=drive, lookahead_base=lookahead)

        turnL = TurnToAngle(drive=drive,
                           angle=(-90),
                           delta=False)
        turnR = TurnToAngle(drive=drive,
                           angle=(90),
                           delta=False)
        turn_chooser = ConditionalCommand("turn_chooser")
        turn_chooser.onTrue = turnR
        turn_chooser.onFalse = turnL
        turn_chooser.condition = lambda: (game_data.get_own_switch_side() == Side.RIGHT)
        drive_short = TimeDriveCommand(drive=drive, power=0.4, time=0.25)
        elevator_to_height = MoveElevatorCommand(elevator, ElevatorPositions.SWITCH)
        intake_out = MoveIntakeCommand(intake, ArmState.DOWN)
        drop_cube = TimedRunIntakeCommand(intake, time=0.5, power=-intake.power)

        if not hal.isSimulation():
            self.addParallel(elevator_to_height)
        self.addSequential(drive_path)
        self.addSequential(turn_chooser)
        self.addSequential(drive_short)

        self.addSequential(intake_out)
        self.addSequential(drop_cube)

    def initialize(self):
        pose.set_new_pose(Pose(1.5, -10, 0))

class SwitchOnlyMonolith(Command):
    def __init__(self, drive: Drivetrain, elevator: Elevator, intake: Intake):
        super().__init__("SwitchOnly Chooser")
        self.side_command = SwitchOnlySideStraight(drive, elevator, intake)
        self.center_command = SwitchOnlyCenter(drive, elevator, intake)

    def initialize(self):
        if game_data.get_robot_side() == Side.CENTER:
            self.center_command.start()
        else:
            if game_data.get_robot_side() == game_data.get_own_switch_side():
                self.side_command.start()

    def isFinished(self):
        return True


