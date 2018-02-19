import commandbased.flowcontrol as fc
import hal
from wpilib.command import CommandGroup, ConditionalCommand

from commands.IntakeCommands import MoveIntakeCommand, TimedRunIntakeCommand
from commands.move_elevator import MoveElevatorCommand
from commands.pursuit_drive import PursuitDriveCommand
from control import GameData, pursuit
from mathutils import Vector2
from systems.drivetrain import Drivetrain
from systems.elevator import Elevator, ElevatorPositions
from systems.intake import Intake, ArmState


class SwitchOnly(CommandGroup):
    def __init__(self, drive: Drivetrain, elevator: Elevator, intake: Intake):
        super().__init__("SwitchOnly command")
        drive_path_waypoints = [Vector2(0, 0), Vector2(3, 0), Vector2(3, 4), Vector2(8, 4)]
        cruise = 0.6
        acc = 0.6
        lookahead = 2
        drive_path_left = PursuitDriveCommand(acc=acc, cruise_speed=cruise,
                                               waypoints=drive_path_waypoints, drive=drive, lookahead_base=lookahead)
        drive_path_right = PursuitDriveCommand(acc=acc, cruise_speed=cruise,
                                              waypoints=pursuit.flip_waypoints_y(drive_path_waypoints),
                                              drive=drive, lookahead_base=lookahead)
        drive_path_chooser = ConditionalCommand("SwitchOnlySideCondition")
        drive_path_chooser.onFalse = drive_path_right
        drive_path_chooser.onTrue = drive_path_left
        drive_path_chooser.condition = lambda: GameData.get_own_switch_side() == GameData.Side.LEFT

        elevator_to_height = MoveElevatorCommand(elevator, ElevatorPositions.SWITCH)
        intake_out = MoveIntakeCommand(intake, ArmState.DOWN)
        drop_cube = TimedRunIntakeCommand(intake, time=0.5, power=-intake.power)

        if not hal.isSimulation():
            self.addParallel(elevator_to_height)
        self.addSequential(drive_path_chooser)

        self.addSequential(intake_out)
        self.addSequential(drop_cube)