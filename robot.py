#!/usr/bin/env python3

import wpilib
from wpilib.command import CommandGroup

from commands.auto.scale_only import ScaleOnly, DoubleScale, ScaleSwitchSameSide
from commands.auto.switch_and_scale import SwitchAndScale
from commands.auto.switch_only import SwitchOnlyCenter, SwitchOnlyMonolith
from commands.auto.vault import VaultOnly
from commands.auto_intake import MoveIntakeCommand
from commands.auto_move_elevator import MoveElevatorCommand
from commands.auto_simple_drive import TimeDriveCommand
from commands.climb import Climb
from commands.drop_forks import DropForkliftCommand
from commands.op_drive import OpDriveCommand
from commands.op_elevator import OpElevatorManualCommand
from commands.op_intake import OpIntakeCommand, OpenIntakeCommand, ToggleIntakeCommand, RunIntakeCommand
from commands.pursuit_drive import PursuitDriveCommand
from commands.zero_elevator import ElevatorZeroCommand
from control import OI, game_data
from control.OI import OIUpdateCommand
from control.TimedCommandBasedRobot import TimedCommandBasedRobot
from control.pose_estimator import Pose
from dashboard import dashboard2
from dashboard.dashboard2 import DashboardUpdateCommand
from mathutils import Vector2
from systems.checked_system import CheckFaults
from systems.drivetrain import Drivetrain
from systems.elevator import Elevator
from systems.climber import Climber
from systems.intake import Intake, ArmState
from util import get_basedir


class MyRobot(TimedCommandBasedRobot):
    def __init__(self):
        super().__init__()
        self.setPeriod(0.02)
        wpilib.LiveWindow.disableAllTelemetry()
        # Initialize subsystems
        self.drivetrain = Drivetrain()
        self.elevator = Elevator()
        self.intake = Intake()
        self.forklift = Climber()

        self.teleop_drive = OpDriveCommand(self.drivetrain, self.elevator)
        self.telop_intake = OpIntakeCommand(self.intake)

        self.auto_chooser = None
        self.side_chooser = None

    def robotInit(self):
        # Start up continuous processes
        # In simulation, cd is code dir. On the robot, it's something else so we need to use abs dir
        basedir = get_basedir()
        dashboard2.run(basedir)

        DashboardUpdateCommand().start()
        OIUpdateCommand().start()
        # CheckFaults([self.elevator, self.forklift]).start()

        wpilib.CameraServer.launch('vision.py:main')

        dashboard2.add_graph("Speed", lambda : 0)
        dashboard2.add_graph("CTE", lambda: 0)
        dashboard2.add_graph("Lookahead", lambda: 0)
        dashboard2.add_graph("Goal Distance", lambda: 0)

        # Actions

        elev_manual_command = OpElevatorManualCommand(self.elevator)
        elev_move_to_top = MoveElevatorCommand(self.elevator, 62)
        elev_move_to_bottom = MoveElevatorCommand(self.elevator, 0)
        elev_zero = ElevatorZeroCommand(self.elevator)
        elev_intake_switch = CommandGroup()
        elev_intake_switch.addSequential(MoveElevatorCommand(self.elevator, 30))
        elev_intake_switch.addSequential(MoveIntakeCommand(self.intake, ArmState.DOWN))

        intake_open = OpenIntakeCommand(self.intake)
        intake_toggle = ToggleIntakeCommand(self.intake)
        intake_run = RunIntakeCommand(self.intake)

        drop_forks = DropForkliftCommand(self.forklift)
        climb_cmd = Climb(climber=self.forklift, elevator=self.elevator)

        OI.get().exec_while_condition(condition=OI.get().elevator_is_manual_control, cmd=elev_manual_command)
        OI.get().add_action_listener(condition=OI.get().elevator_move_to_bottom, action=elev_move_to_bottom.start)
        OI.get().add_action_listener(condition=OI.get().elevator_move_to_top, action=elev_move_to_top.start)

        OI.get().add_action_listener(condition=OI.get().elevator_move_to_switch, action=elev_intake_switch.start)

        OI.get().add_action_listener(condition=OI.get().elevator_zero, action=elev_zero.start)

        OI.get().add_action_listener(condition=OI.get().do_drop_forks, action=drop_forks.start)
        OI.get().exec_while_condition(condition=OI.get().do_climb, cmd=climb_cmd)

        OI.get().add_action_listener(condition=OI.get().toggle_arm, action=intake_toggle.start)
        OI.get().exec_while_condition(condition=OI.get().arm_is_open, cmd=intake_open)
        OI.get().exec_while_condition(condition=OI.get().intake_is_active, cmd=intake_run)

        self.side_chooser = dashboard2.add_chooser("Starting Position")
        self.side_chooser.add_option("Left", game_data.Side.LEFT)
        self.side_chooser.add_option("Center", game_data.Side.CENTER)
        self.side_chooser.add_option("Right", game_data.Side.RIGHT)
        self.side_chooser.set_default("Center")

        # Auto modes
        auto_switch_only = SwitchOnlyMonolith(drive=self.drivetrain, elevator=self.elevator, intake=self.intake)
        auto_scale_only = ScaleOnly(drive=self.drivetrain, elevator=self.elevator, intake=self.intake)
        auto_scale_double = DoubleScale(drive=self.drivetrain, elevator=self.elevator, intake=self.intake)
        auto_switch_scale = ScaleSwitchSameSide(drive=self.drivetrain, elevator=self.elevator, intake=self.intake)
        # auto_vault = VaultOnly(drive=self.drivetrain, intake=self.intake)

        auto_drive_simple = TimeDriveCommand(drive=self.drivetrain, power=0.3, time=3)
        self.auto_chooser = dashboard2.add_chooser("Autonomous")
        self.auto_chooser.add_option("Switch Only", auto_switch_only)
        self.auto_chooser.add_option("Scale Only", auto_scale_only)
        self.auto_chooser.add_option("2x Scale", auto_scale_double)
        self.auto_chooser.add_option("Scale -> Switch", auto_switch_scale)
        # self.auto_chooser.add_option("Switch and Scale", auto_switch_scale)
        # self.auto_chooser.add_option("Vault Only", auto_vault)
        self.auto_chooser.add_option("Drive Forward", PursuitDriveCommand(acc=4, cruise_speed=8,
                                                                          waypoints=[Pose(1.5, 0, 0),
                                                                                     Pose(10, 0, 0)],
                                                                          drive=self.drivetrain))
        self.auto_chooser.set_default("Drive Forward")

    def disabledInit(self):
        pass

    def autonomousInit(self):
        self.drivetrain.robotdrive.ahrs.reset()
        self.drivetrain.set_brake(True)
        self.drivetrain.set_ramp(0)
        game_data.init(self.side_chooser.get_selected())
        self.auto_chooser.get_selected().start()

    def teleopInit(self):
        self.drivetrain.set_brake(True)
        self.drivetrain.set_ramp(1/3)
        self.teleop_drive.start()


if __name__ == '__main__':
    wpilib.run(MyRobot, physics_enabled=True)