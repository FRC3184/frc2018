#!/usr/bin/env python3

import wpilib
from ctre.talonsrx import TalonSRX

from commands.auto.SwitchOnly import SwitchOnly
from commands.move_elevator import MoveElevatorCommand
from commands.op_drive import OpDriveCommand
from commands.op_elevator import OpElevatorManualCommand
from commands.op_intake import OpIntakeCommand
from commands.zero_elevator import ElevatorZeroCommand
from control import OI
import systems
from control.OI import OIUpdateCommand
from control.TimedCommandBasedRobot import TimedCommandBasedRobot
from dashboard import dashboard2
from dashboard.dashboard2 import DashboardUpdateCommand
from systems import *
from systems.drivetrain import Drivetrain
from systems.elevator import Elevator
from systems.intake import Intake


class MyRobot(TimedCommandBasedRobot):
    def __init__(self):
        super().__init__()
        # Initialize subsystems
        self.drivetrain = Drivetrain()
        self.elevator = Elevator()
        self.intake = Intake()

        self.teleop_drive = OpDriveCommand(self.drivetrain, self.elevator)
        self.telop_intake = OpIntakeCommand(self.intake)

        self.auto_chooser = None

    def robotInit(self):
        # Start up continuous processes
        # In simulation, cd is code dir. On the robot, it's something else so we need to use abs dir
        if wpilib.hal.isSimulation():
            basedir = ""
        else:
            basedir = "/home/lvuser/py"
        dashboard2.run(basedir)

        DashboardUpdateCommand().start()
        OIUpdateCommand().start()

        # Actions

        elev_manual_command = OpElevatorManualCommand(self.elevator)
        elev_move_to_top = MoveElevatorCommand(self.elevator, 60)
        elev_move_to_bottom = MoveElevatorCommand(self.elevator, 0)
        elev_zero = ElevatorZeroCommand(self.elevator)

        OI.get().exec_while_condition(condition=OI.get().elevator_is_manual_control, cmd=elev_manual_command)
        OI.get().add_action_listener(condition=OI.get().elevator_move_to_bottom, action=elev_move_to_bottom.start)
        OI.get().add_action_listener(condition=OI.get().elevator_move_to_top, action=elev_move_to_top.start)

        OI.get().add_action_listener(condition=OI.get().elevator_zero, action=elev_zero.start)

        # Auto modes
        auto_switch_only = SwitchOnly(drive=self.drivetrain, elevator=self.elevator, intake=self.intake)
        self.auto_chooser = dashboard2.add_chooser("Autonomous")
        self.auto_chooser.add_option("Switch Only", auto_switch_only)
        self.auto_chooser.set_default("Switch Only")

    def disabledInit(self):
        pass

    def autonomousInit(self):
        self.auto_chooser.get_selected().start()

    def teleopInit(self):

        self.teleop_drive.start()
        self.telop_intake.start()


if __name__ == '__main__':
    wpilib.run(MyRobot, physics_enabled=True)