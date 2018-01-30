#!/usr/bin/env python3

import wpilib
from ctre.talonsrx import TalonSRX

from commands.OpDriveCommand import OpDriveCommand
from control import OI
import systems
from control.TimedCommandBasedRobot import TimedCommandBasedRobot
from dashboard import dashboard2
from dashboard.dashboard2 import DashboardUpdateCommand
from systems import *
from systems.drivetrain import Drivetrain


class MyRobot(TimedCommandBasedRobot):
    def __init__(self):
        super().__init__()
        # Initialize subsystems
        systems.drivetrain = Drivetrain()

    def robotInit(self):
        # Start up continuous processes
        OI.init()
        # In simulation, cd is code dir. On the robot, it's something else so we need to use abs dir
        if wpilib.hal.isSimulation():
            basedir = ""
        else:
            basedir = "/home/lvuser/py"
        dashboard2.run(basedir)

        DashboardUpdateCommand().start()



        self.driveCommand = OpDriveCommand(self)

    def disabledInit(self):
        pass

    def autonomousInit(self):
        pass

    def teleopInit(self):
        self.driveCommand.start()


if __name__ == '__main__':
    wpilib.run(MyRobot)