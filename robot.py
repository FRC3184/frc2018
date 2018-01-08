#!/usr/bin/env python3

import wpilib

import OI
from TimedCommandBasedRobot import TimedCommandBasedRobot
from dashboard import dashboard2
from dashboard.dashboard2 import DashboardUpdateCommand


class MyRobot(TimedCommandBasedRobot):
    def __init__(self):
        super().__init__()

    def robotInit(self):
        # Start up continuous processes
        OI.init()
        # In simulation, cd is code dir. On the robot, it's something else so we need to do abs dir
        if wpilib.hal.isSimulation():
            basedir = ""
        else:
            basedir = "/home/lvuser/py"
        dashboard2.run(basedir)

        self.scheduler.add(DashboardUpdateCommand())

    def disabledInit(self):
        pass

    def autonomousInit(self):
        pass

    def teleopInit(self):
        pass


if __name__ == '__main__':
    wpilib.run(MyRobot)