import math

import hal
from pyfrc.sim import get_user_renderer
from wpilib.command import Command

from control import pose
from mathutils import Vector2
from systems.drivetrain import Drivetrain


class TurnToAngle(Command):
    def __init__(self, drive: Drivetrain, angle: float, margin: float = 5, delta: bool = True):
        super().__init__("TurnToAngle")
        self.requires(drive)
        self.drive = drive
        self.target_angle = angle
        self.delta = delta
        self.base_angle = 0
        self.margin = margin

    def get_cur_angle(self):
        return self.drive.robotdrive.get_heading()

    def initialize(self):
        self.base_angle = self.get_cur_angle()

    def execute(self):
        err = self.get_cur_angle() - self.target_angle
        if self.delta:
            err -= self.base_angle

        min_output = 0.2
        gain = (0.01/180) * err
        if abs(gain) < min_output:
            gain = math.copysign(min_output, gain)

        self.drive.arcade_drive(0, -gain)

    def isFinished(self):
        return abs(self.get_cur_angle() - self.target_angle) < self.margin

    def end(self):
        self.drive.arcade_drive(0, 0)


class TurnToLookat(Command):
    def __init__(self, drive: Drivetrain, lookat: Vector2, margin: float = 5):
        super().__init__("TurnToLookat")
        self.requires(drive)
        self.drive = drive
        self.lookat = lookat
        self.margin = margin
        self.target_angle = 0

    def initialize(self):
        poz = pose.get_current_pose()
        self.target_angle = (self.lookat - poz).angle() * 180 / math.pi
        if hal.isSimulation():
            render = get_user_renderer()
            if render is not None:  # If running the standalone (pyplot) sim, this is None
                render.draw_line([(poz.x, -poz.y + 14), (self.lookat.x, -self.lookat.y + 14)], color="#00ff00", arrow=False)

    def get_cur_angle(self):
        return self.drive.robotdrive.get_heading()

    def execute(self):
        err = self.get_cur_angle() - self.target_angle

        min_output = 0.2
        gain = (0.01/180) * err
        if abs(gain) < min_output:
            gain = math.copysign(min_output, gain)

        self.drive.arcade_drive(0, -gain)

    def isFinished(self):
        return abs(self.get_cur_angle() - self.target_angle) < self.margin

    def end(self):
        self.drive.arcade_drive(0, 0)