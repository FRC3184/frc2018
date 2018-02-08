from wpilib.command import Command

from control import pose
from control.pursuit import PurePursuitController
from mathutils import Vector2
from systems.drivetrain import Drivetrain


class PursuitDriveCommand(Command):
    def __init__(self, drive: Drivetrain, waypoints: [Vector2], cruise_speed, acc, dist_margin=2/12):
        super().__init__("PursuitDriveCommand")
        self.requires(drive)

        self.drive = drive
        self.margin = dist_margin
        self.cruise_speed = cruise_speed
        self.acc = acc

        self.accel_dist = (1/2) * self.cruise_speed**2 / self.acc

        self.pp_controller = PurePursuitController(waypoints, lookahead_base=1)
        self._begin_pose = pose.get_current_pose()
        self._end_pose = waypoints[-1]

    def initialize(self):
        self.pp_controller.init()

    def execute(self):
        poz = pose.get_current_pose()
        dist_to_end = self._end_pose.distance(poz)
        dist_to_begin = self._begin_pose.distance(poz)

        if self.pp_controller.is_approaching_end(poz) and dist_to_end < self.accel_dist:
            speed = self.cruise_speed * dist_to_end / self.accel_dist
        elif dist_to_begin < self.accel_dist:
            speed = self.cruise_speed * dist_to_end / self.accel_dist
        else:
            speed = self.cruise_speed

        curvature = self.pp_controller.curvature(poz, speed)

        self.drive.curvature_drive(speed, curvature)

    def isFinished(self):
        return self.pp_controller.is_at_end(pose.get_current_pose(), self.margin)

    def end(self):
        pass