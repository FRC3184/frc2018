import hal
from wpilib.command import Command

from control import pose
from control.pursuit import PurePursuitController
from mathutils import Vector2
from systems.drivetrain import Drivetrain


class PursuitDriveCommand(Command):
    def __init__(self, drive: Drivetrain, waypoints: [Vector2], cruise_speed, acc, dist_margin=2/12,
                 lookahead_base=6, reverse=False):
        super().__init__("PursuitDriveCommand", timeout=4)
        self.requires(drive)

        self.drive = drive
        self.margin = dist_margin
        self.cruise_speed = cruise_speed * drive.robotdrive.max_speed
        self.acc = acc * drive.robotdrive.max_speed

        self.accel_dist = (1/2) * self.cruise_speed**2 / self.acc

        self.pp_controller = PurePursuitController(waypoints, lookahead_base=lookahead_base)
        cur_pose = pose.get_current_pose()
        self._begin_pose = Vector2(cur_pose.x, cur_pose.y)
        self._end_pose = waypoints[-1]

        self.reverse = reverse

    def initialize(self):
        self.pp_controller.init()
        print("Started pursuit")
        cur_pose = pose.get_current_pose()
        self._begin_pose = Vector2(cur_pose.x, cur_pose.y)
        print(f"Accel dist: {self.accel_dist}")

        if hal.isSimulation():
            from pyfrc.sim import get_user_renderer
            render = get_user_renderer()
            poz = pose.get_current_pose()
            render.draw_line(line_pts=[(w.x, -w.y + 14) for w in self.pp_controller.waypoints],
                             robot_coordinates=False)

    def execute(self):
        poz = pose.get_current_pose()
        dist_to_end = self._end_pose.distance(poz)
        dist_to_begin = self._begin_pose.distance(poz)
        if dist_to_end < self.accel_dist:
            speed = self.cruise_speed * dist_to_end / self.accel_dist
        elif dist_to_begin < self.accel_dist:
            speed = self.cruise_speed * dist_to_begin / self.accel_dist
        else:
            speed = self.cruise_speed

        min_speed = 0.2 * self.drive.robotdrive.max_speed
        if speed < min_speed:
            speed = min_speed

        speed /= self.drive.robotdrive.max_speed
        curvature, cte = self.pp_controller.curvature(poz, speed)
        speed *= (-1 if self.reverse else 1)
        curvature *= (1 if self.reverse else 1)
        if curvature == 0:
            self.drive.tank_drive(speed, speed)
        else:
            radius = 1/curvature
            self.drive.arc(speed, radius)

    def isFinished(self):
        return self.pp_controller.is_at_end(pose.get_current_pose(), self.margin)

    def end(self):
        print("Ended pursuit")