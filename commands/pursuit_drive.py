import hal
from wpilib.command import Command

from control import pose
from control.motion_profile import MotionProfile
from control.pose import Pose
from control.pursuit import PurePursuitController, InterpolationStrategy
from mathutils import Vector2
from systems.drivetrain import Drivetrain


class PursuitDriveCommand(Command):
    def __init__(self, drive: Drivetrain, waypoints: [Pose], cruise_speed, acc, dist_margin=2/12,
                 lookahead_base=3, reverse=False, interpol_strat=InterpolationStrategy.CUBIC):
        super().__init__("PursuitDriveCommand", timeout=4)
        self.requires(drive)

        self.drive = drive
        self.margin = dist_margin
        self.cruise_speed = cruise_speed * drive.robotdrive.max_speed
        self.acc = acc * drive.robotdrive.max_speed

        self.pp_controller = PurePursuitController(waypoints=waypoints,
                                                   lookahead_base=lookahead_base,
                                                   interpol_strat=interpol_strat,
                                                   cruise_speed=self.cruise_speed,
                                                   acc=self.acc)
        cur_pose = pose.get_current_pose()
        self._begin_pose = Vector2(cur_pose.x, cur_pose.y)
        self._end_pose = waypoints[-1]

        self.reverse = reverse

    def initialize(self):
        self.pp_controller.init()
        print("Started pursuit")
        cur_pose = pose.get_current_pose()

        if hal.isSimulation():
            from pyfrc.sim import get_user_renderer
            render = get_user_renderer()
            poz = pose.get_current_pose()
            line_pts = []
            for t in range(1000):
                t0 = t / 1000
                pt = self.pp_controller.path.spline.get_point(t0)
                line_pts += [(pt.x, -pt.y + 14)]
            render.draw_line(line_pts, robot_coordinates=False)

    def execute(self):
        poz = pose.get_current_pose()

        curvature, goal, speed = self.pp_controller.curvature(poz)
        speed /= self.drive.robotdrive.max_speed

        min_speed = 0.2
        if speed < min_speed:
            speed = min_speed

        speed *= (-1 if self.reverse else 1)
        curvature *= (1 if self.reverse else 1)
        if curvature == 0:
            self.drive.tank_drive(speed, speed)
        else:
            radius = -1/curvature
            self.drive.arc(speed, radius)

    def isFinished(self):
        return self.pp_controller.is_at_end(pose.get_current_pose(), self.margin)

    def end(self):
        print("Ended pursuit")