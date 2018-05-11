import hal
from py_pursuit_pathing.pursuit import InterpolationStrategy, PurePursuitController
from wpilib.command import Command

from control import pose_estimator
from control.motion_profile import MotionProfile
from control.pose_estimator import Pose
from dashboard import dashboard2
from mathutils import Vector2
from systems.drivetrain import Drivetrain


class PursuitDriveCommand(Command):
    def __init__(self, drive: Drivetrain, waypoints: [Pose], cruise_speed, acc, dist_margin=2/12,
                 lookahead_base=2, reverse=False, interpol_strat=InterpolationStrategy.BIARC):
        super().__init__("PursuitDriveCommand", timeout=4)
        self.requires(drive)

        self.drive = drive
        self.margin = dist_margin
        self.cruise_speed = cruise_speed
        self.acc = acc

        self.pp_controller = PurePursuitController(waypoints=waypoints,
                                                   lookahead_base=lookahead_base,
                                                   interpol_strat=interpol_strat,
                                                   cruise_speed=self.cruise_speed,
                                                   acc=self.acc)
        cur_pose = pose_estimator.get_current_pose()
        self._begin_pose = Vector2(cur_pose.x, cur_pose.y)
        self._end_pose = waypoints[-1]

        self.reverse = reverse
        self.goal_dist = 0
        self.speed = 0

    def initialize(self):
        self.pp_controller.init()
        print("Started pursuit")
        cur_pose = pose_estimator.get_current_pose()

        poz = pose_estimator.get_current_pose()
        line_pts = []
        for t in range(1000):
            t0 = t / 1000
            pt = self.pp_controller.path.spline.get_point(t0)
            line_pts += [(pt.x, -pt.y + 14)]
        dashboard2.draw(list(map(lambda x: Vector2(x[0], -(x[1] - 14)), line_pts)))
        if hal.isSimulation():
            from pyfrc.sim import get_user_renderer
            render = get_user_renderer()
            render.draw_line(line_pts, robot_coordinates=False)

        dashboard2.add_graph("CTE", self.pp_controller.get_cte)
        dashboard2.add_graph("Lookahead", lambda: self.pp_controller.current_lookahead)
        dashboard2.add_graph("Goal Distance", lambda: self.goal_dist)
        dashboard2.add_graph("Speed", lambda: self.speed)

    def execute(self):
        poz = pose_estimator.get_current_pose()

        curvature, goal, speed, goal_pt = self.pp_controller.curvature(poz)
        dashboard2.draw([poz, goal_pt])
        self.goal_dist = goal
        self.speed = speed

        min_speed = 0.5  # feet/s
        if speed < min_speed:
            speed = min_speed

        speed *= (-1 if self.reverse else 1)
        curvature *= (1 if self.reverse else 1)
        if curvature == 0:
            self.drive.straight(speed)
        else:
            radius = -1/curvature
            self.drive.arc(speed, radius)

    def isFinished(self):
        return self.pp_controller.is_at_end(pose_estimator.get_current_pose(), self.margin)

    def end(self):
        print("Ended pursuit")