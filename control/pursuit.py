from typing import List, Tuple, Optional

import hal

import mathutils
from control import pose
from control.pose import Pose
from control.splines import ComboSpline, CubicSpline, LinearSpline
from mathutils import LineSegment, Vector2

if hal.isSimulation():
    from pyfrc.sim import get_user_renderer


def flip_waypoints_x(waypoints: List[Vector2]):
    waypoints_ = []
    for k in waypoints:
        waypoints_.append(Vector2(-k.x, k.y))
    return waypoints_


def flip_waypoints_y(waypoints: List[Pose]):
    waypoints_ = []
    for k in waypoints:
        waypoints_.append(Pose(k.x, -k.y, -k.heading))
    return waypoints_


class Path:
    def __init__(self):
        pass

    def calc_goal(self, pose: pose.Pose,
                  lookahead_radius: float,
                  unpassed_waypoints: List[Vector2]) -> Tuple[Vector2, float]:
        pass


class SplinePath(Path):
    def __init__(self, waypoints: List[Pose], interpolation_strategy: int):
        super().__init__()
        print(f"Reticulating {interpolation_strategy} of length {len(waypoints)}")
        self.path = waypoints[:]
        if interpolation_strategy == InterpolationStrategy.COMBO4_5:
            self.spline = ComboSpline(self.path)
        elif interpolation_strategy == InterpolationStrategy.CUBIC:
            self.spline = CubicSpline(self.path)
        elif interpolation_strategy == InterpolationStrategy.LINEAR:
            self.spline = LinearSpline(self.path)
        else:
            raise ValueError(f"Invalid interpolation strategy {interpolation_strategy}")

    def calc_goal(self, pose: Pose,
                  lookahead_radius: float, unpassed_waypoints):

        # Find closest t to pose
        t_robot = 0
        min_dist_sq = 1e10
        # TODO better numerical method for finding close point, if necessary
        t_granularity = int(self.spline.length * 5)
        for t in range(t_granularity):
            t = t/t_granularity
            pt = self.spline.get_point(t)
            dist_sq = pt.sq_dist(pose)
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                t_robot = t

        # Find intersection
        t_guess = t_robot + lookahead_radius / self.spline.length
        pt = self.spline.get_point(t_guess)
        line = mathutils.LineSegment(pose, pt)
        pt = line.r(lookahead_radius)

        dist = pt.distance(pose)

        return pt, dist

class InterpolationStrategy:
    LINEAR = 0
    CUBIC = 1
    QUINTIC = 2
    COMBO4_5 = 3


class PurePursuitController:
    def __init__(self, waypoints: List[Pose],
                 lookahead_base: float,
                 interpol_strat: int = InterpolationStrategy.CUBIC):
        self.lookahead_base = lookahead_base
        self.path = SplinePath(waypoints, interpol_strat)
        self.waypoints = waypoints
        self.unpassed_waypoints = waypoints[:]
        self.end_point = waypoints[-1]

    def init(self):
        """
        Resets passed waypoints
        :return:
        """
        self.unpassed_waypoints = self.waypoints[:]

    def lookahead(self, speed: float) -> float:
        """
        Calculate the lookahead distance based on the robot speed
        :param speed: Robot speed, from 0.0 to 1.0 as a percent of the max speed
        :return: Radius of the lookahead circle
        """
        base_ratio = 3/4
        return self.lookahead_base  # * (base_ratio + (1 - base_ratio) * speed)

    def curvature(self, pose: pose.Pose, speed: float) -> Tuple[float, float]:
        """
        Calculate the curvature of the arc needed to continue following the path
        curvature is 1/(radius of turn)
        :param pose: The robot's pose
        :param speed: The speed of the robot, from 0.0 to 1.0 as a percent of max speed
        :return: The curvature of the path and the cross track error
        """
        lookahead_radius = self.lookahead(speed)

        # We're probably only going to pass one waypoint per loop (or have multiple chances to "pass" a waypoint)
        # We need to keep track of the waypoints so we know when we can go to the end
        for point in self.unpassed_waypoints:
            if pose.distance(point) < lookahead_radius:
                self.unpassed_waypoints.remove(point)
                break

        goal, dist = self.path.calc_goal(pose, lookahead_radius, self.unpassed_waypoints)
        goal_rs = goal.translated(pose)
        if hal.isSimulation():
            render = get_user_renderer()
            if render is not None:  # If running the standalone (pyplot) sim, this is None
                render.draw_line([(pose.x, -pose.y + 14), (goal.x, -goal.y + 14)], color="#0000ff", arrow=False)
        goaly = goal_rs.y
        goal_rs_dist = goal_rs.distance(Vector2(0,0))
        try:
            curv = -2 * goaly / lookahead_radius ** 2
        except ZeroDivisionError:
            curv = 0
        return curv, dist

    def is_approaching_end(self, pose):
        return len(self.unpassed_waypoints) == 0

    def is_at_end(self, pose, dist_margin=3/12):
        """
        See if the robot has completed its path
        :param pose: The robot pose
        :return: True if we have gone around the path and are near the end
        """
        translated_end = self.end_point.translated(pose)
        err = abs(translated_end.x)
        return self.is_approaching_end(pose) and translated_end.x < 0

    def get_endcte(self, pose):
        return self.end_point.translated(pose).y
