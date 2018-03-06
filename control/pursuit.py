from typing import List, Tuple, Optional

import hal

from control import pose
from mathutils import LineSegment, Vector2

if hal.isSimulation():
    from pyfrc.sim import get_user_renderer


def flip_waypoints_x(waypoints: List[Vector2]):
    waypoints_ = []
    for k in waypoints:
        waypoints_.append(Vector2(-k.x, k.y))
    return waypoints_


def flip_waypoints_y(waypoints: List[Vector2]):
    waypoints_ = []
    for k in waypoints:
        waypoints_.append(Vector2(k.x, -k.y))
    return waypoints_


class Path:
    def __init__(self):
        pass

    def calc_goal(self, pose: pose.Pose,
                  lookahead_radius: float,
                  unpassed_waypoints: List[Vector2]) -> Tuple[Vector2, float]:
        pass


class LinePath(Path):
    """
    Represents a path made of line segments, built between waypoints
    """
    def __init__(self, waypoints: List[Vector2]):
        super().__init__()
        self.path = []
        self.end_point = waypoints[-1]

        # Build a path of segments
        for k in range(len(waypoints) - 1):
            self.path += [LineSegment(waypoints[k], waypoints[k + 1])]

    @staticmethod
    def calc_intersect(point_on_line: Vector2, line: LineSegment, dist: float, lookahead: float, limit_segment=True) \
            -> Optional[Vector2]:
        """
        Calculate the intersect point of the lookahead circle with the given line, if one exists
        :param point_on_line:
        :param line:
        :param dist:
        :param lookahead:
        :return: The intersection point of the lookahead circle
        """
        if dist > lookahead:
            return None
        t = line.invert(point_on_line)
        d = (lookahead ** 2 - dist ** 2) ** 0.5
        if line.in_segment(t + d) or not limit_segment:
            return line.r(t + d)
        return None

    def calc_goal(self, pose: pose.Pose,
                  lookahead_radius: float,
                  unpassed_waypoints: List[Vector2]) -> Tuple[Vector2, float]:
        """
        Calculate the goal point in order to calculate curvature
        This takes whichever of these is first:
        1. The point on the path that intersects with the lookahead circle (checking line segments in order)
        2. The closest point on the path

        If we are approaching the goal, the controller extends the line further past the goal.

        :param pose:
        :param lookahead_radius:
        :param unpassed_waypoints:
        :return: The goal and the distance to the goal
        """
        project_points = []
        goals = []
        goal = None
        error = None

        # Project the robot's pose onto each line to find the closest line to the robot
        # If we can't find a point that intersects the lookahead circle, use the closest point
        for line in self.path:
            project = line.projected_point(pose)

            dist = project.distance(pose)
            project_points += [(project, dist)]
            if goal is None:
                is_last_line = line == self.path[-1]
                goal = self.calc_intersect(project, line, dist, lookahead_radius, limit_segment=(not is_last_line))
                error = dist
        # Choose the closest point
        if goal is None:
            project_points = sorted(project_points, key=lambda x: x[1])
            goal, error = project_points[0]
        return goal, error


class PurePursuitController:
    def __init__(self, waypoints: List[Vector2], lookahead_base: float):
        self.lookahead_base = lookahead_base
        self.path = LinePath(waypoints)
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
        return self.lookahead_base * (base_ratio + (1 - base_ratio) * speed)

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
            render.draw_line([(pose.x, -pose.y + 13.5), (goal.x, -goal.y + 13.5)], color="#0000ff", arrow=False)
        goaly = goal_rs.y
        goal_rs_dist = goal_rs.distance(Vector2(0,0))
        assert abs(goal_rs_dist - lookahead_radius) < 1e-3
        try:
            curv = -2 * goaly / dist ** 2
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
