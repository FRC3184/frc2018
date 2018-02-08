from typing import List, Tuple, Optional
from mathutils import LineSegment, Vector2
from control import pose


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
    def calc_intersect(point_on_line: Vector2, line: LineSegment, dist: float, lookahead: float) \
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
        if line.in_segment(t + d):
            return line.r(t + d)
        return None

    def calc_goal(self, pose: pose.Pose,
                  lookahead_radius: float,
                  unpassed_waypoints: List[Vector2]) -> Tuple[Vector2, float]:
        """
        Calculate the goal point in order to calculate curvature
        This takes whichever of these is first:
        1. The end point of the path, if we have passed all the waypoints and are nearer than the lookahead distance
        2. The point on the path that intersects with the lookahead circle (checking line segments in order)
        3. The closest point on the path

        :param pose:
        :param lookahead_radius:
        :param unpassed_waypoints:
        :return: The goal and the distance to the goal
        """
        project_points = []
        goal = None
        error = 0

        if len(unpassed_waypoints) <= 1:
            end_err = pose.distance(self.end_point)
            if end_err < lookahead_radius:
                return self.end_point, error

        # Project the robot's pose onto each line to find the closest line to the robot
        # If we can't find a point that intersects the lookahead circle, use the closest point
        for line in self.path:
            project = line.projected_point(pose)

            dist = project.distance(pose)
            project_points += [(project, dist)]
            if goal is None:
                goal = self.calc_intersect(project, line, dist, lookahead_radius)
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
        return self.lookahead_base * (0.75 + speed)

    def curvature(self, pose: pose.Pose, speed: float) -> float:
        """
        Calculate the curvature of the arc needed to continue following the path
        curvature is 1/(radius of turn)
        :param pose: The robot's pose
        :param speed: The speed of the robot, from 0.0 to 1.0 as a percent of max speed
        :return: The curvature of the path
        """
        lookahead_radius = self.lookahead(speed)

        # We're probably only going to pass one waypoint per loop (or have multiple chances to "pass" a waypoint)
        # We need to keep track of the waypoints so we know when we can go to the end
        for point in self.unpassed_waypoints:
            if pose.distance(point) < lookahead_radius:
                self.unpassed_waypoints.remove(point)
                break

        goal, dist = self.path.calc_goal(pose, lookahead_radius, self.unpassed_waypoints)
        try:
            curv = -2 * goal.translated(pose).y / dist ** 2
        except ZeroDivisionError:
            curv = 0
        return curv

    def is_approaching_end(self, pose):
        return len(self.unpassed_waypoints) == 0

    def is_at_end(self, pose, dist_margin=1/12):
        """
        See if the robot has completed its path
        :param pose: The robot pose
        :return: True if we have gone around the path and are near the end
        """
        return len(self.unpassed_waypoints) == 0 and pose.distance(self.end_point) < dist_margin
