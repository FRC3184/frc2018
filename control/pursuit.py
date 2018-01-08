from typing import List, Tuple, Optional
from mathutils import LineSegment, Vector2
from control import pose


class Path:
    def __init__(self):
        pass

    def calc_goal(self, pose: pose.Pose, lookahead_radius: float) -> Vector2:
        pass


class LinePath(Path):
    def __init__(self, waypoints: List[Vector2]):
        super().__init__()
        self.path = []
        # Build a path of segments
        for k in range(len(waypoints) - 1):
            self.path += [LineSegment(waypoints[k], waypoints[k + 1])]

    @staticmethod
    def calc_intersect(point_on_line: Vector2, line: LineSegment, dist: float, lookahead: float) \
            -> Optional[Vector2]:
        if dist > lookahead:
            return None
        t = line.invert(point_on_line)
        d = (lookahead ** 2 - dist ** 2) ** 0.5
        if line.in_segment(t + d):
            return line.r(t + d)
        return None

    def calc_goal(self, pose: pose.Pose, lookahead_radius: float) -> Tuple[Vector2, float]:
        project_points = []
        goal = None
        error = 0
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
    def __init__(self, pose: pose.Pose, waypoints: List[Vector2], lookahead_base: float):
        self.pose = pose
        self.lookahead_base = lookahead_base
        self.path = LinePath(waypoints)

    def lookahead(self, speed: float) -> float:
        return self.lookahead_base * (0.75 + speed)

    def curvature(self, pose: pose.Pose, speed: float) -> float:
        # Get lookahead point
        # TODO How to: Not overshoot endpoint?
        lookahead = self.lookahead(speed)
        goal, dist = self.path.calc_goal(pose, lookahead)

        curv = -2 * goal.translated(pose).y / lookahead ** 2
        return curv
