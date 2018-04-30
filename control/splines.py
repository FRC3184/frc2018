import math
from typing import List, Tuple

import numpy as np

from mathutils import Polynomial, Vector2, polynomial_from_parameters
from control.pose import Pose


class SplinePart:
    def __init__(self, P: Polynomial, x0: float, x1: float, t_begin: float, t_end: float, offset: Pose):
        self.curve = P
        self.begin_x = x0
        self.end_x = x1
        self.length = P.length(x0, x1)
        self.t_begin = t_begin
        self.t_end = t_end
        self.offset = offset

    def get_point(self, t: float) -> Vector2:
        return (Vector2(self._get_x(t), self._get_y(t))).inv_translate(self.offset)

    def get_x(self, t: float) -> float:
        return self.get_point(t).x

    def get_y(self, t: float) -> float:
        return self.get_point(t).y

    def _get_x(self, t: float) -> float:
        return self.end_x * (t - self.t_begin) / (self.t_end - self.t_begin)

    def _get_y(self, t: float) -> float:
        return self.curve.compute(self._get_x(t))

    def slope(self, t: float) -> float:
        return self.curve.slope(self._get_x(t))

    def curvature(self, t: float) -> float:
        return self.curve.second_deriv(self.get_x(t))


class Spline():
    def __init__(self, waypoints: List[Pose]):
        assert len(waypoints) >= 2
        self.waypoints = waypoints[:]
        self.length = 0
        self.parts = []
        self.reticulate()

    def reticulate(self):
        raise NotImplementedError

    def get_part(self, t: float):
        for part in self.parts:
            if part.t_begin <= t <= part.t_end:
                _part = part
                break
        else:
            _part = self.parts[-1]
        return _part

    def get_point(self, t: float):
        if t > 1:
            return self.get_point(1) + Vector2(t / self.length, t * self.get_slope(1) / self.length)
        _part = self.get_part(t)
        return Vector2(_part.get_x(t), _part.get_y(t))

    def get_slope(self, t: float):
        # assert 0 <= t <= 1
        _part = self.get_part(t)
        return _part.slope(t)

    def get_unit_tangent_vector(self, t: float):
        _part = self.get_part(t)
        return Vector2(1, _part.slope(t)).normalized()


class LinearSpline(Spline):
    def __init__(self, waypoints: List[Pose]):
        super().__init__(waypoints)

    def reticulate(self):
        curves = []
        lengths = []
        for n in range(len(self.waypoints) - 1):
            waypoint = self.waypoints[n]
            next_waypoint = self.waypoints[n + 1].translated(waypoint)

            x0 = 0
            y0 = 0
            x1 = next_waypoint.x
            y1 = next_waypoint.y

            def get_linear_system(x0, y0, x1, y1):
                return np.mat([Polynomial.get_row_for_degree(x0, 1),
                               Polynomial.get_row_for_degree(x1, 1)
                               ]), np.mat([[y0], [y1]])

            A, b = get_linear_system(x0, y0, x1, y1)
            curve_constants = np.linalg.solve(A, b)
            P = polynomial_from_parameters(curve_constants)
            curves.append(P)
            lengths.append(P.length(x0, x1))

        self.length = sum(lengths)
        for i in range(len(lengths)):
            lengths[i] /= self.length

        self.parts = []
        t_accum = 0
        for i in range(len(curves)):
            wp = self.waypoints[i]
            n_wp = self.waypoints[i + 1].translated(wp)
            x0 = 0
            x1 = n_wp.x
            t_begin = t_accum
            t_end = t_accum + lengths[i]
            t_accum = t_end
            part = SplinePart(curves[i], x0, x1, t_begin, t_end, wp)

            self.parts.append(part)


class CubicSpline(Spline):
    def __init__(self, waypoints: List[Pose]):
        super().__init__(waypoints)

    def reticulate(self):
        curves = []
        lengths = []
        for n in range(len(self.waypoints) - 1):
            waypoint = self.waypoints[n]
            next_waypoint = self.waypoints[n + 1].translated(waypoint)

            x0 = 0
            y0 = 0
            x1 = next_waypoint.x
            y1 = next_waypoint.y
            t0 = math.tan(0)
            t1 = math.tan(next_waypoint.heading)

            cap_tangent = 1e2
            if abs(t1) > cap_tangent:
                t1 = math.copysign(cap_tangent, t1)
                print(f"Capping large angle at curve {n} -> {n+1}")

            def get_cubic_system(x0, y0, x1, y1, t0, t1):
                return np.mat([Polynomial.get_row_for_degree(x0, 3),
                               Polynomial.get_row_for_degree(x1, 3),
                               Polynomial.get_slope_row_for_degree(x0, 3),
                               Polynomial.get_slope_row_for_degree(x1, 3)
                               ]), np.mat([[y0], [y1], [t0], [t1]])

            A, b = get_cubic_system(x0, y0, x1, y1, t0, t1)
            curve_constants = np.linalg.solve(A, b)
            P = polynomial_from_parameters(curve_constants)
            curves.append(P)
            lengths.append(P.length(x0, x1))

        self.length = sum(lengths)
        for i in range(len(lengths)):
            lengths[i] /= self.length

        self.parts = []
        t_accum = 0
        for i in range(len(curves)):
            wp = self.waypoints[i]
            n_wp = self.waypoints[i + 1].translated(wp)
            x0 = 0
            x1 = n_wp.x
            t_begin = t_accum
            t_end = t_accum + lengths[i]
            t_accum = t_end
            part = SplinePart(curves[i], x0, x1, t_begin, t_end, wp)

            self.parts.append(part)


class ComboSpline(Spline):
    def __init__(self, waypoints: List):
        super().__init__(waypoints)

    def reticulate(self):
        def get_quartic_system(x0, y0, x1, y1, t0, t1, k0, k1):
            return np.mat([Polynomial.get_row_for_degree(x0, 4),
                           Polynomial.get_row_for_degree(x1, 4),
                           Polynomial.get_slope_row_for_degree(x0, 4),
                           Polynomial.get_slope_row_for_degree(x1, 4),
                           Polynomial.get_curvature_row_for_degree(x0, 4),
                           ]), np.mat([[y0], [y1], [t0], [t1], [k0]])

        def get_quintic_system(x0, y0, x1, y1, t0, t1, k0, k1) -> Tuple[np.mat, np.mat]:
            return np.mat([Polynomial.get_row_for_degree(x0, 5),
                           Polynomial.get_row_for_degree(x1, 5),
                           Polynomial.get_slope_row_for_degree(x0, 5),
                           Polynomial.get_slope_row_for_degree(x1, 5),
                           Polynomial.get_curvature_row_for_degree(x0, 5),
                           Polynomial.get_curvature_row_for_degree(x1, 5),
                           ]), np.mat([[y0], [y1], [t0], [t1], [k0], [k1]])

        waypoints = self.waypoints
        curves = []
        lengths = []
        for n in range(len(waypoints) - 1):
            waypoint = waypoints[n]
            next_waypoint = waypoints[n + 1].translated(waypoint)
            quintic_flag = n + 1 == len(waypoints) - 1

            # First waypoint has zero curvature to begin
            if n == 0:
                k0 = 0
            else:
                k0 = curves[-1].second_deriv(waypoint.x)
            k1 = 0  # zero curvature at the end, only used on last spline

            x0 = 0
            y0 = 0
            x1 = next_waypoint.x
            y1 = next_waypoint.y
            t0 = math.tan(0)
            t1 = math.tan(next_waypoint.heading)

            cap_tangent = 1e2
            if abs(t1) > cap_tangent:
                t1 = math.copysign(cap_tangent, t1)
                print(f"Capping large angle at knot {n+1}")

            get_system = get_quintic_system if quintic_flag else get_quartic_system
            A, b = get_system(x0, y0, x1, y1, t0, t1, k0, k1)
            curve_constants = np.linalg.solve(A, b)
            P = polynomial_from_parameters(curve_constants)
            curves.append(P)
            lengths.append(P.length(0, next_waypoint.x))

        self.length = sum(lengths)
        for i in range(len(lengths)):
            lengths[i] /= self.length

        self.parts = []
        t_accum = 0
        for i in range(len(curves)):
            wp = waypoints[i]
            n_wp = waypoints[i + 1].translated(wp)
            x0 = 0
            x1 = n_wp.x
            t_begin = t_accum
            t_end = t_accum + lengths[i]
            t_accum = t_end
            part = SplinePart(curves[i], x0, x1, t_begin, t_end, wp)

            self.parts.append(part)


class QuinticSpline(Spline):
    def __init__(self, waypoints: List):
        super().__init__(waypoints)

    def reticulate(self):
        def get_quintic_system(x0, y0, x1, y1, t0, t1, k0, k1) -> Tuple[np.mat, np.mat]:
            return np.mat([Polynomial.get_row_for_degree(x0, 5),
                           Polynomial.get_row_for_degree(x1, 5),
                           Polynomial.get_slope_row_for_degree(x0, 5),
                           Polynomial.get_slope_row_for_degree(x1, 5),
                           Polynomial.get_curvature_row_for_degree(x0, 5),
                           Polynomial.get_curvature_row_for_degree(x1, 5),
                           ]), np.mat([[y0], [y1], [t0], [t1], [k0], [k1]])

        waypoints = self.waypoints
        curves = []
        lengths = []
        for n in range(len(waypoints) - 1):
            waypoint = waypoints[n]
            next_waypoint = waypoints[n + 1].translated(waypoint)
            quintic_flag = n + 1 == len(waypoints) - 1

            # First waypoint has zero curvature to begin
            k0 = 0
            k1 = 0

            x0 = 0
            y0 = 0
            x1 = next_waypoint.x
            y1 = next_waypoint.y
            t0 = math.tan(0)
            t1 = math.tan(next_waypoint.heading)

            cap_tangent = 1e2
            if abs(t1) > cap_tangent:
                t1 = math.copysign(cap_tangent, t1)
                print(f"Capping large angle at knot {n+1}")

            A, b = get_quintic_system(x0, y0, x1, y1, t0, t1, k0, k1)
            curve_constants = np.linalg.solve(A, b)
            P = polynomial_from_parameters(curve_constants)
            curves.append(P)
            lengths.append(P.length(0, next_waypoint.x))

        self.length = sum(lengths)
        for i in range(len(lengths)):
            lengths[i] /= self.length

        self.parts = []
        t_accum = 0
        for i in range(len(curves)):
            wp = waypoints[i]
            n_wp = waypoints[i + 1].translated(wp)
            x0 = 0
            x1 = n_wp.x
            t_begin = t_accum
            t_end = t_accum + lengths[i]
            t_accum = t_end
            part = SplinePart(curves[i], x0, x1, t_begin, t_end, wp)

            self.parts.append(part)
