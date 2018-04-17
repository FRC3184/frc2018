import math
import numpy as np
from typing import List, Tuple


def sgn(x):
    return 0 if x == 0 else x/abs(x)


def normalize_angle(theta):
    """
    Normalize angle to +/- pi/2
    :param theta: The angle to normalize, in radians
    :return: The normalized angle
    """
    return math.atan2(math.sin(theta), math.cos(theta))


def angle_difference(theta1, theta2):
    return normalize_angle(theta1 - theta2)


def signed_power(x, pow):
    s = sgn(x)
    return s * abs(x)**pow


def deadband(x, deadband):
    if abs(x) < deadband:
        return 0
    return x


def clamp(x, vmin, vmax):
    return min(max(x, vmin), vmax)
  
  
class Vector2:
    """
    A 2D vector. x,y are in world coordinates
    """

    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def translated(self, pose):
        dx = self.x - pose.x
        dy = self.y - pose.y
        c = math.cos(pose.heading)
        s = math.sin(pose.heading)
        dxp = dx * c + dy * s
        dyp = dx * -s + dy * c
        return Vector2(dxp, dyp)

    def inv_translate(self, pose):
        c = math.cos(pose.heading)
        s = math.sin(pose.heading)
        dxp = self.x * c - self.y * s
        dyp = self.x * s + self.y * c
        return Vector2(dxp + pose.x, dyp + pose.y)

    def sq_dist(self, point):
        return (point.x - self.x) ** 2 + (point.y - self.y) ** 2

    def distance(self, point):
        return self.sq_dist(point)**0.5

    def normalized(self):
        magn = abs(self)
        return Vector2(self.x / magn, self.y / magn)

    def __mul__(self, other):
        if type(other) == Vector2:
            return self.x * other.x + self.y * other.y
        else:
            return Vector2(self.x * other, self.y * other)

    def __repr__(self):
        return "Vector({}, {})".format(self.x, self.y)

    def __sub__(self, other):
        from control import pose
        assert type(other) == Vector2 or type(other) == pose.Pose
        return Vector2(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        from control import pose
        assert type(other) == Vector2 or type(other) == pose.Pose
        return Vector2(self.x + other.x, self.y + other.y)

    def __abs__(self):
        return self.distance(Vector2(0, 0))

    def __copy__(self):
        return Vector2(self.x, self.y)

    def __neg__(self):
        return self * -1

    def angle(self):
        return math.atan2(self.y, self.x)


class LineSegment:
    def __init__(self, point1: Vector2, point2: Vector2):
        d = Vector2(point2.x - point1.x, point2.y - point1.y)
        self.slope = d.normalized()
        self.max_t = abs(d)
        self.intersect = point1
        self.point1 = point1
        self.point2 = point2

    def plot(self, plt, resolution=0.1):
        t0 = 0
        t1 = self.max_t
        i = t0
        xx = []
        yy = []
        while i < t1:
            p = self.r(i)
            xx += [p.x]
            yy += [p.y]
            i += resolution
        plt.plot(xx, yy)

    def projected_point(self, point: Vector2):
        pt = point - self.intersect
        a_scalar = pt * self.slope
        return self.slope * a_scalar + self.intersect

    def invert(self, point: Vector2):
        tr_point = point - self.intersect
        if self.slope.x == 0:
            return tr_point.y / self.slope.y
        elif self.slope.y == 0:
            return tr_point.x / self.slope.x
        return (tr_point.x / self.slope.x + tr_point.y / self.slope.y) / 2

    def r(self, t):
        return self.intersect + self.slope * t

    def on_line(self, point: Vector2, epsilon=1e-3):

        tr_point = point - self.intersect
        if self.slope.x == 0:
            return abs(tr_point.x) < epsilon
        elif self.slope.y == 0:
            return abs(tr_point.y) < epsilon
        return abs(tr_point.x / self.slope.x - tr_point.y / self.slope.y) < epsilon

    def in_segment(self, t: float):
        return 0 <= t <= self.max_t

    def __repr__(self):
        return "Line(t<{}, {}> + <{}, {}>".format(self.slope.x, self.slope.y, self.intersect.x, self.intersect.y)

def approximate_curve(x0, y0, xs, s, k):
    M = np.mat([[x0 ** 2, x0, 1], [2 * xs, 1, 0], [2, 0, 0]])
    b = np.mat([[y0], [s], [k]])
    coeff = np.linalg.solve(M, b)
    return polynomial_from_parameters(coeff)


class Polynomial:
    def __init__(self, coefficients: List[float]):
        self.coefficients = coefficients

    def compute(self, x: float) -> float:
        sum = 0
        for i, coeff in enumerate(reversed(self.coefficients)):
            sum += coeff * (x ** i)
        return sum

    def slope(self, x: float) -> float:
        sum = 0
        for i, coeff in enumerate(reversed(self.coefficients)):
            if i == 0:
                continue
            sum += i * coeff * (x ** (i - 1))
        return sum

    def second_deriv(self, x: float) -> float:
        sum = 0
        for i, coeff in enumerate(reversed(self.coefficients)):
            if i == 0 or i == 1:
                continue
            sum += i * (i - 1) * coeff * (x ** (i - 2))
        return sum

    def get_degree(self) -> int:
        return len(self.coefficients) - 1

    @staticmethod
    def get_row_for_degree(x: float, degree: int) -> List[float]:
        return [x ** i for i in range(degree, -1, -1)]

    @staticmethod
    def get_slope_row_for_degree(x: float, degree: int) -> List[float]:
        return [i * x ** (i - 1) for i in range(degree, 0, -1)] + [0]

    @staticmethod
    def get_curvature_row_for_degree(x: float, degree: int) -> List[float]:
        return [i * (i - 1) * x ** (i - 2) for i in range(degree, 1, -1)] + [0, 0]

    def __str__(self):
        return f"P_{self.get_degree()} {self.coefficients}"

    def length(self, x0: float, x1: float) -> float:
        accum = 0
        dx = 0.001
        x = float(x0)
        while x < x1:
            slope = float(self.slope(x))
            accum += (1 + slope ** 2) ** (1 / 2) * dx
            x += dx
        return accum

    def get_local_quadratic_approximation(self, x: float):
        return approximate_curve(x, self.compute(x), x, self.slope(x), self.second_deriv(x))


def polynomial_from_parameters(parameters: np.ndarray) -> Polynomial:
    parameters = list(parameters.flatten().tolist())[0]  # ew ew ew
    return Polynomial(parameters)