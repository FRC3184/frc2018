from math import copysign
from typing import List, Sequence, Iterator

import time
from ctre import ControlMode
from ctre.talonsrx import TalonSRX

from ctre import TrajectoryPoint as TalonPoint

import mathutils
from control import robot_time
from mathutils import sgn


class TrajectoryPoint:
    def __init__(self, position, velocity, acc, time):
        self.position = position
        self.velocity = velocity
        self.acc = acc
        self.time = time


class MotionProfile:
    """
    Represents a trapezoidal motion profile
    """
    def __init__(self, start, end, cruise_speed, acc, frequency=100):
        # https://www.desmos.com/calculator/ponjr7cwze

        # If we're going in reverse we need to flip the sign of speed and acc
        # Although in theory these are vectors, it makes the API easier if they're given as magnitude
        dist = end - start
        cruise_speed = copysign(cruise_speed, dist)
        acc = copysign(acc, dist)

        ramp_time = cruise_speed / acc
        ramp_dist = acc * ramp_time ** 2 / 2
        cruise_dist = dist - 2 * ramp_dist
        cruise_time = cruise_dist / cruise_speed

        if sgn(cruise_dist) != sgn(dist):
            # All ramp, no cruise. Fix parameters to match
            cruise_time = 0
            cruise_dist = 0
            ramp_time = (dist / acc) ** 0.5
            ramp_dist = dist / 2

        time = cruise_time + 2 * ramp_time

        def get_pos(t):
            if t <= ramp_time:
                return start + acc * t ** 2 / 2
            elif t <= ramp_time + cruise_time:
                return start + ramp_dist + (t - ramp_time) * get_vel(ramp_time)
            else:
                tp = (t - ramp_time - cruise_time)
                return start + ramp_dist + cruise_dist + get_vel(ramp_time + cruise_time) * tp - acc * tp ** 2 / 2

        def get_vel(t):
            if t <= ramp_time:
                return get_acc(t) * t
            elif t <= ramp_time + cruise_time:
                return cruise_speed
            else:
                tp = (t - ramp_time - cruise_time)
                return get_vel(ramp_time + cruise_time) - acc * tp

        def get_acc(t):
            if t <= ramp_time:
                return acc
            elif t <= ramp_time + cruise_time:
                return 0
            else:
                return -acc

        self._points = []
        for i in range(int(time * frequency) + 1):
            t = i / frequency
            self._points.append(TrajectoryPoint(position=get_pos(t),
                                                velocity=get_vel(t),
                                                acc=get_acc(t),
                                                time=t))

    def __iter__(self) -> Iterator[TrajectoryPoint]:
        return self._points.__iter__()


class MotionProfileThread:
    def __init__(self, target: TalonSRX, points: List[TrajectoryPoint]):
        self.timeoutMs = 0
        self.target = target
        self.points = points
        self.margin = 0
        self.slotIdx = 0

    def start(self):
        self.time_begin = time.time().now()

    def calc_feedforward(self, current_point):
        return current_point.velocity

    def run(self):
        self.current_point = TrajectoryPoint(0,0)
        time_point = self.current_point.time

        if self.target.getClosedLoopError(0) < self.margin:
            self.time_begin = time.time().now()

        if time.time().now() - self.time_begin > time_point:
            self.current_point = self.points.pop()
            self.target.config_kF(self.slotIdx, self.calc_feedforward(self.current_point), self.timeoutMs)
            self.target.set(ControlMode.Position, self.current_point.position)

        robot_time.sleep(millis=5)