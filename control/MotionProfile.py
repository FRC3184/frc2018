import threading
from math import copysign
from typing import List, Sequence, Iterator

import time

import hal
from ctre import ControlMode
from ctre._impl.autogen.ctre_sim_enums import SetValueMotionProfile
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
        """
        Generate a trapezoidal motion profile, starting at `start` and ending at `end`
        :param start:
        :param end:
        :param cruise_speed: Magnitude of cruise velocity
        :param acc: Magnitude of acceleration
        :param frequency: Number of points to generate per second
        """
        # https://www.desmos.com/calculator/ponjr7cwze

        # If we're going in reverse we need to flip the sign of speed and acc
        # Although in theory these are vectors, it makes the API easier if they're given as magnitude
        dist = end - start
        cruise_speed = copysign(cruise_speed, dist)
        acc = copysign(acc, dist)

        self.start = start
        self.end = end
        self.displacement = dist

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

    def __getitem__(self, item):
        return self._points.__getitem__(item)


class SRXMotionProfileManager:
    def __init__(self, talon: TalonSRX, frame_period: int, min_points=20):
        self.talon = talon
        if not hal.isSimulation():
            self.talon.configMotionProfileTrajectoryPeriod(frame_period, 0)
        self.min_points = min_points
        self.frame_period = frame_period

        self._process_mp_thread = threading.Thread(target=self._process_mp)

    def _process_mp(self):
        while True:
            status = self.get_status()
            if status.topBufferCnt > 0:
                self.talon.processMotionProfileBuffer()
            robot_time.sleep(millis=self.frame_period//2)

    def init_profile(self, points: List[TalonPoint], process=True):
        self.talon.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value)
        self.talon.clearMotionProfileTrajectories()

        for pt in points:
            self.talon.pushMotionProfileTrajectory(trajPt=pt)
        if not self._process_mp_thread.is_alive():
            self._process_mp_thread.start()

    def start_profile(self):
        if self.get_status().btmBufferCnt >= self.min_points:
            self.talon.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable.value)
            return True
        return False

    def is_done(self):
        return self.get_status().isLast

    def get_status(self):
        status = self.talon.getMotionProfileStatus()
        return status

