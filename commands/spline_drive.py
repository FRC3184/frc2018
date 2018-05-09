import hashlib
import math
import os
import pickle
from typing import List

import wpilib
from py_pursuit_pathing import mathlib
from py_pursuit_pathing.pose import Pose
from wpilib.command import Command

import mathutils
from dashboard import dashboard2
from systems.drivetrain import Drivetrain

import pathfinder as pf
from pathfinder import Segment

from util import get_basedir


def convert_waypoints(path: List[Pose]):
    for wp in path:
        yield pf.Waypoint(wp.x, wp.y, wp.heading)


class SplineDriveCommand(Command):
    def __init__(self, drivetrain: Drivetrain, path: List[Pose], cruise:float=10, acc:float=20, jerk:float=40):
        super().__init__()
        self.requires(drivetrain)
        self.drivetrain = drivetrain

        # Generate PF trajectory

        self.trajectory = self._gen_trajectory(path, cruise, acc, jerk)
        modifier = pf.modifiers.TankModifier(self.trajectory).modify(drivetrain.robotdrive.robot_width)
        self.left_follower = BlazeEncoderFollower(modifier.getLeftTrajectory(),
                                                  *drivetrain.robotdrive.get_left_fwd_ff(),
                                                  kP=2, kD=0)
        self.right_follower = BlazeEncoderFollower(modifier.getRightTrajectory(),
                                                  *drivetrain.robotdrive.get_right_fwd_ff(),
                                                  kP=2, kD=0)

    def initialize(self):
        self.left_follower.set_base_distance(self.drivetrain.robotdrive.get_left_distance())
        self.right_follower.set_base_distance(self.drivetrain.robotdrive.get_right_distance())
        self.left_follower.reset()
        self.right_follower.reset()
        if wpilib.hal.isSimulation():
            import pyfrc.sim
            render = pyfrc.sim.get_user_renderer()
            render.draw_pathfinder_trajectory(self.trajectory, show_dt=True)
        dashboard2.add_graph("Left Err", lambda: self.left_follower.last_err)
        dashboard2.add_graph("Right Err", lambda: self.right_follower.last_err)

    def execute(self):
        l_v = self.left_follower.calculate(-self.drivetrain.robotdrive.get_left_distance())
        r_v = self.right_follower.calculate(self.drivetrain.robotdrive.get_right_distance())
        k_turn = 0 # 0.5 / math.pi

        turn_v = k_turn * (self.left_follower.heading -
                           mathutils.normalize_angle(self.drivetrain.robotdrive.get_heading_rads()))

        self.drivetrain.robotdrive.tank_drive((l_v + turn_v)/12, (r_v - turn_v)/12, deadband=0)

    def _gen_trajectory(self, path: List[Pose], cruise: float, acc: float, jerk: float):
        fname = f"{self._get_path_hash(path, cruise, acc, jerk)}.pickle"
        pickle_file = os.path.join(get_basedir(), 'paths', fname)

        if wpilib.RobotBase.isSimulation():
            # generate the trajectory here
            info, trajectory = pf.generate(list(convert_waypoints(path)), pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
                                           dt=0.05,  # 50ms
                                           max_velocity=cruise,
                                           max_acceleration=acc,
                                           max_jerk=jerk)
            # and then write it out
            with open(pickle_file, 'wb') as fp:
                pickle.dump(trajectory, fp)
        else:
            with open('fname', 'rb') as fp:
                trajectory = pickle.load(fp)

        return trajectory

    def _get_path_hash(self, points, cruise, acc, jerk):
        return hashlib.md5(f"{str(points)}, {cruise}, {acc}, {jerk}".encode("U8")).hexdigest()

    def isFinished(self):
        return self.left_follower.is_finished()


class BlazeEncoderFollower:
    def __init__(self, trajectory: List[Segment],
                 k_int: float,
                 kV: float,
                 kA: float,
                 kP: float,
                 kD: float):
        self.trajectory = trajectory
        self.offset = 0
        self.kP = kP
        self.kD = kD
        self.kV = kV
        self.kA = kA
        self.k_int = k_int
        self.segment = 0
        self.last_err = 0
        self.heading = 0

    def set_base_distance(self, offset: int):
        self.offset = offset

    def reset(self):
        self.segment = 0
        self.last_err = 0

    def calculate(self, wheel_position: int):
        distance_covered = (wheel_position - self.offset)
        if self.segment < len(self.trajectory):
            cur_segment = self.trajectory[self.segment]
            error = cur_segment.position - distance_covered
            value = self.kP * error + self.kD * (error - self.last_err) / cur_segment.dt + \
                self.kV * cur_segment.velocity + self.kA * cur_segment.acceleration + self.k_int
            self.heading = cur_segment.heading
            self.last_err = error
            self.segment += 1
            return value
        return 0

    def is_finished(self):
        return self.segment >= len(self.trajectory)
