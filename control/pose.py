import math
import threading

import wpilib

import robot_time
from mathutils import Vector2

_estimator_thread = None
_estimator = None


class Pose(Vector2):
    """
    Robot pose in world coordinates
    """
    def __init__(self, x: float, y: float, heading: float):
        super().__init__(x, y)
        self.heading = heading

    def __repr__(self):
        return "Pose(x={}, y={}, heading={})".format(self.x, self.y, (self.heading * 180 / math.pi))

    def __add__(self, other):
        if type(other) == type(self):
            return Pose(self.x + other.x, self.y + other.y, self.heading + other.heading)
        elif type(other) == Vector2:
            return Vector2(self.x - other.x, self.y - other.y)

    def __sub__(self, other):
        if type(other) == Pose:
            return Pose(self.x - other.x, self.y - other.y, self.heading - other.heading)
        elif type(other) == Vector2:
            return Vector2(self.x - other.x, self.y - other.y)


def init(left_encoder_callback, right_encoder_callback, gyro_callback=None,
                   current_pose=Pose(0, 0, 0), wheelbase=None, encoder_factor=1):
    global _estimator, _estimator_thread
    _estimator = PoseEstimator(left_encoder_callback, right_encoder_callback, gyro_callback,
                               current_pose, wheelbase, encoder_factor)
    if _estimator_thread is None:
        _estimator_thread = threading.Thread(target=lambda: _update_estimator(_estimator))
        _estimator_thread.start()


def get_current_pose() -> Pose:
    if _estimator is not None:
        return _estimator.current_pose
    raise ValueError("Estimator has not been initialized")


class PoseEstimator:

    def __init__(self, left_encoder_callback, right_encoder_callback, gyro_callback=None,
                 current_pose=Pose(0, 0, 0), wheelbase=None, encoder_factor=1):
        self.left_encoder_callback = left_encoder_callback
        self.right_encoder_callback = right_encoder_callback
        self.gyro_callback = gyro_callback
        self.robot_width = wheelbase
        self.encoder_factor = encoder_factor

        self._left_last_enc = self.left_encoder_callback()
        self._right_last_enc = self.right_encoder_callback()

        self.current_pose = current_pose

    def update(self, dt):
        enc_left = self.left_encoder_callback()
        enc_right = self.right_encoder_callback()
        dist_left = (enc_left - self._left_last_enc) * self.encoder_factor
        dist_right = (enc_right - self._right_last_enc) * self.encoder_factor
        self._left_last_enc = enc_left
        self._right_last_enc = enc_right

        # Use gyro for heading if we can, otherwise difference between wheels
        # In future, combine approaches?
        if self.gyro_callback is not None:
            self.current_pose.heading = self.gyro_callback()
        else:
            self.current_pose.heading += (dist_right - dist_left) / self.robot_width
        dist = (dist_left + dist_right) / 2
        self.current_pose.x += dist * math.cos(self.current_pose.heading)
        self.current_pose.y += dist * math.sin(self.current_pose.heading)

        print(self.current_pose)


def _update_estimator(pose_estimator: PoseEstimator, sleep_sec=(10/1000)):
    ct = 0
    while True:
        pose_estimator.update(dt=sleep_sec)
        ct += sleep_sec
        if ct > 1:
            print(get_current_pose())
            ct = 0
        robot_time.sleep(seconds=sleep_sec)