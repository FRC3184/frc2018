import math

import wpilib
from py_pursuit_pathing.pose import Pose

from mathutils import Vector2

_estimator_thread = None
_estimator = None


def init(left_encoder_callback, right_encoder_callback, gyro_callback=None,
                   current_pose=Pose(0, 0, 0), wheelbase=None, encoder_factor=1):
    global _estimator, _estimator_thread
    _estimator = PoseEstimator(left_encoder_callback, right_encoder_callback, gyro_callback,
                               current_pose, wheelbase, encoder_factor)
    if _estimator_thread is None:
        _estimator_thread = wpilib.Notifier(run=lambda: _estimator.update(dt=10/1000))
        _estimator_thread.startPeriodic(10/1000)


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

        self.gyro_offset = 0

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
            self.current_pose.heading = self.gyro_callback() - self.gyro_offset
        else:
            self.current_pose.heading += (dist_right - dist_left) / self.robot_width
        dist = (-dist_left + dist_right) / 2

        self.current_pose.x += dist * math.cos(self.current_pose.heading)
        self.current_pose.y += dist * math.sin(self.current_pose.heading)


def set_new_pose(poz: Pose):
    _estimator.gyro_offset = get_current_pose().heading + 0
    _estimator.current_pose = poz