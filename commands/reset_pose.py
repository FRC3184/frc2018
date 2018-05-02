from wpilib.command import Command

from control import pose_estimator
from control.pose_estimator import Pose


class ResetPoseCommand(Command):
    def execute(self):
        pose_estimator.set_new_pose(Pose(0, 0, 0))
        self.isFinished = lambda: True

    def isFinished(self):
        return False