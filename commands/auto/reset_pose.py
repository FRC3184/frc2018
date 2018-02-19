from wpilib.command import Command

from control import pose
from control.pose import Pose


class ResetPoseCommand(Command):
    def execute(self):
        pose.set_new_pose(Pose(0, 0, 0))
        self.isFinished = lambda: True

    def isFinished(self):
        return False