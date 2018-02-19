from wpilib.command import Command


class WaitUntilConditionCommand(Command):
    def __init__(self, condition: callable):
        super().__init__()
        self.condition = condition

    def isFinished(self):
        return self.condition()