from wpilib.command import Command


class CommandNode:
    def __init__(self, cmd: Command):
        self.cmd = cmd
        self.children = []
        self.parents = []


class StructureExecutor(Command):
    def __init__(self, head: CommandNode):
        super().__init__()
        self.head = head
        self.current = [head]
        self.completed = []
        self.queued = []

    def initialize(self):
        pass

    def execute(self):
        for node in self.current:
            if node.cmd.isCompleted():
                self.completed.append(node)
                for child in node.children:
                    if child not in self.queued:
                        self.queued.append(child)

        for completed_node in self.completed:
            if completed_node in self.current:
                self.current.remove(completed_node)

        for queued_node in self.queued:
            for parent in queued_node.parents:
                if parent in self.completed:
                    break
            else:
                self.current.append(queued_node)
                queued_node.start()

    def isFinished(self):
        return len(self.current) == 0