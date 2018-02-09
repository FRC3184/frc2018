from wpilib.command import Command


class Logger:
    def __init__(self, filename):
        self.filename = filename
        self._lock = False
        self.watchers = []
        self._file = None

    def add(self, name, callback):
        if not self._lock:
            self.watchers += [(name, callback)]

    def start(self):
        if not self._lock:
            self._lock = True
            self._file = open(self.filename, "w")
            for name, _ in self.watchers:
                print(name, end=", ", file=self._file)
            print(file=self._file)
            self.get_update_command().start()

    def update(self):
        if self._lock:
            for _, watcher in self.watchers:
                print(watcher(), end=", ", file=self._file)
            print(file=self._file)
            self._file.flush()

    def end(self):
        self._file.close()

    def get_update_command(self):
        class LoggerUpdateCommand(Command):
            def __init__(self, logger: Logger):
                super().__init__(f"LoggerUpdateCommand {logger.filename}")
                self.setRunWhenDisabled(True)
                self.logger = logger

            def execute(self):
                self.logger.update()

            def isFinished(self):
                return False
        return LoggerUpdateCommand(self)
