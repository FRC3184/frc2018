import os
from time import strftime, gmtime

import hal
from wpilib.command import Command

from control import robot_time


def get_logger_timestamp():
    return strftime("%H.%M.%S..%Y.%m.%d", gmtime())


if hal.isSimulation():
    BASEDIR = "./logs"
else:
    BASEDIR = "/home/lvuser/logs"
if not os.path.exists(BASEDIR):
    os.makedirs(BASEDIR)


class Logger:
    def __init__(self, name):
        self.filename = f"{BASEDIR}/{name}.{get_logger_timestamp()}.csv"
        self._lock = False
        self.watchers = [("Time", robot_time.millis)]
        self._file = None
        self.update_command = LoggerUpdateCommand(self)

    def add(self, name, callback):
        if not callable(callback):
            callback = lambda: callback
        if not self._lock:
            self.watchers += [(name, callback)]

    def start(self):
        if not self._lock:
            self._lock = True
            self._file = open(self.filename, "w")
            for name, _ in self.watchers:
                print(name, end=", ", file=self._file)
            print(file=self._file)
            self.update_command.start()

    def update(self):
        if self._lock:
            for _, watcher in self.watchers:
                print(watcher(), end=", ", file=self._file)
            print(file=self._file)
            self._file.flush()

    def end(self):
        self._file.close()
        self.update_command.cancel()
        self._lock = False


class LoggerUpdateCommand(Command):
    def __init__(self, logger: Logger):
        super().__init__(f"LoggerUpdateCommand {logger.filename}")
        self.setRunWhenDisabled(True)
        self.logger = logger

    def execute(self):
        self.logger.update()

    def isFinished(self):
        return False