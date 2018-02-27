import time
from datetime import datetime

import wpilib

_auto_time = 15  # Seconds in auto period
_teleop_time = 2 * 60 + 15  # Seconds in teleop period
_endgame_time = 30


def sleep(millis=None, seconds=None):
    if millis is None and seconds is None:
        raise ValueError("Provide millis or seconds")
    if millis is not None:
        time.sleep(millis / 1000)
    if seconds is not None:
        time.sleep(seconds)


def get_match_time():
    return wpilib.DriverStation.getInstance().getMatchTime()


def millis():
    return datetime.now().timestamp() * 1000

_start_time = millis()


def delta_time():
    return millis() - _start_time