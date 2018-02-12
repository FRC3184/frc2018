import socket
import sys

from wpilib import DriverStation

WEEKZERO = False
WZ_GAMEDATA_HOST = '10.0.100.44'
WZ_GAMEDATA_PORT = 5555
_cached_string = None


class Side:
    LEFT = 0
    RIGHT = 1

    @staticmethod
    def from_char(char):
        return Side.LEFT if char == "L" else Side.RIGHT


def getGameSpecificMessage_WeekZero():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (WZ_GAMEDATA_HOST, WZ_GAMEDATA_PORT)
    sock.connect(server_address)
    gamedata_msg = sock.recv(16)
    sock.close()
    return gamedata_msg


def get_message():
    global _cached_string
    if _cached_string is None:
        if WEEKZERO:
            _cached_string = getGameSpecificMessage_WeekZero()
        else:
            _cached_string = DriverStation.getInstance().getGameSpecificMessage()
    if _cached_string == "":
        _cached_string = "LLL"
        DriverStation.getInstance().reportWarning("Got empty game data string", False)
    return _cached_string


def get_own_switch_side():
    return Side.from_char(get_message()[0])


def get_scale_side():
    return Side.from_char(get_message()[1])


def get_opp_switch_side():
    return Side.from_char(get_message()[2])