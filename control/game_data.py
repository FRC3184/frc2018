import socket

from wpilib import DriverStation

from control import robot_time

WEEKZERO = False
WZ_GAMEDATA_HOST = '10.0.100.44'
WZ_GAMEDATA_PORT = 5555
_cached_string = None
_robot_side = None


class Side:
    LEFT = 0
    RIGHT = 1
    CENTER = 2

    @staticmethod
    def from_char(char):
        return Side.LEFT if char == "L" else (Side.RIGHT if char == "R" else Side.CENTER)


def getGameSpecificMessage_WeekZero():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (WZ_GAMEDATA_HOST, WZ_GAMEDATA_PORT)
    sock.connect(server_address)
    gamedata_msg = sock.recv(16)
    sock.close()
    return gamedata_msg


def get_message():
    global _cached_string
    if WEEKZERO:
        _cached_string = getGameSpecificMessage_WeekZero()
    else:
        _cached_string = DriverStation.getInstance().getGameSpecificMessage()
    return _cached_string


def init(placement):
    global _robot_side, _cached_string
    _robot_side = placement
    count = 1
    while len(get_message()) < 3:
        print(f"Getting game-specific message... Retry {count}")
        if count == 10:
            print("Canceling game-specific message, either you messed up or you need to talk to an FTA!")
            _cached_string = "RRR"
            break
        robot_time.sleep(seconds=1)


def get_own_switch_side():
    return Side.from_char(get_message()[0])


def get_scale_side():
    return Side.from_char(get_message()[1])


def get_opp_switch_side():
    return Side.from_char(get_message()[2])


def get_robot_side():
    return _robot_side