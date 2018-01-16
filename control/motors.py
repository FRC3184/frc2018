# This file should contain code to set up motors and their properties, along with motor groups
from control.MotorGroup import SmartMotorGroup

_instance = None


class Motors:
    def __init__(self):
        pass

def get() -> Motors:
    global _instance
    if _instance is None:
        _instance = Motors()
    return _instance

def init():
    get()