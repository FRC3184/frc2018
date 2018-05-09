import wpilib


def get_basedir():
    if wpilib.hal.isSimulation():
        basedir = ""
    else:
        basedir = "/home/lvuser/py"
    return basedir