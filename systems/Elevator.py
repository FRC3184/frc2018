from ctre import TrajectoryPoint as TalonPoint
from ctre.talonsrx import TalonSRX
from wpilib import DriverStation
from wpilib.command import Subsystem

from control.MotionProfile import MotionProfileThread, MotionProfile

TOP_EXTENT = 70
CARRIAGE_TRAVEL = 40

MAIN_IDX = 0
EXTENT_IDX = 1

CRUISE_SPEED = 0
ACC = 0

class Elevator(Subsystem):
    def __init__(self):
        super().__init__("Elevator")

        self.talon_master = TalonSRX(4)
        self.talon_slave = TalonSRX(5)

        self.talon_slave.follow(self.talon_master)

        self.talon_master.config_kF(MAIN_IDX, 1023/12, timeoutMs=0)
        self.talon_master.config_kF(EXTENT_IDX, 1023/12, timeoutMs=0)
        # so, to do custom feedforward in MP mode, you have to set the feedforward gain to 1023/12, and then pass in your desired feedforward voltage as TrajectoryPoint.velocity

    def get_back_torque(self, pos):
        if pos <= CARRIAGE_TRAVEL:
            return 0  # Weight of carriage * Radius of spool
        elif pos <= TOP_EXTENT:
            return 0  # (Weight of carriage + Weight of first stage) * Radius of spool

    def get_pid_index(self, pos):
        if pos <= CARRIAGE_TRAVEL:
            return MAIN_IDX
        elif pos <= TOP_EXTENT:
            return EXTENT_IDX

    def calc_ff(self, pos, vel, acc):
        hold_voltage = 12 * self.get_back_torque(pos) / self.get_stall_torque()

        return hold_voltage

    def gen_profile(self, start, end):
        talon_points = []
        rawmp = MotionProfile(start=start, end=end, cruise_speed=CRUISE_SPEED, acc=ACC)
        for point in rawmp:
            talonpt = TalonPoint(position=point.position,
                                 velocity=self.calc_ff(point.position, point.velocity, point.acc),
                                 headingDeg=0,
                                 profileSlotSelect0=self.get_pid_index(point.position),
                                 profileSlotSelect1=0,
                                 isLastPoint=False,
                                 zeroPos=False,
                                 timeDur=0)
            talon_points.append(talonpt)
        talon_points[-1].isLastPoint = True


    def get_elevator_position(self):
        """

        :return: The elevator's position as measured by the encoder, in inches. 0 is bottom, 70 is top
        """
        raise NotImplementedError

    def get_stall_torque(self):
        return 0.71 * 2 * 20

