import math
from typing import List, Tuple

from ctre import TrajectoryPoint as TalonPoint
from ctre.talonsrx import TalonSRX
from wpilib import DriverStation
from wpilib.command import Subsystem

from control.MotionProfile import MotionProfileThread, MotionProfile

TOP_EXTENT = 70
CARRIAGE_TRAVEL = 40

MAIN_IDX = 0
EXTENT_IDX = 1

CRUISE_SPEED = 80
ACC = 1.5*CRUISE_SPEED

GEAR_RATIO = 9
SPOOL_RADIUS = 0.5
CARRIAGE_WEIGHT = 20
EXTENT_WEIGHT = 10


class Elevator(Subsystem):
    def __init__(self):
        super().__init__("Elevator")

        self.talon_master = TalonSRX(4)
        self.talon_slave = TalonSRX(5)

        self.talon_slave.follow(self.talon_master)

        self.talon_master.config_kF(MAIN_IDX, 1023/12, timeoutMs=0)
        self.talon_master.config_kF(EXTENT_IDX, 1023/12, timeoutMs=0)
        # so, to do custom feedforward in MP mode, you have to set the feedforward gain to 1023/12, and then pass in your desired feedforward voltage as TrajectoryPoint.velocity

    def get_mass(self, pos):
        """
        Mass in lbm
        :param pos:
        :return:
        """
        if pos <= CARRIAGE_TRAVEL:
            return CARRIAGE_WEIGHT
        elif pos <= TOP_EXTENT:
            return CARRIAGE_WEIGHT + EXTENT_WEIGHT

    def get_hold_torque(self, pos):
        """
        Torque in lb-in
        :param pos:
        :return:
        """
        return self.get_mass(pos) * SPOOL_RADIUS

    def get_pid_index(self, pos):
        if pos <= CARRIAGE_TRAVEL:
            return MAIN_IDX
        elif pos <= TOP_EXTENT:
            return EXTENT_IDX

    def calc_ff(self, pos, vel, acc):
        """
        Returns feedforward in volts
        :param pos: Position of the elevator, in inches
        :param vel: Desired velocity, in in/s
        :param acc: Desired acceleration, in in/s^2
        :return: Feedforward voltage
        """
        hold_voltage = 12 * self.get_hold_torque(pos) / self.get_stall_torque()
        vel_maxv = 12 - hold_voltage
        vel_voltage = vel_maxv * vel / (self.get_max_speed() * (vel_maxv / 12))
        acc_maxv = 12 - hold_voltage - vel_voltage
        acc_voltage = 0

        return hold_voltage + vel_voltage + acc_voltage

    def gen_profile(self, start, end) -> Tuple[List[TalonPoint], int]:
        talon_points = []
        freq = 100
        rawmp = MotionProfile(start=start, end=end, cruise_speed=CRUISE_SPEED, acc=ACC, frequency=freq)
        for point in rawmp:
            talonpt = TalonPoint(position=self.in_to_native_units(point.position),
                                 velocity=self.calc_ff(point.position, point.velocity, point.acc),
                                 headingDeg=0,
                                 profileSlotSelect0=self.get_pid_index(point.position),
                                 profileSlotSelect1=0,
                                 isLastPoint=False,
                                 zeroPos=False,
                                 timeDur=0)
            talon_points.append(talonpt)
        # talon_points[-1].isLastPoint = True TODO can't set attribute

        return talon_points, freq

    def get_elevator_position(self):
        """

        :return: The elevator's position as measured by the encoder, in inches. 0 is bottom, 70 is top
        """
        raise NotImplementedError

    def get_stall_torque(self):
        """
        Motor stall torque (N-m) * # of motors * gear ratio * 8.851 lb-in/N-m
        :return: 12V stall torque in lb-in
        """
        return 0.71 * 2 * GEAR_RATIO * 8.851

    def get_max_speed(self):
        """
        Motor free speed (rpm) * (1/60) seconds/minute / gear ratio
        2*pi*r * RPS = IPS
        :return: 12V speed of the elevator in in/s
        """
        return 2*math.pi*SPOOL_RADIUS*(18730 * (1/60) / GEAR_RATIO)

    def in_to_native_units(self, inches):
        return (inches / (2*math.pi*SPOOL_RADIUS)) * 4096

    def native_to_inches(self, native_distance):
        return (native_distance / 4096) * (2*math.pi*SPOOL_RADIUS)

