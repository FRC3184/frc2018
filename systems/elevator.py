import math
from typing import List, Tuple

import ctre
from ctre import TrajectoryPoint as TalonPoint, FeedbackDevice, ControlMode
from ctre._impl.autogen.ctre_sim_enums import SetValueMotionProfile
from ctre.talonsrx import TalonSRX
from wpilib import DriverStation, DigitalInput, threading
from wpilib.command import Subsystem

from Logger import Logger
from control import robot_time
from control.MotionProfile import MotionProfile, SRXMotionProfileManager
from dashboard import dashboard2

TOP_EXTENT = 62
CARRIAGE_TRAVEL = 28.5

# Two different PID indices for gain scheduling
# MAIN is between 0 and CARRIAGE_TRAVEL
# EXTENT is between CARRIAGE TRAVEL and TOP EXTENT
MAIN_IDX = 0
EXTENT_IDX = 1

ZERO_POS = 4100
ZERO_MAX_ERR = 150

CRUISE_SPEED = 80
ACC = 1.5*CRUISE_SPEED

GEAR_RATIO = 9
SPOOL_RADIUS = 1
CARRIAGE_WEIGHT = 5
EXTENT_WEIGHT = 8

FREQUENCY = 100


class ElevatorState:
    HOLDING = 0
    MOVING = 1
    MANUAL = 2
    ZEROING = 3


class Elevator(Subsystem):
    def __init__(self, mock=False):
        super().__init__("Elevator")

        self.talon_master = TalonSRX(4)
        self.talon_slave = TalonSRX(5)

        self._state = ElevatorState.HOLDING

        self.nlogger = Logger("elevator")
        self.nlogger.add("Time", robot_time.delta_time)
        self.nlogger.add("Position", self.get_elevator_position)
        self.nlogger.add("Voltage", self.talon_master.getMotorOutputVoltage)
        self.nlogger.add("Current", self.talon_master.getOutputCurrent)
        # self.nlogger.start()

        dashboard2.add_graph("Elevator Position", self.get_elevator_position)
        dashboard2.add_graph("Elevator Voltage", self.talon_master.getMotorOutputVoltage)

        if not mock:
            self.mp_manager = SRXMotionProfileManager(self.talon_master, 1000 // FREQUENCY)

            self.talon_master.setQuadraturePosition(0, 0)

            self.talon_slave.follow(self.talon_master)
            # 1023 units per 12V
            # This lets us pass in feedforward as voltage
            self.talon_master.config_kF(MAIN_IDX, 1023/12, 0)
            self.talon_master.config_kF(EXTENT_IDX, 1023/12, 0)

            self.talon_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
            self.talon_master.setSensorPhase(True)
            self.talon_master.setInverted(True)
            self.talon_slave.setInverted(True)

    def init_profile(self, new_pos):
        if self._state != ElevatorState.HOLDING:
            return False
        profile, _ = self.gen_profile(self.get_elevator_position(), new_pos)
        self.mp_manager.init_profile(profile)
        self.talon_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        return True

    def start_profile(self):
        self._state = ElevatorState.MOVING
        return self.mp_manager.start_profile()

    def has_finished_profile(self):
        return self.mp_manager.is_done()

    def finish_profile(self):
        self._state = ElevatorState.HOLDING

    def set_power(self, power):
        self._state = ElevatorState.MANUAL
        self.talon_master.set(ControlMode.PercentOutput, power)

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
        rawmp = MotionProfile(start=start, end=end, cruise_speed=CRUISE_SPEED, acc=ACC, frequency=FREQUENCY)
        for point in rawmp:
            last = point == rawmp[-1]
            talonpt = TalonPoint(position=self.in_to_native_units(point.position),
                                 velocity=self.calc_ff(point.position, point.velocity, point.acc),
                                 headingDeg=0,
                                 profileSlotSelect0=self.get_pid_index(point.position),
                                 profileSlotSelect1=0,
                                 isLastPoint=last,
                                 zeroPos=False,
                                 timeDur=0)
            talon_points.append(talonpt)

        return talon_points, FREQUENCY

    def get_elevator_position(self):
        """

        :return: The elevator's position as measured by the encoder, in inches. 0 is bottom, 70 is top
        """
        return self.native_to_inches(self.talon_master.getQuadraturePosition())

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

    def start_zero_position(self):
        self.talon_master.selectProfileSlot(MAIN_IDX, 0)
        self.talon_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0)
        self.talon_master.set(ControlMode.Position, ZERO_POS)
        self._state = ElevatorState.ZEROING

    def is_done_zeroing(self):
        return self._state == ElevatorState.ZEROING and self.talon_master.getClosedLoopError(0) < ZERO_MAX_ERR

    def finish_zero_position(self):
        self._state = ElevatorState.HOLDING
        self.talon_master.setQuadraturePosition(0, 0)

    def hold(self):
        pos = self.get_elevator_position()
        self.talon_master.selectProfileSlot(self.get_pid_index(pos), 0)
        self.talon_master.set(ControlMode.Position, self.in_to_native_units(pos))
        self._state = ElevatorState.HOLDING

    def is_at_top(self):
        return self.talon_master.isFwdLimitSwitchClosed()

    def is_at_bottom(self):
        return self.talon_master.isRevLimitSwitchClosed()
