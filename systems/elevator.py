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

TOP_EXTENT = 68.5
CARRIAGE_TRAVEL = 32

TRAVEL_RATIO = TOP_EXTENT / 47.6

# Two different PID indices for gain scheduling
# MAIN is between 0 and CARRIAGE_TRAVEL
# EXTENT is between CARRIAGE TRAVEL and TOP EXTENT
MAIN_IDX = 0
EXTENT_IDX = 1
HOLD_MAIN_IDX = 2
HOLD_EXTENT_IDX = 3

ZERO_POS = 4100
ZERO_MAX_ERR = 150

CRUISE_SPEED = 40
ACC = 2*CRUISE_SPEED

GEAR_RATIO = 20
SPOOL_RADIUS = 0.5
CARRIAGE_WEIGHT = 20 + 10/16
EXTENT_WEIGHT = 10

FREQUENCY = 100


class ElevatorPositions:
    BOTTOM = 0
    SWITCH = 20
    TOP = TOP_EXTENT


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
        dashboard2.add_graph("Elevator Current", self.talon_master.getOutputCurrent)
        dashboard2.add_graph("Elevator Current2", self.talon_slave.getOutputCurrent)
        dashboard2.add_graph("Elevator State", lambda: self._state)

        if not mock:
            self.mp_manager = SRXMotionProfileManager(self.talon_master, 1000 // FREQUENCY)

            self.talon_master.setQuadraturePosition(0, 0)

            self.talon_slave.follow(self.talon_master)
            # 1023 units per 12V
            # This lets us pass in feedforward as voltage
            self.talon_master.config_kF(MAIN_IDX, 1023/12, 0)
            self.talon_master.config_kF(EXTENT_IDX, 1023/12, 0)

            self.talon_master.config_kP(HOLD_MAIN_IDX, .1 * 1023 / 4096, 0)
            self.talon_master.config_kP(HOLD_EXTENT_IDX, .3 * 1023 / 4096, 0)

            self.talon_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
            self.talon_master.setSensorPhase(True)
            invert = False
            self.talon_master.setInverted(invert)
            self.talon_slave.setInverted(invert)

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
        self.hold()

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
        if not 0 <= end <= TOP_EXTENT:
            raise ValueError(f"End must be within 0 and {TOP_EXTENT}")
        talon_points = []
        rawmp = MotionProfile(start=start, end=end, cruise_speed=CRUISE_SPEED, acc=ACC, frequency=FREQUENCY)
        for point in rawmp:
            last = point == rawmp[-1]
            talonpt = TalonPoint(position=-self.in_to_native_units(point.position),
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
        return -self.native_to_inches(self.talon_master.getQuadraturePosition())

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
        return (inches / (2*math.pi*SPOOL_RADIUS)) * 4096 / TRAVEL_RATIO

    def native_to_inches(self, native_distance):
        return (native_distance / 4096) * (2*math.pi*SPOOL_RADIUS) * TRAVEL_RATIO

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
        target = self.in_to_native_units(pos)
        pid = self.get_pid_index(pos) + 2
        self.talon_master.selectProfileSlot(pid, 0)
        ff = (1023/12) * self.calc_ff(pos, 0, 0)
        if target == 0:
            ff = 0
        else:
            ff /= target
        self.talon_master.config_kF(pid, ff, 0)
        self.talon_master.set(ControlMode.Position, target)
        self._state = ElevatorState.HOLDING

    def is_at_top(self):
        return self.talon_master.isFwdLimitSwitchClosed()

    def is_at_bottom(self):
        return self.talon_master.isRevLimitSwitchClosed()
