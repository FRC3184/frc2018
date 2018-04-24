from ctre import FeedbackDevice
from ctre.talonsrx import TalonSRX
from wpilib.command import Subsystem

import Logger
from control.BlazeTalon import BlazeTalon
from control.smartdrive import SmartRobotDrive


class Drivetrain(Subsystem):
    def __init__(self):
        super().__init__("Drivetrain")

        self.talon_left_rear = BlazeTalon(0)
        self.talon_left_front = BlazeTalon(1)

        self.setup_talons(self.talon_left_rear, self.talon_left_front)

        self.talon_right_rear = BlazeTalon(2)
        self.talon_right_front = BlazeTalon(3)

        self.setup_talons(self.talon_right_rear, self.talon_right_front, invert=True)

        self.robotdrive = SmartRobotDrive(self.talon_left_rear, self.talon_right_rear)

        self.dt_logger = Logger.Logger("Drivetrain")
        self.dt_logger.add("Left Speed", self.robotdrive.get_left_speed)
        self.dt_logger.add("Right Speed", self.robotdrive.get_right_speed)
        self.dt_logger.add("Right Voltage", self.robotdrive.get_right_voltage)
        self.dt_logger.add("Left Voltage", self.robotdrive.get_left_voltage)
        self.dt_logger.start()

    def setup_talons(self, master: TalonSRX, slave: TalonSRX, invert=False,
                     pidIdx=0, timeoutMs=0, brake=True):
        slave.follow(master)
        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, pidIdx, timeoutMs)

        master.enableVoltageCompensation(True)
        master.configOpenLoopRamp(1/3, timeoutMs)
        master.configContinuousCurrentLimit(50, timeoutMs)
        master.configPeakCurrentLimit(80, timeoutMs)
        master.configPeakCurrentDuration(500, timeoutMs)

        neut_mode = TalonSRX.NeutralMode.Brake if brake else TalonSRX.NeutralMode.Coast
        master.setNeutralMode(neut_mode)
        slave.setNeutralMode(neut_mode)

        master.setSensorPhase(False)
        master.setInverted(invert)
        slave.setInverted(invert)

    def set_brake(self, brake=True):
        mode = TalonSRX.NeutralMode.Brake if brake else TalonSRX.NeutralMode.Coast
        self.talon_left_front.setNeutralMode(mode)
        self.talon_left_rear.setNeutralMode(mode)
        self.talon_right_front.setNeutralMode(mode)
        self.talon_right_rear.setNeutralMode(mode)

    def set_ramp(self, ramp=0):
        self.talon_left_rear.configOpenLoopRamp(ramp, 0)
        self.talon_right_rear.configOpenLoopRamp(ramp, 0)

    def arcade_drive(self, drive_power, turn_power):
        self.robotdrive.arcade_drive(drive_power, turn_power)

    def tank_drive(self, left_power, right_power):
        self.robotdrive.tank_drive(left_power, right_power)

    def curvature_drive(self, drive_power, turn_command):
        self.robotdrive.radius_drive(drive_power, turn_command, 1)

    def arc(self, speed, radius):
        self.robotdrive.radius_turn(speed, radius)
