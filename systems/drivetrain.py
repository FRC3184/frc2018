from ctre import FeedbackDevice
from ctre.talonsrx import TalonSRX
from wpilib.command import Subsystem

from control.smartdrive import SmartRobotDrive


class Drivetrain(Subsystem):
    def __init__(self):
        super().__init__("Drivetrain")

        self.talon_left_rear = TalonSRX(0)
        self.talon_left_front = TalonSRX(1)

        self.setup_talons(self.talon_left_rear, self.talon_left_front)

        self.talon_right_rear = TalonSRX(2)
        self.talon_right_front = TalonSRX(3)

        self.setup_talons(self.talon_right_rear, self.talon_right_front)

        self.robotdrive = SmartRobotDrive(self.talon_left_rear, self.talon_right_rear)

    def setup_talons(self, master: TalonSRX, slave: TalonSRX, invert=False, pidIdx=0, timeoutMs=0):
        slave.follow(master)
        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, pidIdx, timeoutMs)
        master.setSensorPhase(True)

        master.configOpenLoopRamp(1/3, timeoutMs)
        master.configContinuousCurrentLimit(50, timeoutMs)
        master.configPeakCurrentLimit(80, timeoutMs)
        master.configPeakCurrentDuration(500, timeoutMs)

        master.setInverted(invert)
        slave.setInverted(invert)

    def arcade_drive(self, drive_power, turn_power):
        self.robotdrive.arcade_drive(drive_power, turn_power)

    def tank_drive(self, left_power, right_power):
        self.robotdrive.tank_drive(left_power, right_power)

    def curvature_drive(self, drive_power, turn_command):
        pass
