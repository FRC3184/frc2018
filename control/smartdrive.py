import math
import threading
import warnings
from typing import Tuple

import ctre.talonsrx
import wpilib
from robotpy_ext.common_drivers.navx.ahrs import AHRS

import control.pose_estimator as pose
import mathutils
from control import robot_time
from dashboard import dashboard2


class SmartRobotDrive(wpilib.MotorSafety):
    class Mode:
        PercentVbus = ctre.ControlMode.PercentOutput
        Speed = ctre.ControlMode.Velocity
        MotionMagic = ctre.ControlMode.MotionMagic
        MotionProfile = ctre.ControlMode.MotionProfile

    def __init__(self, left_motor: ctre.talonsrx.TalonSRX, right_motor: ctre.talonsrx.TalonSRX, **kwargs):
        '''
        Represents a drivetrain that uses CANTalons and so manages those advanced features
        :param left_motor: 
        :param right_motor: 
        :param kwargs: 
        '''
        wpilib.MotorSafety.__init__(self)
        self.robot_width = kwargs.pop("robot_width", 24 / 12)
        self.max_turn_radius = kwargs.pop("max_radius", 10)
        self.wheel_diameter = kwargs.pop("wheel_diameter", 6)
        self.max_speed = kwargs.pop("max_speed", 16)

        self.ahrs = AHRS.create_spi()
        self.ahrs.reset()
        self._left_motor = left_motor
        self._right_motor = right_motor

        pose.init(left_encoder_callback=self.get_left_distance,
                  right_encoder_callback=self.get_right_distance,
                  gyro_callback=self.get_heading_rads,
                  wheelbase=self.robot_width)
        dashboard2.add_graph("Pose X", lambda: pose.get_current_pose().x)
        dashboard2.add_graph("Pose Y", lambda: pose.get_current_pose().y)

        self._max_output = 1
        self._mode = SmartRobotDrive.Mode.PercentVbus
        self.set_mode(self._mode)
        
        # Motor safety
        self.setSafetyEnabled(True)

        pose.init(left_encoder_callback=self.get_left_distance, right_encoder_callback=self.get_right_distance,
                  gyro_callback=self.get_heading_rads,
                  wheelbase=self.robot_width,
                  encoder_factor=1)

        dashboard2.add_graph("Heading", lambda: pose.get_current_pose().heading * 180 / math.pi)

    def get_left_fwd_ff(self) -> Tuple[float, float, float]:
        """

        :return: k_int, kV, kA
        """
        return 1.010, 0.758, 0.299

    def get_right_fwd_ff(self) -> Tuple[float, float, float]:
        """

        :return: k_int, kV, kA
        """
        return 1.030, 0.742, 0.312

    def set_mode(self, mode):
        if self._mode != mode:
            self._mode = mode
            if self._mode == SmartRobotDrive.Mode.PercentVbus:
                self._max_output = 1
                self.setSafetyEnabled(True)
            elif self._mode == SmartRobotDrive.Mode.Speed:
                self._max_output = self.rpm_to_native_speed(self.get_fps_rpm_ratio())
                self.setSafetyEnabled(True)
            else:
                self.setSafetyEnabled(False)
                self._max_output = 0  # The idea of a max setpoint doesn't make sense for motion profiles

    def radius_ratio(self, radius):
        D = self.robot_width / 2
        return (radius - D) / (radius + D)

    def radius_turn(self, speed, radius):
        # (Vo + Vi) / 2 = pow
        # Vo*r = Vi
        # Vo(1+r)/2 = pow

        # 2*pow/(1+r) = Vo
        # 2*r*pow/(1+r) = Vi

        # With this calculation, it's possible to saturate the motors. If that happens, rescale to maintain the curve.
        turn_dir = mathutils.sgn(radius)
        radius = abs(radius)
        r = self.radius_ratio(radius)
        Vo = 2 * speed / (1 + r)
        Vi = r*Vo
        if Vo > self.max_speed:
            Vi /= Vo
            Vo /= Vo
            Vo *= self.max_speed
            Vi *= self.max_speed

        Vi /= self.max_speed
        Vo /= self.max_speed
        if turn_dir > 0:
            self._set_motor_outputs(Vo, Vi)
            # self.drive_profile_open_loop(Vo, Vi, 0, 0)
        else:
            self._set_motor_outputs(Vi, Vo)
            # self.drive_profile_open_loop(Vi, Vo, 0, 0)

    def radius_drive(self, forward_power, turn_power, deadband=0.05):
        forward_power *= self.max_speed
        if self.is_manual_control_mode():
            if abs(turn_power) < deadband:
                self.drive_profile_open_loop(forward_power, forward_power, 0, 0)
                return
            if abs(forward_power) < deadband:
                self._set_motor_outputs(0, 0)
                return
            turn_power = mathutils.signed_power(turn_power, 1/3)
            radius = self.robot_width / 2 + self.max_turn_radius * (1 - abs(turn_power))
            self.radius_turn(forward_power,
                             radius * mathutils.sgn(turn_power))
        else:
            warnings.warn("Not in a control mode for Radius Drive", RuntimeWarning)
            self._set_motor_outputs(0, 0)

    def motion_magic(self, distance: float, speed: float, acc: float, curvature: float = 0):
        """
        Set the talons to drive in an arc or straight at speed, accelerating at acc.
        Only needs to be called once.
        If curvature != 0, the outside wheel goes distance at speed and acc, and the inner wheel speed is decreased.
        :param distance: Distance in feet to travel
        :param speed: Speed (in feet/sec) to cruise at
        :param acc: Acceleration (in feet/sec^2) to accelerate/decelerate at
        :param curvature: 1/radius of turn. If 0, drive straight.
        :return: 
        """
        if curvature == 0:
            ratio = 1
            turn_dir = 1
        else:
            radius = 1 / curvature
            D = self.robot_width / 2
            turn_dir = mathutils.sgn(radius)
            radius = abs(radius)
            ratio = (radius - D) / (radius + D)

        # Change units to what the talons are expecting
        vel_rpm = self.fps_to_rpm(speed)
        vel_native = SmartRobotDrive.rpm_to_native_speed(vel_rpm)
        acc_rpm = self.fps_to_rpm(acc)  # Works because required unit is rpm/sec for no real good reason.
        acc_native = SmartRobotDrive.rpm_to_native_speed(acc_rpm)
        dist_revs = self.feet_to_revs(distance)
        dist_native = SmartRobotDrive.revs_to_native_distance(dist_revs)
        print(dist_revs)

        # Don't set encoder position to 0, because that would mess up pose estimation
        # Instead, set to current position, plus however far we want to go
        left_current_pos = self._left_motor.getQuadraturePosition()
        right_current_pos = self._right_motor.getQuadraturePosition()


        # Set the talon parameters
        # If turn > 0, left is outside
        if turn_dir > 0:
            left_ratio = 1
            right_ratio = ratio
        else:
            left_ratio = ratio
            right_ratio = 1
        timeout_ms = 0

        self._left_motor.configMotionCruiseVelocity(vel_native * left_ratio, timeout_ms)
        self._right_motor.configMotionCruiseVelocity(vel_native * right_ratio, timeout_ms)
        self._left_motor.configMotionAcceleration(acc_native * left_ratio, timeout_ms)
        self._right_motor.configMotionAcceleration(acc_native * right_ratio, timeout_ms)
        self._left_motor.set(SmartRobotDrive.Mode.MotionMagic,
                             left_current_pos + dist_native * left_ratio)
        self._right_motor.set(SmartRobotDrive.Mode.MotionMagic,
                              right_current_pos + dist_native * right_ratio)

    def tank_drive(self, left: float, right: float, deadband=0.05):
        if self.is_manual_control_mode():
            left = mathutils.deadband(left, deadband)
            right = mathutils.deadband(right, deadband)
            self._set_motor_outputs(left, right)
        else:
            warnings.warn("Not in a control mode for Tank Drive", RuntimeWarning)
            self._set_motor_outputs(0, 0)
    
    def arcade_drive(self, forward: float, turn: float, deadband=0.05):
        if self.is_manual_control_mode():
            forward = mathutils.deadband(forward, deadband)
            turn = mathutils.deadband(turn, deadband)

            if forward > 0.0:
                if turn > 0.0:
                    left = forward - turn
                    right = max(forward, turn)
                else:
                    left = max(forward, -turn)
                    right = forward + turn
            else:
                if turn > 0.0:
                    left = -max(-forward, turn)
                    right = forward + turn
                else:
                    left = forward - turn
                    right = -max(-forward, -turn)

            #self.drive_profile_open_loop(left * self.max_speed,
            #                             right * self.max_speed, 0, 0)
            self._set_motor_outputs(left, right)
        else:
            warnings.warn("Not in a control mode for Arcade Drive", RuntimeWarning)
            self._set_motor_outputs(0, 0)

    def get_heading(self):
        return -self.ahrs.getYaw()

    def get_heading_rads(self):
        return self.get_heading() * math.pi / 180

    def get_left_distance(self):
        return self.native_distance_to_feet(self._left_motor.getQuadraturePosition())

    def get_right_distance(self):
        return self.native_distance_to_feet(self._right_motor.getQuadraturePosition())

    def get_right_speed(self):
        return (1/60) * math.pi * self.wheel_diameter * \
               self.native_speed_to_rpm(self._right_motor.getQuadratureVelocity())

    def get_left_speed(self):
        return (1/60) * math.pi * self.wheel_diameter * \
                self.native_speed_to_rpm(self._left_motor.getQuadratureVelocity())

    def get_right_voltage(self):
        return self._right_motor.getMotorOutputVoltage()

    def get_left_voltage(self):
        return self._left_motor.getMotorOutputVoltage()

    def default(self):
        self._set_motor_outputs(0, 0)

    def get_fps_rpm_ratio(self):
        return 12 * 60 * self.max_speed / (math.pi * self.wheel_diameter)

    def fps_to_rpm(self, fps: float):
        return 60 * 12 * fps / (math.pi * self.wheel_diameter)

    def feet_to_revs(self, feet: float):
        return 12 * feet / (math.pi * self.wheel_diameter)

    def revs_to_feet(self, revs: float):
        try:
            return (math.pi * self.wheel_diameter) * revs / 12
        except ZeroDivisionError:
            return 0

    @staticmethod
    def rpm_to_native_speed(rpm: float) -> int:
        return int(rpm * 4096 / 600)

    @staticmethod
    def revs_to_native_distance(revs: float) -> int:
        return int(revs * 4096)

    @staticmethod
    def native_distance_to_revs(native_distance: int) -> float:
        return native_distance / 4096

    @staticmethod
    def native_speed_to_rpm(native_speed: int):
        return 600 * native_speed / 4096

    def native_distance_to_feet(self, native_distance: int) -> float:
        return (native_distance / 4096) * math.pi * self.wheel_diameter / 12

    def feet_to_native_distance(self, feet: float) -> int:
        return int(SmartRobotDrive.revs_to_native_distance(self.feet_to_revs(feet)))

    def drive_profile_open_loop(self, left_speed: float, right_speed: float, left_acc: float, right_acc: float):
        r_int, r_kV, r_kA = self.get_right_fwd_ff()
        l_int, l_kV, l_kA = self.get_left_fwd_ff()

        left = (math.copysign(l_int, left_speed) + l_kV * left_speed + l_kA * left_acc) / 12
        right = (math.copysign(r_int, right_speed) + r_kV * right_speed + r_kA * right_acc) / 12
        self._set_motor_outputs(left, right)

    def _set_motor_outputs(self, left: float, right: float):
        if self._mode == SmartRobotDrive.Mode.Speed:
            left = self.fps_to_rpm(left * self.max_speed)
            right = self.fps_to_rpm(right * self.max_speed)
        self._left_motor.set(self._mode, left)
        self._right_motor.set(self._mode, right)
        self.feed()

    def has_finished_motion_magic(self, margin=1/12):
        left_err = self.native_distance_to_feet(self._left_motor.getClosedLoopError(0))
        right_err = self.native_distance_to_feet(self._right_motor.getClosedLoopError(0))
        return abs(left_err + right_err) / 2 < margin

    def is_manual_control_mode(self):
        return self._mode in (SmartRobotDrive.Mode.PercentVbus, SmartRobotDrive.Mode.Speed)

    def getDescription(self):
        return "SmartDrivetrain"

    def stopMotor(self):
        self._left_motor.set(ctre.ControlMode.Disabled, 0)
        self._right_motor.set(ctre.ControlMode.Disabled, 0)
        self.feed()
