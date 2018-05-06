#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn 
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# NOTE: THIS API IS ALPHA AND WILL MOST LIKELY CHANGE!
#       ... if you have better ideas on how to implement, submit a patch!
#
import math

from pyfrc.physics import drivetrains, tankmodel, motor_cfgs
from pyfrc.physics.units import units


class PhysicsEngine(object):
    
    def __init__(self, physics_controller):
        '''
            :param physics_controller: `pyfrc.physics.core.PhysicsInterface` object
                                       to communicate simulation effects to
        '''
        
        self.physics_controller = physics_controller
        self.physics_controller.add_device_gyro_channel('navxmxp_spi_4_angle')

        r_int = 1.030
        r_kV = 0.742
        r_kA = 0.312

        l_int = 1.010
        l_kV = 0.758
        l_kA = 0.299
        volts_per_fps = units.volts / (units.inch / units.second)
        volts_per_acc = units.volts / (units.inch / units.second**2)
        self.drive = tankmodel.TankModel(motor_config=motor_cfgs.MOTOR_CFG_CIM,
                                         robot_mass=135 * units.lbs,
                                         x_wheelbase=2 * units.feet,
                                         robot_width=24*units.inch, robot_length=33*units.inch,
                                         l_kv=l_kV * units.tm_kv, r_kv=r_kV * units.tm_kv,
                                         l_ka=l_kA * units.tm_ka, r_ka=r_kA * units.tm_ka,
                                         l_vi=l_int * units.volts, r_vi=r_int * units.volts)
        # self.drive._bm = 1
        # self.drive._inertia = 1
        self.l_encoder = 0
        self.r_encoder = 0
            
    def update_sim(self, hal_data, now, tm_diff):
        '''
            Called when the simulation parameters for the program need to be
            updated.
            
            :param now: The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
        '''
        
        # Simulate the drivetrain
        l_motor = -hal_data['CAN'][0]['value']
        r_motor = hal_data['CAN'][2]['value']

        def calc_nophys():
            speed, rotation = drivetrains.two_motor_drivetrain(l_motor, r_motor, speed=16,
                                                               x_wheelbase=2)
            self.physics_controller.drive(speed, rotation, tm_diff)
            ENCODER_TICKS_PER_FT = 4096 / ((6 * math.pi) / 12)
            self.l_encoder = int(-l_motor * 16 * tm_diff * ENCODER_TICKS_PER_FT)
            self.r_encoder = int(r_motor * 16 * tm_diff * ENCODER_TICKS_PER_FT)
            hal_data['CAN'][0]['quad_position'] += self.l_encoder
            hal_data['CAN'][2]['quad_position'] += -self.r_encoder

        def calc_phys():
            x,y,t = self.drive.get_distance(l_motor, r_motor, tm_diff)
            self.physics_controller.distance_drive(x,y,t)
            ENCODER_TICKS_PER_FT = 4096 / ((6*math.pi)/12)
            self.l_encoder = int(self.drive.l_position * ENCODER_TICKS_PER_FT)
            self.r_encoder = int(self.drive.r_position * ENCODER_TICKS_PER_FT)
            hal_data['CAN'][0]['quad_position'] = self.l_encoder
            hal_data['CAN'][2]['quad_position'] = -self.r_encoder

        calc_nophys()
