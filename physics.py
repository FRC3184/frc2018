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

        self.drive = tankmodel.TankModel.theory(motor_cfgs.MOTOR_CFG_CIM,
                                                robot_mass=120 * units.lbs,
                                                gearing=8.45, nmotors=2,
                                                x_wheelbase=2 * units.feet,
                                                wheel_diameter=6*units.inch)
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
        
        speed, rotation = drivetrains.two_motor_drivetrain(l_motor, r_motor, speed=16,
                                                           x_wheelbase=2)
        self.physics_controller.drive(speed, rotation, tm_diff)
        x,y,t = self.drive.get_distance(l_motor, r_motor, tm_diff)
        # self.physics_controller.distance_drive(x,y,t)
        ENCODER_TICKS_PER_FT = 4096 / ((6*math.pi)/12)
        self.l_encoder += int(-l_motor * 16 * tm_diff * ENCODER_TICKS_PER_FT)
        self.r_encoder += int(r_motor * 16 * tm_diff * ENCODER_TICKS_PER_FT)
        hal_data['CAN'][0]['quad_position'] = self.l_encoder
        hal_data['CAN'][2]['quad_position'] = -self.r_encoder
