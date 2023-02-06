import rospy
from .ros_utility import get_ros_param
from .state_2d import State2D
import numpy as np

class GeneratePwm():
    '''
    This class apply an open-loop model to convert 
    acceleration and steering angle to PWM that can
    be read by the ESC and servo
    '''
    def __init__(self):
        '''
        Constructor for the GeneratePwm class
        '''        
        # Define the open-loop model
        self.d_open_loop = np.array([-0.0146, 0.0874, -0.0507, 0.0005, -0.0332, 0.1869, 0.0095, 0.0170, -0.0583, 0.0388])
        # Read the parameters from the parameter server
        self.read_parameters()
        
    def read_parameters(self):
        '''
        Read the maximum and minimum throttle for safety
        '''
        self.max_throttle = get_ros_param('~max_throttle', 0.5)
        self.min_throttle = get_ros_param('~min_throttle', -0.3)
        
    def convert(self, accel: float, steer: float, state: State2D):
        '''
        convert the acceleration and steering angle to PWM given the current state
        Parameters:
            accel: float, linear acceleration of the robot [m/s^2]
            steer: float, steering angle of the robot [rad]
            state: State2D, current state of the robot
        '''
        v = state.v_long        
        if accel<0:
            d = accel/10 - 0.5
        else:
            temp = np.array([v**3, v**2, v, accel**3, accel**2, accel, v**2*accel, v*accel**2, v*accel, 1])
            d = temp@self.d_open_loop
            # add a small offset to the throttle to compensate the friction of the steering
            d += min(steer*steer*0.8,0.05)
        
        # clip the throttle to the maximum and minimum throttle
        throttle_pwm = np.clip(d, self.min_throttle, self.max_throttle)
        
        # negative pwm means turn left (positive steering angle)
        steer_pwm = -np.clip(steer/0.3, -1, 1)
        
        return throttle_pwm, steer_pwm
        
        
        