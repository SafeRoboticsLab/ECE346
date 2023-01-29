import numpy as np
from scipy.spatial.transform import Rotation as R
from tf.transformations import euler_from_quaternion

class State2D():
    '''
    2D vehicle state
    '''
    def __init__(self, odom_msg = None) -> None:

        self.t = 0 # time stamp
        self.x = 0 #x position
        self.y = 0 #y position
        self.psi = 0 # pose angle around z axis
        
        self.v_dir = 0 # direction of the velocity
        self.v_long = 0 # longitudinal velocity
        self.v_lat = 0 # lateral velocity
        
        self.w = 0 # angular velocity around z axis
        self.v = 0

        self.initialized = False
        
        if odom_msg is not None:
            self.from_odom_msg(odom_msg)

    def from_odom_msg(self, odom_msg):
        '''
        Construct state from odometry message
        '''
        self.t = odom_msg.header.stamp.to_sec()
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        q = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, 
                odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        self.psi = euler_from_quaternion(q)[-1]
        
        self.v_long = odom_msg.twist.twist.linear.x
        self.v_lat = odom_msg.twist.twist.linear.y

        self.v = np.sqrt(self.v_long*self.v_long + self.v_lat*self.v_lat)
        self.v_dir = np.arctan2(self.v_lat, self.v_long)

        self.w = odom_msg.twist.twist.angular.z
        
        self.initialized = True
        
    def from_SE3(self, pose_base_to_world, twist_base, t):
        '''
        Construct state from SE3 pose and twist
            pose_base_to_world: SE3 pose from world to base
            twist_base: twist of the base
        '''
        self.t = t
        self.x = pose_base_to_world[0,3]
        self.y = pose_base_to_world[1,3]
        
        rot = R.from_matrix(pose_base_to_world[:3,:3])
        self.psi = rot.as_euler('zyx', degrees=False)[0]
        
        self.v_long = twist_base[0,3]
        self.v_lat = twist_base[1,3]
        
        self.v = np.sqrt(self.v_long*self.v_long + self.v_lat*self.v_lat)
        self.v_dir = np.arctan2(self.v_lat, self.v_long)
        
        self.w = twist_base[2,0] # angular velocity around z axis
        
        self.initialized = True
    
    def from_state(self, x, y, v_long, psi, w, t):
        '''
        Construct state from a state object
        '''
        self.t = t
        self.x = x
        self.y = y
        self.psi = psi
        
        self.w = w
        
        # assume no slip
        self.v = v_long
        self.v_dir = 0 
        self.v_long = v_long
        self.v_lat = 0
        
        self.initialized = True

    def state_vector(self, delta):
        '''
        Return the state vector
        '''
        return np.array([self.x, self.y, self.v_long, self.psi, delta])
    
    def transformation_matrix(self):
        '''
        Retrun Translation matrix from world to base
        '''
        T = np.array([[np.cos(self.psi), -np.sin(self.psi), self.x],
                    [np.sin(self.psi), np.cos(self.psi), self.y],
                    [0, 0, 1]])
        return T
        
    def __str__(self):
        return f"State at [{np.round(self.x,3)}, {np.round(self.y,3)}] "+ \
            f"with pose {np.round(np.rad2deg(self.psi),3)} deg\n" + \
            f"Speed: {np.round(self.v,3)} pointing to {np.round(np.rad2deg(self.v_dir),3)} deg; Omega: {np.round(self.w,3)} \n"+ \
            f"v_long: {self.v_long}; v_lat: {self.v_lat}; \n"
