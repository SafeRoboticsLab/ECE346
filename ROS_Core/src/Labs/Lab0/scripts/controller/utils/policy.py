import numpy as np
import rospy
from nav_msgs.msg import Path as TrajMsg # used to display the trajectory on RVIZ
from .ros_utility import state_to_pose_stamped
'''
    A container class to store the feedback policy
'''
class Policy():
    def __init__(self, x, u, K, t0, dt, N) -> None:
        self.nominal_x = x
        self.nominal_u = u
        self.K = K
        self.t0 = t0
        self.dt = dt
        self.N = N
    
    def get_policy(self, t):
        '''
        Return the policy at time t
        '''
        i = self.get_index(t)
        if i>= (self.N-1):
            rospy.logwarn("Try to retrive policy beyond horizon")
            x_i =np.zeros_like(self.nominal_x[:,-1])
            u_i = np.zeros_like(self.nominal_u[:,0])
            K_i = np.zeros_like(self.K[:,:,0])
        else:
            x_i = self.nominal_x[:,i]
            u_i = self.nominal_u[:,i]
            K_i = self.K[:,:,i]

        return x_i, u_i, K_i
    

    def get_index(self,t):
        return int(np.ceil((t-self.t0)/self.dt))

    def get_ref_controls(self, t):
        '''
        Return the nominal control at time t and forward
        '''
        i = self.get_index(t)
        if i>= self.N:
            return None
        else:
            ref_u = np.zeros_like(self.nominal_u)
            ref_u[:,:self.N-i] = self.nominal_u[:,i:]

            return ref_u
        
    def to_msg(self, frame_id='map'):
        traj_msg = TrajMsg()
        traj_msg.header.frame_id = frame_id
        traj_msg.header.stamp = rospy.Time.from_sec(self.t0)
        
        states = self.nominal_x
        for i in range(self.N):
            t = self.t0 + i * self.dt
            pose = state_to_pose_stamped(states[:,i], t, frame_id)
            traj_msg.poses.append(pose)

        return traj_msg
    
    def __str__(self) -> str:
        return f"Policy: t0: {self.t0}, dt: {self.dt}, N: {self.N}\n"+\
                f"nominal_x: {self.nominal_x}\n"+\
                f"nominal_u: {self.nominal_u}\n"
