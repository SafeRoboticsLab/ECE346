#!/usr/bin/env python

import threading
import rospy
from copy import deepcopy
import numpy as np
from Track import Track
from iLQR import iLQR
from traj_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
import yaml

class Planning_MPC():

    def __init__(self,
                 track_file=None,
                 pose_topic='/zed2/zed_node/odom',
                 ref_traj_topic='/planning/trajectory',
                 params_file='modelparams.yaml'):
        '''
        Main class for the MPC trajectory planner
        Input:
            freq: frequence to publish the control input to ESC and Servo
            T: prediction time horizon for the MPC
            N: number of integration steps in the MPC
        '''
        # load parameters
        with open(params_file) as file:
            self.params = yaml.load(file, Loader= yaml.FullLoader)
            
        # parameters for the ocp solver
        self.T = self.params['T']
        self.N = self.params['N']
        self.replan_dt = self.params['repaln_dt']
        
        self.L = self.params['L'] # wheel base (m)
        # distance from pose center to the rear axis (m)
        self.d_r = self.params['d_r']

        # previous trajectory for replan
        self.prev_plan = None
        self.prev_control = None

        # create track
        if track_file is None:
            # make a circle with 1.5m radius
            r = 1

            theta = np.linspace(0, 2 * np.pi, 100, endpoint=True)
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            self.track = Track(np.array([x, y]), 0.5, 0.5, True)
        else:
            self.track = Track()
            self.track.load_from_file(track_file)

        # set up the optimal control solver

        self.ocp_solver = iLQR(self.track, params_file=params_file)

        rospy.loginfo("Successfully initialized the solver with horizon " +
                      str(self.T) + "s, and " + str(self.N) + " steps.")

        # objects to schedule trajectory publishing
        self.cur_t = None
        self.cur_pose = None
        self.cur_state = None

        self.last_pub_t = None
        self.thread_lock = threading.Lock()

        # set up publiser to the reference trajectory and subscriber to the pose
        self.traj_pub = rospy.Publisher(ref_traj_topic,
                                        Trajectory,
                                        queue_size=1)
        
        self.pose_sub = rospy.Subscriber(pose_topic, Odometry,
                                             self.odom_sub_callback)

        # start planning thread
        threading.Thread(target=self.ilqr_pub_thread).start()
    
    def odom_sub_callback(self, odomMsg):
        """
        Subscriber callback function of the robot pose
        """        
        # postion
        x = odomMsg.pose.pose.position.x
        y = odomMsg.pose.pose.position.y
        
        # pose
        r = Rotation.from_quat([
            odomMsg.pose.pose.orientation.x,
            odomMsg.pose.pose.orientation.y,
            odomMsg.pose.pose.orientation.z,
            odomMsg.pose.pose.orientation.w
        ])
        
        rot_vec = r.as_rotvec()
        psi = rot_vec[2]
        
        # linear velocity
        vx = odomMsg.twist.twist.linear.x
        vy = odomMsg.twist.twist.linear.y
        v = (vx**2+vy**2)**0.5
        self.thread_lock.acquire()
        self.cur_t = odomMsg.header.stamp
        self.cur_state = np.array([x, y, v, psi])
        self.thread_lock.release()
                        
    def ilqr_pub_thread(self):
        rospy.loginfo("iLQR Planning publishing thread started")
        while not rospy.is_shutdown():
            # determine if we need to publish
            self.thread_lock.acquire()

            since_last_pub = self.replan_dt if self.last_pub_t is None else (
                self.cur_t - self.last_pub_t).to_sec()
            if since_last_pub >= self.replan_dt:
                # make a copy of the data
                cur_t = deepcopy(self.cur_t)
                if self.cur_state is not None:
                    cur_state = np.array(self.cur_state, copy=True)
                else:
                    cur_state = None
            self.thread_lock.release()

            if since_last_pub >= self.replan_dt and cur_state is not None:
                if self.prev_plan is None:
                    u_init = None
                else:
                    u_init = np.zeros((2, self.N))
                    u_init[:, :-1] = self.prev_control[:, 1:]

                sol_x, sol_u, t_solve, _, theta = self.ocp_solver.solve(cur_state, u_init)

                # contruct the new planning
                plan = Trajectory()
                plan.header.stamp = cur_t
                plan.dt = self.T / self.N
                plan.step = self.N
                plan.x = sol_x[0, :].tolist()
                plan.y = sol_x[1, :].tolist()
                plan.vel = sol_x[2, :].tolist()
                plan.psi = sol_x[3, :].tolist()

                plan.throttle = sol_u[0, :].tolist()
                plan.steering = sol_u[1, :].tolist()

                self.traj_pub.publish(plan)
                self.last_pub_t = cur_t
                self.prev_plan = sol_x
                self.prev_control = sol_u

                rospy.loginfo("Use " + str(t_solve) +
                              " to plan with progress = " +
                              str(theta[-1]-theta[0]))

    def run(self, debug=False):
        rospy.spin()