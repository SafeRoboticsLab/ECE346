#!/usr/bin/env python

import threading
import rospy
import numpy as np
from iLQR import iLQR, Track, EllipsoidObj
from realtime_buffer import RealtimeBuffer
from traj_msgs.msg import TrajMsg
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from rc_control_msgs.msg import RCControl
import yaml, csv
import time


class State():
    def __init__(self, state, t) -> None:
        self.state = state
        self.t = t
        
class Plan():
    def __init__(self, x, u, K, t0, dt, N) -> None:
        self.nominal_x = x
        self.nominal_u = u
        self.K = K
        self.t0 = t0
        self.dt = dt
        self.N = N
    
    def get_policy(self, t):
        k = int(np.floor((t-self.t0).to_sec()/self.dt))
        if k>= self.N:
            rospy.logwarn("Try to retrive policy beyond horizon")
            x_k = self.nominal_x[:,-1]
            x_k[2:] = 0
            u_k = np.zeros(2)
            K_k = np.zeros((2,4))
        else:
            x_k = self.nominal_x[:,k]
            u_k = self.nominal_u[:,k]
            K_k = self.K[:,:,k]

        return x_k, u_k, K_k

        
class Planning_MPC():

    def __init__(self,
                track_file=None,
                pose_topic='/zed2/zed_node/odom',
                control_topic='/planning/trajectory',
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
            self.params = yaml.load(file, Loader=yaml.FullLoader)

        # parameters for the ocp solver
        self.T = self.params['T']
        self.N = self.params['N']
        self.d_open_loop = np.array(self.params['d_open_loop'])
        self.replan_dt = self.T / (self.N - 1)

        # create track
        if track_file is None:
            # make a circle with 1.5m radius
            r = 1

            theta = np.linspace(0, 2 * np.pi, 100, endpoint=True)
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            self.track = Track(np.array([x, y]), 0.5, 0.5, True)
        else:
            x = []
            y = []
            with open(track_file, newline='') as f:
                spamreader = csv.reader(f, delimiter=',')
                for i, row in enumerate(spamreader):
                    if i > 0:
                        x.append(float(row[0]))
                        y.append(float(row[1]))

            center_line = np.array([x, y])
            self.track = Track(center_line=center_line,
                            width_left=self.params['track_width_L'],
                            width_right=self.params['track_width_R'],
                            loop=True)

        # set up the optimal control solver

        self.ocp_solver = iLQR(self.track, params=self.params)

        rospy.loginfo("Successfully initialized the solver with horizon " +
                    str(self.T) + "s, and " + str(self.N) + " steps.")

        self.state_buffer = RealtimeBuffer()
        self.plan_buffer = RealtimeBuffer()

        # set up publiser to the reference trajectory and subscriber to the pose
        self.control_pub = rospy.Publisher(control_topic, RCControl, queue_size=1)

        self.pose_sub = rospy.Subscriber(pose_topic,
                                        Odometry,
                                        self.odom_sub_callback,
                                        queue_size=1)
    
        # start planning thread
        threading.Thread(target=self.ilqr_pub_thread).start()

    def odom_sub_callback(self, odomMsg):
        """
        Subscriber callback function of the robot pose
        """
        cur_t = odomMsg.header.stamp
        # postion
        x = odomMsg.pose.pose.position.x
        y = odomMsg.pose.pose.position.y

        # pose
        r = Rotation.from_quat([
            odomMsg.pose.pose.orientation.x, odomMsg.pose.pose.orientation.y,
            odomMsg.pose.pose.orientation.z, odomMsg.pose.pose.orientation.w
        ])

        rot_vec = r.as_rotvec()
        psi = rot_vec[2]
        
        # get previous state
        prev_state = self.state_buffer.readFromRT()

        # linear velocity
        if prev_state is not None:
            dx = x - prev_state.state[0]
            dy = y - prev_state.state[1]
            dt = (cur_t-prev_state.t).to_sec()
            v = np.sqrt(dx * dx + dy * dy) / dt
        else:
            v = 0
        cur_X = np.array([x, y, v, psi])
        # obtain the latest plan
        last_plan = self.plan_buffer.readFromRT()
        if last_plan is not None:
            # get the control policy
            X_k, u_k, K_k = last_plan.get_policy(cur_t)
            u = u_k+ K_k@(cur_X - X_k)           
            self.publish_control(v, u, cur_t)
        
        # write the new pose to the buffer
        self.state_buffer.writeFromNonRT(State(cur_X, cur_t))

    def publish_control(self, v, u, cur_t):
        control = RCControl()
        control.header.stamp = cur_t
        a = u[0]
        delta = -u[1]
        
        if a<0:
            d = a/10-0.5
        else:
            temp = np.array([v**3, v**2, v, a**3, a**2, a, v**2*a, v*a**2, v*a, 1])
            d = temp@self.d_open_loop
            d = d+min(delta*delta*0.5,0.05)
        
        control.throttle = np.clip(d, -1.0, 1.0)
        control.steer = np.clip(delta/0.3, -1.0, 1.0)
        control.reverse = False
        self.control_pub.publish(control)


    def ilqr_pub_thread(self):
        time.sleep(5)
        rospy.loginfo("iLQR Planning publishing thread started")
        while not rospy.is_shutdown():
            # determine if we need to publish
            
            cur_state = self.state_buffer.readFromRT()
            prev_plan = self.plan_buffer.readFromRT()
            if cur_state is None:
                continue
            since_last_pub = self.replan_dt if prev_plan is None else (
                cur_state.t - prev_plan.t0).to_sec()
            if since_last_pub >= self.replan_dt:
                if prev_plan is None:
                    u_init = None
                else:
                    u_init = np.zeros((2, self.N))
                    u_init[:, :-1] = prev_plan.nominal_u[:, 1:]

                # add in obstacle to solver
                # ego_a = 0.5 / 2.0
                # ego_b = 0.2 / 2.0
                # ego_q = np.array([0, 5.6])[:, np.newaxis] 
                # ego_Q = np.diag([ego_a**2, ego_b**2])
                # static_obs = EllipsoidObj(q=ego_q, Q=ego_Q)
                # static_obs_list = [static_obs for _ in range(self.N)]
                
                sol_x, sol_u, _, _, _, sol_K, _, _ = self.ocp_solver.solve(
                    cur_state.state, u_init, record=True, obs_list=[])
                # print(np.round(sol_x,2))
                # print(np.round(sol_u[1,:],2))
                cur_plan = Plan(sol_x, sol_u, sol_K, cur_state.t, self.replan_dt, self.N)
                self.plan_buffer.writeFromNonRT(cur_plan)
                

    def run(self):
        rospy.spin() 