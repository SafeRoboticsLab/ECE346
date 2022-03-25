#!/usr/bin/env python

import threading
import rospy
from copy import deepcopy
import numpy as np
from iLQR import iLQR, Track, EllipsoidObj
from traj_msgs.msg import TrajMsg
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool
import yaml, csv
import time


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
            self.params = yaml.load(file, Loader=yaml.FullLoader)

        # parameters for the ocp solver
        self.T = self.params['T']
        self.N = self.params['N']
        self.replan_dt = self.T / (self.N - 1)

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

        # objects to schedule trajectory publishing
        self.prev_x = None
        self.prev_y = None
        self.prev_t = None

        self.cur_t = None
        self.cur_pose = None
        self.cur_state = None

        self.last_pub_t = None
        self.controller_ready = False
        self.thread_lock = threading.Lock()

        # set up publiser to the reference trajectory and subscriber to the pose
        self.traj_pub = rospy.Publisher(ref_traj_topic, TrajMsg, queue_size=1)

        self.pose_sub = rospy.Subscriber(pose_topic,
                                         Odometry,
                                         self.odom_sub_callback,
                                         queue_size=1)

        self.controller_status_sub = rospy.Subscriber(
            "/controller_status",
            Bool,
            self.controller_status_callback,
            queue_size=10)

        # start planning thread
        threading.Thread(target=self.ilqr_pub_thread).start()

    def controller_status_callback(self, statusMsg):
        if not self.controller_ready:
            self.controller_ready = statusMsg.data
            rospy.loginfo("iLQR Starts")
            rospy.loginfo(self.controller_ready)

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

        # linear velocity
        if self.prev_t is not None:
            dx = x - self.prev_x
            dy = y - self.prev_y
            dt = cur_t.to_sec() - self.prev_t
            v = np.sqrt(dx * dx + dy * dy) / dt
            self.thread_lock.acquire()
            self.cur_t = odomMsg.header.stamp
            self.cur_state = np.array([x, y, v, psi])
            self.thread_lock.release()
        else:
            v = None

        self.prev_x = x
        self.prev_y = y
        self.prev_t = cur_t.to_sec()

    def ilqr_pub_thread(self):
        time.sleep(5)
        rospy.loginfo("iLQR Planning publishing thread started")
        while not rospy.is_shutdown():
            # determine if we need to publish
            self.thread_lock.acquire()

            since_last_pub = self.replan_dt if self.last_pub_t is None else (
                self.cur_t - self.last_pub_t).to_sec()
            if since_last_pub >= self.replan_dt:
                # print(since_last_pub)
                # make a copy of the data
                cur_t = deepcopy(self.cur_t)
                # if self.prev_plan is not None:
                # #     cur_state = self.prev_plan[:,1]
                if self.cur_state is not None:
                    cur_state = np.array(self.cur_state, copy=True)
                else:
                    cur_state = None
            self.thread_lock.release()

            if since_last_pub >= self.replan_dt and cur_state is not None and self.controller_ready:
                if self.prev_plan is None:
                    u_init = None
                else:
                    u_init = np.zeros((2, self.N))
                    u_init[:, :-1] = self.prev_control[:, 1:]

                # add in obstacle to solver
                ego_a = 0.5 / 2.0
                ego_b = 0.2 / 2.0
                ego_q = np.array([0, 5.6])[:, np.newaxis] 
                ego_Q = np.diag([ego_a**2, ego_b**2])
                static_obs = EllipsoidObj(q=ego_q, Q=ego_Q)
                static_obs_list = [static_obs for _ in range(self.N)]

                sol_x, sol_u, t_solve, status, theta, _, _, _ = self.ocp_solver.solve(
                    cur_state, u_init, record=False, obs_list=[static_obs_list])

                # contruct the new planning
                plan = TrajMsg()
                plan.header.stamp = cur_t
                plan.dt = self.T / (self.N - 1)
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
                latency = rospy.get_rostime() - cur_t
                # print(status, t_solve)
                # rospy.loginfo(latency.to_sec())

                # rospy.loginfo("Use " + str(t_solve) +
                #               " to plan with progress = " +
                #               str(theta[-1] - theta[0]))

    def run(self, debug=False):
        rospy.spin()