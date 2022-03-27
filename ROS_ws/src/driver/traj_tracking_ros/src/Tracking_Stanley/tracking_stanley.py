#!/usr/bin/env python
import queue
import rospy
from dynamic_reconfigure.server import Server

from traj_msgs.msg import TrajMsg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rc_control_msgs.msg import RCControl
from std_msgs.msg import Bool

import numpy as np
from pyspline.pyCurve import Curve
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation
from .realtime_buffer import RealtimeBuffer
from .utils import *
import yaml
from threading import Lock
import pickle


class Trajectory(object):
    """
    Creata lists of x, y, psi, vel using  fitting 
    based on the x, y, psi, vel of the Trajectory message
    """

    def __init__(self, msg: TrajMsg, L=0.257):
        '''
        Decode the ros message and apply cubic interpolation 
        '''
        self.t_0 = msg.header.stamp.to_sec()
        self.dt = msg.dt
        self.step = msg.step
        self.t = np.arange(self.step) * self.dt
        self.t_final = self.step * self.dt

        # Convert the trajectory planned at rear axis to the front axis
        x_f = msg.x + np.cos(msg.psi) * L
        y_f = msg.y + np.sin(msg.psi) * L

        # use b-spline to fit trajectory
        self.ref_traj_f = Curve(x=msg.x, y=msg.y, k=3)

        # use cubic spline to fit time dependent v, psi, pos (x, y)
        self.ref_x_f = CubicSpline(self.t, np.array(x_f))
        self.ref_y_f = CubicSpline(self.t, np.array(y_f))
        self.ref_v_r = CubicSpline(self.t, np.array(msg.vel))
        self.ref_psi_r = CubicSpline(self.t, np.array(msg.psi))
        self.loop = False

        self.length = self.ref_traj_f.getLength()
        theta_sample = np.linspace(0, 1, self.step * 10,
                                   endpoint=False) * self.length
        self.data, slope = self.interp(theta_sample)

    def get_closest_pt(self, x, y, psi, t):
        '''
        Points have [2xn] shape
        '''

        s, _ = self.ref_traj_f.projectPoint(np.array([x, y]), eps=1e-3)
        closest_pt = self.ref_traj_f.getValue(s)

        # Instead of using the slope of the spline, use the actual psi from the planner
        deri = self.ref_traj_f.getDerivative(s)
        slope_spline = np.arctan2(deri[1], deri[0])

        slope = self.get_psi_ref(t)

        error_spline = np.sin(slope_spline) * (x - closest_pt[0]) \
                        - np.cos(slope_spline) * (y - closest_pt[1])

        error = np.sin(psi) * (x - closest_pt[0]) \
                        - np.cos(psi) * (y - closest_pt[1])
        
        return slope, slope_spline, error, error_spline

    def get_pos_ref(self, t):
        dt = t.to_sec() - self.t_0
        return self.ref_x_f(dt), self.ref_y_f(dt)

    def get_v_ref(self, t):
        dt = t.to_sec() - self.t_0
        return self.ref_v_r(dt)

    def get_psi_ref(self, t):
        dt = t.to_sec() - self.t_0
        return self.ref_psi_r(dt)

    def _interp_s(self, s):
        '''
        Given a list of s (progress since start), return corresponing (x,y) points  
        on the track. In addition, return slope of trangent line on those points
        '''
        n = len(s)

        interp_pt = self.ref_traj_f.getValue(s)
        slope = np.zeros(n)

        for i in range(n):
            deri = self.ref_traj_f.getDerivative(s[i])
            slope[i] = np.arctan2(deri[1], deri[0])
        return interp_pt.T, slope

    def interp(self, theta_list):
        '''
        Given a list of theta (progress since start), return corresponing (x,y) points  
        on the track. In addition, return slope of trangent line on those points
        '''
        if self.loop:
            s = np.remainder(theta_list, self.length) / self.length
        else:
            s = np.array(theta_list) / self.length
            s[s > 1] = 1
        return self._interp_s(s)


class Tracking_Stanley(object):

    def __init__(self,
                 vicon_pose=False,
                 pose_topic='/zed2/zed_node/odom',
                 ref_traj_topic='/simple_trajectory_topic',
                 controller_topic='/rc_control',
                 params_file='stanley.yaml'):

        self.vicon_pose = vicon_pose

        # objects to calculate the velocity
        self.prev_pos_r = None
        self.prev_t = None
        self.prev_x = None
        self.prev_y = None
        self.prev_ev = None
        self.prev_epsi = None

        self.prev_d = 0

        # real time buffer for planned trajectory
        self.traj_buffer = RealtimeBuffer()

        self.t_0 = None

        with open(params_file) as file:
            params = yaml.load(file, Loader=yaml.FullLoader)

        # parameters
        self.dyn_reconfig_lock = Lock()
        self.p_lat = params['p_lat']
        self.p_lon = params['p_lon']
        # positional gain for pursuiting actual ground truth x, y
        self.p_lon_complement = params['p_lon_complement']

        self.p_psi = params["p_psi"]

        # integration term
        self.e_int = 0  #
        self.i_lon = params['i_lon']
        self.i_sat = params['i_sat']

        # derivative term
        self.d_lon = params['d_lon']
        self.d_psi = params["d_psi"]

        self.L = params['L']  # wheel base (m)
        # distance from pose center to the front axis (m)
        self.d_f = params['d_f']
        # distance from pose center to the rear axis (m)
        self.d_r = self.d_f - self.L

        # set up subscriber to the reference trajectory and pose
        self.course_sub = rospy.Subscriber(ref_traj_topic, TrajMsg,
                                           self.ref_traj_sub_callback)

        if self.vicon_pose:
            self.pose_sub = rospy.Subscriber(pose_topic, TransformStamped,
                                             self.vicon_sub_callback)
        else:
            self.pose_sub = rospy.Subscriber(pose_topic,
                                             Odometry,
                                             self.odom_sub_callback,
                                             queue_size=10)
        # set up publisher to the low-level ESC and servo controller
        self.control_pub = rospy.Publisher(controller_topic,
                                           RCControl,
                                           queue_size=1)

        self.controller_status_pub = rospy.Publisher("/controller_status",
                                                     Bool,
                                                     queue_size=10)

    def ref_traj_sub_callback(self, msg: TrajMsg):
        """
        Subscriber callback function of the reference trajectory
        """
        self.traj_buffer.writeFromNonRT(Trajectory(msg=msg))

    def odom_sub_callback(self, odomMsg):
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

        # position of front axis
        x_f = x + np.cos(psi) * self.d_f
        y_f = y + np.sin(psi) * self.d_f

        # linear velocity
        if self.prev_t is not None:
            dx = x_f - self.prev_x
            dy = y_f - self.prev_y
            dt = cur_t.to_sec() - self.prev_t
            v = np.sqrt(dx * dx + dy * dy) / dt

            # publish the tracking status
            msg = Bool()
            msg.data = True
            self.controller_status_pub.publish(msg)
        else:
            v = None
        if v is not None:
            cur_traj = self.traj_buffer.readFromRT()

            if cur_traj is not None:
                ref_v = cur_traj.get_v_ref(cur_t)
                delta = self.lateral_control(cur_traj, x_f, y_f, psi, v, cur_t, dt)
                d = self.longitudial_control(cur_traj.get_pos_ref(cur_t),
                                             [x_f, y_f], v, ref_v, psi, dt)
                d = d + abs(delta) * 0.01
                # constraint for acceleration
                d = np.clip(
                    (d - self.prev_d) / dt, -0.1, 0.1) * dt + self.prev_d

                self.publish_control(cur_t, d, delta)

                self.prev_d = d

        self.prev_x = x_f
        self.prev_y = y_f
        self.prev_t = cur_t.to_sec()

    def vicon_sub_callback(self, msg):
        """
        Subscriber callback function of the robot pose
        """
        # Convert the current pose msg into [x,y,z,yaw]
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        # z = msg.transform.translation.z

        r = Rotation.from_quat([
            msg.transform.rotation.x, msg.transform.rotation.y,
            msg.transform.rotation.z, msg.transform.rotation.w
        ])

        rot_vec = r.as_rotvec()
        cur_psi = rot_vec[2]
        cur_t = msg.header.stamp

        # position of rear axis
        x_r = x + np.cos(cur_psi) * self.d_r
        y_r = y + np.sin(cur_psi) * self.d_r

        # position of front axis
        x_f = x + np.cos(cur_psi) * self.d_f
        y_f = y + np.sin(cur_psi) * self.d_f

        if self.prev_t is not None:
            # approximate the velocity
            dt = (cur_t - self.prev_t).to_sec()
            cur_traj = self.traj_buffer.readFromRT()
            cur_v_r = (((x_r - self.prev_state[0])**2 +
                        (y_r - self.prev_state[1])**2)**0.5) / dt
            if cur_traj is not None:
                ref_v = cur_traj.get_v_ref(cur_t)

                self.dyn_reconfig_lock.acquire()
                delta = self.lateral_control(cur_traj, x_f, y_f, cur_psi,
                                             cur_v_r, cur_t)
                d = self.longitudial_control(cur_traj.get_pos_ref(cur_t),
                                             [x_f, y_f], cur_v_r, ref_v, dt)
                self.dyn_reconfig_lock.release()

                self.publish_control(cur_t, d, delta)

        # Update the state and time of pose estimation
        self.prev_state = np.array([x_r, y_r])
        self.prev_t = cur_t

    def publish_control(self, cur_t, d, delta):
        control = RCControl()
        control.header.stamp = cur_t
        #! MAP VALUE OF STANLEY OUTPUT TO THROTTLE AND STEERING
        control.throttle = np.clip(d, -1.0, 1.0)
        control.steer = np.clip(delta, -1.0, 1.0)
        control.reverse = False
        self.control_pub.publish(control)

    def lateral_control(self, cur_traj, x_f, y_f, psi, v, cur_t, dt):
        """
        slope: reference psi from ilqr
        slope_spline: reference psi from where the car is wrt to reference spline
        error: error of car's current psi and slope
        error_spline: error of car's curretn psi and slope_spline
        """
        slope, slope_spline, error, error_spline = cur_traj.get_closest_pt(x_f, y_f, psi, cur_t)

        # theta_e corrects the heading error
        e_psi = normalize_angle(slope_spline - psi)
        theta_e = e_psi* self.p_psi
        
        # theta_d contouring error
        # cap v so that this will not go to inf
        theta_d = np.arctan2(self.p_lat * error_spline, 1+v)
        # print(e_psi, error_spline)
        if self.prev_epsi is not None:
            epsi_dev = (e_psi - self.prev_epsi) / dt
        else:
            epsi_dev = e_psi / dt
        self.prev_epsi = e_psi

        # Steering control
        delta = theta_e + theta_d - epsi_dev * self.d_psi

        # print("theta_e: {:.3f}, theta_d: {:.3f}, delta: {:.3f}".format(theta_e, theta_d, delta))

        return -1.0 * delta

    def longitudial_control(self, pos_ref, cur_pos, cur_v, ref_v, cur_psi, dt):
        e_v = ref_v - cur_v
        # print("e_v:\t\t", e_v)

        # reset the integral term
        if abs(e_v) < 0.002:
            self.e_int = 0
        else:
            self.e_int = np.clip(self.e_int + e_v, -self.i_sat, self.i_sat)

        # derivate term
        if self.prev_ev is not None:
            ev_dev = (e_v - self.prev_ev) / dt
        else:
            ev_dev = e_v / dt
        self.prev_ev = e_v

        # # check to see where ground truth is
        dpos = np.array(cur_pos) - np.array(pos_ref)
        e_pos = np.sqrt(dpos @ dpos.T)

        # # check to see if the car is in front or behind the ground truth
        # heading_angle = np.arctan2(dpos[1], dpos[0])
        # is_front = False
        # if abs(
        #         np.arctan2(np.sin(heading_angle - cur_psi),
        #                    np.cos(heading_angle - cur_psi))) < np.pi * 0.5:
        #     is_front = True

        d = self.p_lon*e_v                  \
            + self.i_lon*self.e_int         \
            + e_pos * self.p_lon_complement \
            - ev_dev * self.d_lon

        # if is_front:
        #     d = 0

        return d