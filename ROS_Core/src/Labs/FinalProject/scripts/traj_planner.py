#!/usr/bin/env python

import threading
import rospy
import numpy as np
import os
import time

from utils import RealtimeBuffer, get_ros_param, Policy, GeneratePwm, get_obstacle_vertices
from ILQR import RefPath
from ILQR import ILQR_jax as ILQR

from racecar_msgs.msg import ServoMsg, OdometryArray
from racecar_planner_optimized.cfg import plannerConfig
from visualization_msgs.msg import MarkerArray

from dynamic_reconfigure.server import Server
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg # used to display the trajectory on RVIZ
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import PoseStamped
import queue

import pickle
import os
import copy
import pylanelet

class TrajectoryPlanner():
    '''
    Main class for the Receding Horizon trajectory planner
    '''

    def __init__(self):

        f = open(os.path.dirname(os.path.realpath(__file__)) + "/track.pkl", "rb")
        self.lanelet_map = pickle.load(f)
        self.lanelet_map.plot_map()
        self.lanelet_map.build_graph(2)

        # Indicate if the planner is used to generate a new trajectory
        self.update_lock = threading.Lock()
        self.latency = 0.0

        goal_order = get_ros_param('~/goal_order',[1,2,3,4])
        self.goals = [get_ros_param(f'~/goal_{i}',[1,1]) for i in goal_order]
        self.goal_idx = 0

        self.read_parameters()

        # Initialize the PWM converter
        self.pwm_converter = GeneratePwm()

        # set up the optimal control solver
        self.setup_planner()

        # A goal message
        self.goal_msg = None

        # Manual input message from keyboard
        self.manual_msg = None

        self.setup_publisher()

        self.setup_subscriber()

        self.setup_service()

        # start planning and control thread
        threading.Thread(target=self.control_thread).start()
        threading.Thread(target=self.receding_horizon_planning_thread).start()

    def read_parameters(self):
        '''
        This function reads the parameters from the parameter server
        '''
        # Required parameters
        self.package_path = rospy.get_param('~package_path')

        self.receding_horizon = get_ros_param('~receding_horizon', False)

        # Read ROS topic names to subscribe
        self.odom_topic = get_ros_param('~odom_topic', '/slam_pose')
        self.path_topic = get_ros_param('~path_topic', '/Routing/Path')

        # Read ROS topic names to publish
        self.control_topic = get_ros_param('~control_topic', '/control/servo_control')
        self.keyboard_topic = get_ros_param('~keyboard_topic', '/Keyboard')
        self.traj_topic = get_ros_param('~traj_topic', '/Planning/Trajectory')
        self.static_obs_topic = get_ros_param('~static_obs_topic', '/Obstacles/Static')
        self.adj_path_topic = get_ros_param('~adj_path_topic', '/Planning/AdjustedPath')
        self.adj_path_topic_L = get_ros_param('~adj_path_topic', '/Planning/AdjustedPathL')
        self.adj_path_topic_R = get_ros_param('~adj_path_topic', '/Planning/AdjustedPathR')

        # Read the simulation flag,
        # if the flag is true, we are in simulation
        # and no need to convert the throttle and steering angle to PWM
        self.simulation = get_ros_param('~simulation', True)

        # Whether keyboard inputs are accepted
        self.keyboard_enabled = get_ros_param('~keyboard_enabled', False)

        # Read Planning parameters
        # if true, the planner will load a path from a file rather than subscribing to a path topic
        self.replan_dt = get_ros_param('~replan_dt', 0.1)

        # Read the ILQR parameters file, if empty, use the default parameters
        ilqr_params_file = get_ros_param('~ilqr_params_file', '')
        if ilqr_params_file == '':
            self.ilqr_params_abs_path = None
        elif os.path.isabs(ilqr_params_file):
            self.ilqr_params_abs_path = ilqr_params_file
        else:
            self.ilqr_params_abs_path = os.path.join(self.package_path, ilqr_params_file)

    def setup_planner(self):
        '''
        This function setup the ILQR solver
        '''
        # Initialize ILQR solver
        self.planner = ILQR(self.ilqr_params_abs_path)

        # create buffers to handle multi-threading
        self.plan_state_buffer = RealtimeBuffer()
        self.control_state_buffer = RealtimeBuffer()
        self.policy_buffer = RealtimeBuffer()
        self.path_buffer = RealtimeBuffer()
        self.static_obstacle_dict = {}
        self.static_obstacle_dict_persistent = {}
        self.planner_ready = True

    def setup_publisher(self):
        '''
        This function sets up the publisher for the trajectory
        '''
        # Publisher for the planned nominal trajectory for visualization
        self.trajectory_pub = rospy.Publisher(self.traj_topic, PathMsg, queue_size=1)

        # Publisher for the control command
        self.control_pub = rospy.Publisher(self.control_topic, ServoMsg, queue_size=1)

        # Publisher for adjusted path
        self.adj_path_pub = rospy.Publisher(self.adj_path_topic, PathMsg, queue_size=1)

        # Publisher for adjusted path, left boundary
        self.adj_path_pub_L = rospy.Publisher(self.adj_path_topic_L, PathMsg, queue_size=1)

        # Publisher for adjusted path, left boundary
        self.adj_path_pub_R = rospy.Publisher(self.adj_path_topic_R, PathMsg, queue_size=1)
    
        # Publisher for new goal
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    def setup_subscriber(self):
        '''
        This function sets up the subscriber for the odometry and path
        '''
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=10)

        # TODO (Task 2) - Write a corresponding publisher for the keyboard
        # ...
        
        self.path_sub = rospy.Subscriber(self.path_topic, PathMsg, self.path_callback, queue_size=10)
        self.static_obs_sub = rospy.Subscriber(self.static_obs_topic, MarkerArray, self.static_obstacle_callback, queue_size=10)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.replan_callback, queue_size=10)

    def static_obstacle_callback(self, msg):
        '''
        Static obstacle callback function
        '''

        # Keep track of obstacles
        for obs in msg.markers:
            id, vertices = get_obstacle_vertices(obs)
            xpos = np.mean(vertices[:,0])
            ypos = np.mean(vertices[:,1])

            # A new obstacle we have not seen before
            if obs.id not in self.static_obstacle_dict:
                print('new obstacle!!!')
                self.static_obstacle_dict_persistent[id] = vertices
                if self.goal_msg is not None:
                    self.goal_pub.publish(self.goal_msg)

            # An obstacle changed its position
            else:
                old_vertices = self.static_obstacle_dict_persistent[obs.id]
                old_xpos = np.mean(old_vertices[:,0])
                old_ypos = np.mean(old_vertices[:,1])
                distSq = (xpos - old_xpos) ** 2 + (ypos - old_ypos) ** 2
                if (distSq > 0.2):
                    print("obstacle moved!!!")
                    if self.goal_msg is not None:
                        self.goal_pub.publish(self.goal_msg)
                    self.static_obstacle_dict_persistent[id] = vertices
            self.static_obstacle_dict[id] = vertices

    def replan_callback(self, goal_msg):
        self.goal_msg = goal_msg

    def keyboard_callback(self, manual_msg):
        self.keyboard_throttle_pwm = manual_msg.throttle
        self.keyboard_steer_pwm = manual_msg.steer

    def setup_service(self):
        '''
        Set up ros service
        '''
        self.start_srv = rospy.Service('/planning/start_planning', Empty, self.start_planning_cb)
        self.stop_srv = rospy.Service('/planning/stop_planning', Empty, self.stop_planning_cb)

        self.dyn_server = Server(plannerConfig, self.reconfigure_callback)

    def start_planning_cb(self, req):
        '''
        ros service callback function for start planning
        '''
        rospy.loginfo('Start planning!')
        self.planner_ready = True
        return EmptyResponse()

    def stop_planning_cb(self, req):
        '''
        ros service callback function for stop planning
        '''
        rospy.loginfo('Stop planning!')
        self.planner_ready = False
        self.policy_buffer.reset()
        return EmptyResponse()

    def reconfigure_callback(self, config, level):
        self.update_lock.acquire()
        self.latency = config['latency']
        rospy.loginfo(f"Latency Updated to {self.latency} s")
        if self.latency < 0.0:
            rospy.logwarn(f"Negative latency compensation {self.latency} is not a good idea!")
        self.update_lock.release()
        return config

    def odometry_callback(self, odom_msg):
        '''
        Subscriber callback function of the robot pose
        '''
        # Add the current state to the buffer
        # Controller thread will read from the buffer
        # Then it will be processed and add to the planner buffer
        # inside the controller thread
        self.control_state_buffer.writeFromNonRT(odom_msg)

    def path_callback(self, path_msg):
        x = []
        y = []
        width_L = []
        width_R = []
        speed_limit = []

        '''
        TODO (Task 1) Start:
        Use the "pylanelet" library to alter the reference path for better obstacle avoidance

        Hint: Use the lanelets of the path, obstacle, and goal to proactively decide when to switch lanes!
        '''

        # Helper code: Keep track of the coordinates and closest lanelet ID of each obstacle
        # obs_info follows the format:
        # (obstacle x coordinate, obstacle y coordinate, closest lanelet ID)
        obs_info = []
        for corners in self.static_obstacle_dict.values():
            # Get the closest lanelet for this obstacle
            oL, _ = self.lanelet_map.get_closest_lanelet([np.mean(corners[:,0]), np.mean(corners[:,1])])
            obs_info.append((np.mean(corners[:,0]), np.mean(corners[:,1]), oL.id))

        # Helper code: Keep track of the coordinates and closest lanelet of each path segment (waypoint)
        # path_info follows the format:
        # (waypoint x coordinate, waypoint y coordinate, closest lanelet object, normalized position of the lanelet centerline)
        goal_idx = self.goal_idx
        path_info = []
        for it in range(len(path_msg.poses)):
            waypoint = path_msg.poses[it]
            wx = waypoint.pose.position.x
            wy = waypoint.pose.position.y
            wL, wLs = self.lanelet_map.get_closest_lanelet([wx, wy])
            path_info.append((wx, wy, wL, wLs))

        # Helper code: Keep track of the coordinates and closest lanelet of the goal
        goal_idx = self.goal_idx
        gL, _ = self.lanelet_map.get_closest_lanelet([self.goals[goal_idx][0], self.goals[goal_idx][1]])

        # TODO: This code is a placeholder! Replace it with better obstacle avoidance code.
        for waypoint in path_msg.poses:
            x.append(waypoint.pose.position.x)
            y.append(waypoint.pose.position.y)
            width_L.append(waypoint.pose.orientation.x)
            width_R.append(waypoint.pose.orientation.y)
            speed_limit.append(waypoint.pose.orientation.z)

        '''
        TODO (Task 1) End
        '''

        # Creates the centerline
        centerline = np.array([x, y])

        # Some helper code to show the updated path and boundaries on RViz
        self.adj_path_pub.publish(path_msg)
        path_msg_L = copy.deepcopy(path_msg)
        path_msg_R = copy.deepcopy(path_msg)
        for it in range(1, len(path_msg.poses)-1):
            waypointL = path_msg_L.poses[it]
            waypointR = path_msg_R.poses[it]
            dx = path_msg.poses[it].pose.position.x - path_msg.poses[it-1].pose.position.x
            dy = path_msg.poses[it].pose.position.y - path_msg.poses[it-1].pose.position.y
            dr = (dx * dx + dy * dy)**0.5
            dx, dy = -dy / dr, dx / dr
            waypointL.pose.position.x += dx * path_msg.poses[it].pose.orientation.x
            waypointL.pose.position.y += dy * path_msg.poses[it].pose.orientation.x
            waypointR.pose.position.x -= dx * path_msg.poses[it].pose.orientation.y
            waypointR.pose.position.y -= dy * path_msg.poses[it].pose.orientation.y
        self.adj_path_pub_L.publish(path_msg_L)
        self.adj_path_pub_R.publish(path_msg_R)

        try:
            ref_path = RefPath(centerline, width_L, width_R, speed_limit, loop=False)
            self.path_buffer.writeFromNonRT(ref_path)
            rospy.loginfo('Path received!')
        except:
            rospy.logwarn('Invalid path received! Move your robot and retry!')

    @staticmethod
    def compute_control(x, x_ref, u_ref, K_closed_loop):
        '''
        Given the current state, reference trajectory, control command
        and closed loop gain, compute the control command

        Args:
            x: np.ndarray, [dim_x] current state
            x_ref: np.ndarray, [dim_x] reference trajectory
            u_ref: np.ndarray, [dim_u] reference control command
            K_closed_loop: np.ndarray, [dim_u, dim_x] closed loop gain

        Returns:
            accel: float, acceleration command [m/s^2]
            steer_rate: float, steering rate command [rad/s]
        '''

        dx = x - x_ref
        dx[3] = np.mod(dx[3] + np.pi, 2 * np.pi) - np.pi
        u = u_ref + K_closed_loop @ dx
        accel = u[0]
        steer_rate = u[1]

        return accel, steer_rate

    def control_thread(self):
        '''
        Main control thread to publish control command
        '''
        rate = rospy.Rate(40)
        u_queue = queue.Queue()

        # values to keep track of the previous control command
        prev_state = None #[x, y, v, psi, delta]
        prev_u = np.zeros(3) # [accel, steer, t]

        # helper function to compute the next state
        def dyn_step(x, u, dt):
            dx = np.array([x[2]*np.cos(x[3]),
                        x[2]*np.sin(x[3]),
                        u[0],
                        x[2]*np.tan(u[1]*1.1)/0.257,
                        0
                        ])
            x_new = x + dx*dt
            x_new[2] = max(0, x_new[2]) # do not allow negative velocity
            x_new[3] = np.mod(x_new[3] + np.pi, 2 * np.pi) - np.pi
            x_new[-1] = u[1]
            return x_new

        while not rospy.is_shutdown():
            # initialize the control command
            accel = -5
            steer = 0
            state_cur = None
            policy = self.policy_buffer.readFromRT()

            if self.simulation:
                t_act = rospy.get_rostime().to_sec()
            else:
                self.update_lock.acquire()
                t_act = rospy.get_rostime().to_sec() + self.latency
                self.update_lock.release()


            # check if there is new state available
            if self.control_state_buffer.new_data_available:
                odom_msg = self.control_state_buffer.readFromRT()
                rx, ry = odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y
                gSqDist = (rx - self.goals[self.goal_idx][0]) ** 2 + (ry - self.goals[self.goal_idx][1]) ** 2
                if (gSqDist < 1.0):
                    print("found goal!")
                    if self.goal_idx == 0:
                        rospy.sleep(2)
                    self.goal_idx = (self.goal_idx + 1) % (len(self.goals) - 1)
                    gnx, gny = self.goals[self.goal_idx][0], self.goals[self.goal_idx][1]
                    goal_msg = PoseStamped()
                    goal_msg.pose.position.x = gnx
                    goal_msg.pose.position.y = gny
                    goal_msg.pose.orientation.x = 0.0
                    goal_msg.pose.orientation.y = 0.0
                    goal_msg.pose.orientation.z = 0.0
                    goal_msg.pose.orientation.w = 1.0
                    goal_msg.header.frame_id = "map"
                    self.goal_pub.publish(goal_msg)
                    self.goal_msg = goal_msg

                t_slam = odom_msg.header.stamp.to_sec()

                u = np.zeros(3)
                u[-1] = t_slam
                while not u_queue.empty() and u_queue.queue[0][-1] < t_slam:
                    u = u_queue.get() # remove old control commands

                # get the state from the odometry message
                q = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
                        odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
                # get the heading angle from the quaternion
                psi = euler_from_quaternion(q)[-1]

                state_cur = np.array([
                            odom_msg.pose.pose.position.x,
                            odom_msg.pose.pose.position.y,
                            odom_msg.twist.twist.linear.x,
                            psi,
                            u[1]
                        ])

                # predict the current state use past control command
                for i in range(u_queue.qsize()):
                    u_next = u_queue.queue[i]
                    dt = u_next[-1] - u[-1]
                    state_cur = dyn_step(state_cur, u, dt)
                    u = u_next

                # predict the cur state with the most recent control command
                state_cur = dyn_step(state_cur, u, t_act - u[-1])

                # update the state buffer for the planning thread
                plan_state = np.append(state_cur, t_act)
                self.plan_state_buffer.writeFromNonRT(plan_state)

            # if there is no new state available, we do one step forward integration to predict the state
            elif prev_state is not None:
                t_prev = prev_u[-1]
                dt = t_act - t_prev
                # predict the state using the last control command is executed
                state_cur = dyn_step(prev_state, prev_u, dt)

            # Generate control command from the policy
            if policy is not None:
                # get policy
                if not self.receding_horizon:
                    state_ref, u_ref, K = policy.get_policy_by_state(state_cur)
                else:
                    state_ref, u_ref, K = policy.get_policy(t_act)

                if state_ref is not None:
                    accel, steer_rate = self.compute_control(state_cur, state_ref, u_ref, K)
                    steer = max(-0.37, min(0.37, prev_u[1] + steer_rate*dt))
                else:
                    # reset the policy buffer if the policy is not valid
                    rospy.logwarn("Try to retrieve a policy beyond the horizon! Reset the policy buffer!")
                    self.policy_buffer.reset()

            # generate control command
            if not self.simulation and state_cur is not None:
                # If we are using robot,
                # the throttle and steering angle needs to convert to PWM signal
                throttle_pwm, steer_pwm = self.pwm_converter.convert(accel, steer, state_cur[2])
            else:
                throttle_pwm = accel
                steer_pwm = steer

            # publish control command
            servo_msg = ServoMsg()
            servo_msg.header.stamp = rospy.get_rostime() # use the current time to avoid synchronization issue
            servo_msg.throttle = throttle_pwm
            servo_msg.steer = steer_pwm
            self.control_pub.publish(servo_msg)

            # Record the control command and state for next iteration
            u_record = np.array([accel, steer, t_act])
            u_queue.put(u_record)
            prev_u = u_record
            prev_state = state_cur

            # end of while loop
            rate.sleep()

    def receding_horizon_planning_thread(self):
        '''
        This function is the main thread for receding horizon planning
        We repeatedly call ILQR to replan the trajectory (policy) once the new state is available
        '''

        # helper function to compute the next state
        def dyn_step(x, u, dt):
            dx = np.array([x[2]*np.cos(x[3]),
                        x[2]*np.sin(x[3]),
                        u[0],
                        x[2]*np.tan(u[1]*1.1)/0.257,
                        0
                        ])
            x_new = x + dx*dt
            x_new[2] = max(0, x_new[2]) # do not allow negative velocity
            x_new[3] = np.mod(x_new[3] + np.pi, 2 * np.pi) - np.pi
            x_new[-1] = u[1]
            return x_new

        rospy.loginfo('Receding Horizon Planning thread started waiting for ROS service calls...')
        t_last_replan = 0
        while not rospy.is_shutdown():
            # determine if we need to replan
            if self.plan_state_buffer.new_data_available:
                state_cur = self.plan_state_buffer.readFromRT()

                t_cur = state_cur[-1] # the last element is the time
                dt = t_cur - t_last_replan

                # Do replanning
                if dt >= self.replan_dt:
                    # Get the initial controls for hot start
                    init_controls = None

                    self.original_policy = self.policy_buffer.readFromRT()
                    if self.original_policy is not None:
                        init_controls = self.original_policy.get_ref_controls(t_cur)

                    # Update the path
                    if self.path_buffer.new_data_available:
                        new_path = self.path_buffer.readFromRT()
                        self.planner.update_ref_path(new_path)

                    # Update the static obstacles
                    obstacles_list = []
                    for vertices in self.static_obstacle_dict.values():
                        obstacles_list.append(vertices)

                    self.planner.update_obstacles(obstacles_list)

                    # Replan use ilqr
                    new_plan = self.planner.plan(state_cur[:-1], init_controls, verbose=False)

                    plan_status = new_plan['status']
                    if plan_status == -1:
                        rospy.logwarn_once('No path specified!')
                        continue

                    if self.planner_ready:
                        # If stop planning is called, we will not write to the buffer
                        new_policy = Policy(
                            X = new_plan['trajectory'],
                            U = new_plan['controls'],
                            K = new_plan['K_closed_loop'],
                            t0 = t_cur,
                            dt = self.planner.dt,
                            T = self.planner.T
                        )

                        self.policy_buffer.writeFromNonRT(new_policy)

                        # publish the new policy for RVIZ visualization
                        self.trajectory_pub.publish(new_policy.to_msg())
                        t_last_replan = t_cur