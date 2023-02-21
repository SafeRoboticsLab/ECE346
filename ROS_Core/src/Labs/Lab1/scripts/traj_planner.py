#!/usr/bin/env python

import threading
import rospy
import numpy as np
import os
import time

from utils import RealtimeBuffer, get_ros_param, Policy, GeneratePwm
from ILQR import RefPath
from ILQR import ILQR

from racecar_msgs.msg import ServoMsg 
from racecar_planner.cfg import plannerConfig

from dynamic_reconfigure.server import Server
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg # used to display the trajectory on RVIZ
from std_srvs.srv import Empty, EmptyResponse
import queue

class TrajectoryPlanner():
    '''
    Main class for the Receding Horizon trajectory planner
    '''

    def __init__(self):
        # Indicate if the planner is used to generate a new trajectory
        self.update_lock = threading.Lock()
        self.latency = 0.0
        
        self.read_parameters()
        
        # Initialize the PWM converter
        self.pwm_converter = GeneratePwm()
        
        # set up the optimal control solver
        self.setup_planner()
        
        self.setup_publisher()
        
        self.setup_subscriber()

        self.setup_service()

        # start planning and control thread
        threading.Thread(target=self.control_thread).start()
        if not self.receding_horizon:
            threading.Thread(target=self.policy_planning_thread).start()
        else:
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
        self.obstacle_topic = get_ros_param('~obstacle_topic', '/prediction/obstacles')
        
        # Read ROS topic names to publish
        self.control_topic = get_ros_param('~control_topic', '/control/servo_control')
        self.traj_topic = get_ros_param('~traj_topic', '/Planning/Trajectory')
        
        # Read the simulation flag, 
        # if the flag is true, we are in simulation 
        # and no need to convert the throttle and steering angle to PWM
        self.simulation = get_ros_param('~simulation', True)
        
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
        self.obstacle_buffer = RealtimeBuffer()
        # Indicate if the planner is ready to generate a new trajectory
        self.planner_ready = True

    def setup_publisher(self):
        '''
        This function sets up the publisher for the trajectory
        '''
        # Publisher for the planned nominal trajectory for visualization
        self.trajectory_pub = rospy.Publisher(self.traj_topic, PathMsg, queue_size=1)

        # Publisher for the control command
        self.control_pub = rospy.Publisher(self.control_topic, ServoMsg, queue_size=1)

    def setup_subscriber(self):
        '''
        This function sets up the subscriber for the odometry and path
        '''
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=10)
        self.path_sub = rospy.Subscriber(self.path_topic, PathMsg, self.path_callback, queue_size=10)

    def setup_service(self):
        '''
        Set up ros service
        '''
        self.start_srv = rospy.Service('/Planning/Start', Empty, self.start_planning_cb)
        self.stop_srv = rospy.Service('/Planning/Stop', Empty, self.stop_planning_cb)
        
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
        
        for waypoint in path_msg.poses:
            x.append(waypoint.pose.position.x)
            y.append(waypoint.pose.position.y)
            width_L.append(waypoint.pose.orientation.x)
            width_R.append(waypoint.pose.orientation.y)
            speed_limit.append(waypoint.pose.orientation.z)
                    
        centerline = np.array([x, y])
        
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

        ###############################
        #### TODO: Task 2 #############
        ###############################
        # Implement your control law here using ILQR policy
        # Hint: make sure that the difference in heading is between [-pi, pi]
        
        accel = 0 # TO BE REPLACED
        steer_rate = 0 # TO BE REPLACED

        ##### END OF TODO ##############

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
            
            # take the latency of publish into the account
            self.update_lock.acquire()
            t_act = rospy.get_rostime().to_sec() + self.latency 
            self.update_lock.release()
            
            # check if there is new state available
            if self.control_state_buffer.new_data_available:
                odom_msg = self.control_state_buffer.readFromRT()
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

    def policy_planning_thread(self):
        '''
        This function is the main thread for open loop planning
        We plan entire trajectory (policy) everytime when a new reference path is available
        '''
        rospy.loginfo('Policy Planning thread started waiting for ROS service calls...')
        while not rospy.is_shutdown():
            # determine if we need to replan
            if self.path_buffer.new_data_available and self.planner_ready:
                new_path = self.path_buffer.readFromRT()
                self.planner.update_ref_path(new_path)
                
                # check if there is an existing policy
                original_policy = self.policy_buffer.readFromRT()
                if original_policy is not None:
                    # reset the buffer, which will stop the car from moving
                    self.policy_buffer.reset() 
                    time.sleep(2) # wait for 2 seconds to make sure the car is stopped
                rospy.loginfo('Planning a new policy...')
                # Get current state
                state = self.plan_state_buffer.readFromRT()[:-1] # the last element is the time
                prev_progress = -np.inf
                _, _, progress = new_path.get_closest_pts(state[:2])
                progress = progress[0]
                
                nominal_trajectory = []
                nominal_controls = []
                K_closed_loop = []
                
                # stop when the progress is not increasing
                while (progress - prev_progress)*new_path.length > 1e-3: # stop when the progress is not increasing
                    nominal_trajectory.append(state)
                    new_plan = self.planner.plan(state, None, verbose=False)
                    nominal_controls.append(new_plan['controls'][:,0])
                    K_closed_loop.append(new_plan['K_closed_loop'][:,:,0])
                    
                    # get the next state and its progress
                    state = new_plan['trajectory'][:,1]
                    prev_progress = progress
                    _, _, progress = new_path.get_closest_pts(state[:2])
                    progress = progress[0]
                    print('Planning progress %.4f' % progress, end='\r')

                nominal_trajectory = np.array(nominal_trajectory).T # (dim_x, N)
                nominal_controls = np.array(nominal_controls).T # (dim_u, N)
                K_closed_loop = np.transpose(np.array(K_closed_loop), (1,2,0)) # (dim_u, dim_x, N)
                
                T = nominal_trajectory.shape[-1] # number of time steps
                t0 = rospy.get_rostime().to_sec()
            
                # If stop planning is called, we will not write to the buffer
                new_policy = Policy(X = nominal_trajectory, 
                                    U = nominal_controls,
                                    K = K_closed_loop, 
                                    t0 = t0, 
                                    dt = self.planner.dt,
                                    T = T)
                
                self.policy_buffer.writeFromNonRT(new_policy)
                
                rospy.loginfo('Finish planning a new policy...')
                
                # publish the new policy for RVIZ visualization
                self.trajectory_pub.publish(new_policy.to_msg())        

    def receding_horizon_planning_thread(self):
        '''
        This function is the main thread for receding horizon planning
        We repeatedly call ILQR to replan the trajectory (policy) once the new state is available
        '''
        
        rospy.loginfo('Receding Horizon Planning thread started waiting for ROS service calls...')
        t_last_replan = 0
        while not rospy.is_shutdown():
            ###############################
            #### TODO: Task 3 #############
            ###############################

            '''
            Implement the receding horizon planning thread
            Hint: Make sure you are familiar with the <Policy> class in utils/policy.py
            1. Determine if we need to replan by
                - checking if there is new data in the plan_state_buffer using 
                    <self.plan_state_buffer.new_data_available>
                - checking if the time since <t_last_replan> is larger than <self.replan_dt>
                - checking if <self.planner_ready> is True
            2. If we need to replan, 
                - Get the current state from the plan_state_buffer using <self.plan_state_buffer.readFromRT>
                - Get the previous policy from the policy_buffer using <self.policy_buffer.readFromRT>
                - Get the initial controls for hot start if there is a previous policy
                    you can use helper function <get_ref_controls> in the <Policy> class
                - Check if there is a new path in the path_buffer using <self.path_buffer.new_data_available>.
                    if true, Update the reference path in ILQR using <self.planner.update_ref_path(new path)>
                - Replan using ILQR 
            3. If the replan is successful,
                - Create a new <Policy> object using your new plan
                - Write the new policy to the policy buffer using <self.policy_buffer.writeFromNonRT>
                - Publish the new policy for RVIZ visualization
                    for example: self.trajectory_pub.publish(new_policy.to_msg())       
            '''
            ###############################
            #### END OF TODO #############
            ###############################
            time.sleep(0.01)
