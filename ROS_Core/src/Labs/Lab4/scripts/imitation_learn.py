#!/usr/bin/env python

import threading
import rospy
import numpy as np
import os
from datetime import datetime

from utils import RealtimeBuffer, get_ros_param, RefPath, GeneratePwm
from neural_network import NeuralNetwork

from racecar_msgs.msg import ServoMsg
from racecar_learning.cfg import controllerConfig
from racecar_routing.srv import Plan, PlanResponse, PlanRequest
from visualization_msgs.msg import MarkerArray

from dynamic_reconfigure.server import Server
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg # used to display the trajectory on RVIZ
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, EmptyResponse
import queue
import pickle

class ImitationLearning():
    '''
    Main class for the Receding Horizon trajectory planner
    '''
    def __init__(self):
        # Indicate if the planner is used to generate a new trajectory
        self.update_lock = threading.Lock()
        self.learning_lock = threading.Lock()

        self.learning = None # None for idle, True for learning, False for testing
        self.state_action_buffer = queue.Queue()
        self.state_vec = []
        self.action_vec = []
        
        self.latency = 0.0
        self.v_ref = 0
        self.pwm_converter = GeneratePwm()
        
        # create buffers to handle multi-threading
        self.state_buffer = RealtimeBuffer()
        self.path_buffer = RealtimeBuffer()
        
        self.read_parameters()
        
        self.setup_publisher()
        
        self.setup_subscriber()

        self.setup_service()
        
        self.neural_network = NeuralNetwork(input_size = 4, output_size = 1,  
                                            lr=self.lr,
                                            save_dir = os.path.join(self.package_path, 'models'), 
                                            load_model=self.model_path,
                                            # load_model = os.path.join(self.package_path, 'models', 'model_example.pt')
                                        )

        # start planning and control thread
        threading.Thread(target=self.control_thread).start()
        threading.Thread(target=self.learning_thread).start()

    def read_parameters(self):
        '''
        This function reads the parameters from the parameter server
        '''
        # Required parameters
        self.package_path = rospy.get_param('~package_path')
                
        # Read ROS topic names to subscribe 
        self.odom_topic = get_ros_param('~odom_topic', '/slam_pose')
        self.path_topic = get_ros_param('~path_topic', '/Routing/Path')
        
        # Read ROS topic names to publish
        self.control_topic = get_ros_param('~control_topic', '/control/servo_control')
        
        # Read the simulation flag, 
        # if the flag is true, we are in simulation 
        # and no need to convert the throttle and steering angle to PWM
        self.simulation = get_ros_param('~simulation', True)
        self.lr = get_ros_param('~lr', 0.001)
        self.model_path = get_ros_param('~model_path', '')
        if self.model_path == '':
            self.model_path = None
            
            
    def setup_publisher(self):
        '''
        This function sets up the publisher for the trajectory
        '''
        # Publisher for the control command
        self.control_pub = rospy.Publisher(self.control_topic, ServoMsg, queue_size=1)
        
        self.path_pub = rospy.Publisher(self.path_topic, PathMsg, queue_size=10)

    def setup_subscriber(self):
        '''
        This function sets up the subscriber for the odometry and path
        '''
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=10)
        self.path_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.path_callback, queue_size=1)
        
    def path_callback(self, goal_msg):
        '''
        Plan a new trajectory when a new goal is received
        '''
        goal_x = goal_msg.pose.position.x
        goal_y = goal_msg.pose.position.y
        q = [goal_msg.pose.orientation.x, goal_msg.pose.orientation.y, 
                goal_msg.pose.orientation.z, goal_msg.pose.orientation.w]
        goal_psi = euler_from_quaternion(q)[-1]
        goal_pose = [goal_x, goal_y, goal_psi]
        
        request = PlanRequest(goal_pose, goal_pose)
        respond = self.get_ref_path(request)
        
        path = respond.path
        path.header = goal_msg.header
        self.path_pub.publish(path)
        
        # extract the path from the respond
        x = []
        y = []
        width_L = []
        width_R = []
        speed_limit = []
        
        for waypoint in path.poses:
            x.append(waypoint.pose.position.x)
            y.append(waypoint.pose.position.y)
            width_L.append(waypoint.pose.orientation.x)
            width_R.append(waypoint.pose.orientation.y)
            speed_limit.append(waypoint.pose.orientation.z)
                    
        centerline = np.array([x, y])
        
        try:
            ref_path = RefPath(centerline, width_L, width_R, speed_limit, loop=True)
            self.path_buffer.writeFromNonRT(ref_path)
            rospy.loginfo('Path received!')
        except:
            rospy.logwarn('Invalid path received! Move your robot and retry!')

    def setup_service(self):
        '''
        Set up ros service
        '''
        self.start_srv = rospy.Service('/learning/start_learn', Empty, self.start_learn_cb)
        self.stop_srv = rospy.Service('/learning/start_eval', Empty, self.start_eval_cb)
        self.stop_srv = rospy.Service('/learning/save_model', Empty, self.save_model_cb)
        self.stop_srv = rospy.Service('/learning/save_data', Empty, self.save_data_cb)
        
        self.dyn_server = Server(controllerConfig, self.reconfigure_callback)

        rospy.wait_for_service('/routing/plan')
        self.get_ref_path = rospy.ServiceProxy('/routing/plan', Plan)

    def save_model_cb(self, req):
        self.learning_lock.acquire()
        self.neural_network.save_model()
        self.learning_lock.release()
        return EmptyResponse()
    
    def save_data_cb(self, req):
        self.learning_lock.acquire()
        filename = "data_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".pkl"
        path = os.path.join(self.package_path, 'data', filename)
        with open(path, 'wb') as f:
            pickle.dump([self.state_vec, self.action_vec], f)
        print(f"Data saved to {path}")
        self.learning_lock.release()
        return EmptyResponse()
    
    def start_learn_cb(self, req):
        '''
        ros service callback function for start planning
        '''
        rospy.loginfo('Start learning!')
        self.learning = True
        self.state_buffer.reset()
        return EmptyResponse()

    def start_eval_cb(self, req):
        '''
        ros service callback function for stop planning
        '''
        rospy.loginfo('Start Evaluation!')
        self.learning = False
        return EmptyResponse()
    
    def reconfigure_callback(self, config, level):
        self.update_lock.acquire()
        self.latency = config['latency']
        rospy.loginfo(f"Latency Updated to {self.latency} s")
        if self.latency < 0.0:
            rospy.logwarn(f"Negative latency compensation {self.latency} is not a good idea!")
        self.v_ref = config['ref_speed']
        self.update_lock.release()
        return config
    
    def odometry_callback(self, odom_msg):
        '''
        Subscriber callback function of the robot pose
        '''
        if self.learning:
            self.get_state_action_pair(odom_msg)
        self.state_buffer.writeFromNonRT(odom_msg)
        
    def global_to_full_state(self, state_global):
        # state_global = [x, y, v, psi, timestamp]
        # full_state = [x, y, v, psi, progress, lateral, slope, curvature, d_heading, timestamp]
        # transform the state to the local frame
        ref_path = self.path_buffer.readFromRT()
        
        state_local = ref_path.global2local(state_global[:2]) # [progress, lateral, slope, curvature]
        
        d_heading = np.arctan2(np.sin(state_global[3] - state_local[2] ), np.cos(state_global[3] - state_local[2]))
        
        full_state = np.array([
            state_global[0], # global x
            state_global[1], # global y
            state_global[2], # longitudinal velocity
            state_global[3], # heading angle
            state_local[0], # progress
            state_local[1], # lateral displacement
            state_local[2], # slope
            state_local[3], # curvature
            d_heading, # relative heading
            state_global[-1] # timestamp
        ])
        
        return full_state
        
    def extract_state_odom(self, odom_msg):
        '''
        Extract the state from the full state [x, y, v, psi, timestamp]
        '''
        t_slam = odom_msg.header.stamp.to_sec()
        # get the state from the odometry message
        q = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, 
                odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
        # get the heading angle from the quaternion
        psi = euler_from_quaternion(q)[-1]
        
        state_global =np.array([
                            odom_msg.pose.pose.position.x,
                            odom_msg.pose.pose.position.y,
                            odom_msg.twist.twist.linear.x,
                            psi,
                            t_slam
            ])
        return state_global
        
    def get_control(self, state):
        if self.path_buffer.readFromRT() is None:
            return 0, 0
        
        full_state = self.global_to_full_state(state)
        
        # [x, y, v, psi, progress, lateral, slope, curvature, d_heading, timestamp]
        # we interested in local state [v, lateral error, curvature, heading error, ]
        local_state = np.array([full_state[2], full_state[5], full_state[-3], full_state[-2]])
        throttle = min((self.v_ref - full_state[2])*4, 5)
        steering = self.neural_network.inference_step(local_state)[0] * 0.37
        # print(local_state, steering)
        return throttle, steering
        
    def control_thread(self):
        '''
        Main control thread to publish control command
        '''
        rate = rospy.Rate(40)
        u_queue = queue.Queue()
        
        # values to keep track of the previous control command
        prev_state = None #[x, y, v, psi, time]
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
            x_new[-1] = u[-1]
            return x_new
        
        while not rospy.is_shutdown():
            
            if self.learning is not False:
                # not in evaluation mode
                rate.sleep()
                continue
            
            # initialize the control command
            accel = -5
            steer = 0
            state_cur = None            
            if self.simulation:
                t_act = rospy.get_rostime().to_sec()
            else:
                self.update_lock.acquire()
                t_act = rospy.get_rostime().to_sec() + self.latency 
                self.update_lock.release()
        
            # check if there is new state available
            if self.state_buffer.new_data_available:
                odom_msg = self.state_buffer.readFromRT()
                
                t_slam = odom_msg.header.stamp.to_sec()
                
                u = np.zeros(3)
                u[-1] = t_slam
                while not u_queue.empty() and u_queue.queue[0][-1] < t_slam:
                    u = u_queue.get() # remove old control commands
                state_cur = self.extract_state_odom(odom_msg)

                # predict the current state use past control command
                for i in range(u_queue.qsize()):
                    u_next = u_queue.queue[i]
                    dt = u_next[-1] - u[-1]
                    state_cur = dyn_step(state_cur, u, dt)
                    u = u_next
                    
                # predict the cur state with the most recent control command
                state_cur = dyn_step(state_cur, u, t_act - u[-1]) 
                
            # if there is no new state available, we do one step forward integration to predict the state
            elif prev_state is not None:
                t_prev = prev_u[-1]
                dt = t_act - t_prev
                # predict the state using the last control command is executed
                state_cur = dyn_step(prev_state, prev_u, dt)
            
            # compute the control command
            accel, steer = self.get_control(state_cur)

            # generate control command
            if not self.simulation:
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

    def get_state_action_pair(self, odom_msg):
        '''
        This function use two consecutive odometry messages to generate the state action pair
        '''
        prev_odom_msg = self.state_buffer.readFromRT()
        
        if prev_odom_msg is None:
            return 
        
        # No path available
        if self.path_buffer.readFromRT() is None:
            rospy.logwarn_once("No path available")
            return
        
        prev_state = self.extract_state_odom(prev_odom_msg)
        if prev_state[2]<0.1:
            return 
        cur_state = self.extract_state_odom(odom_msg)
        
        dt = cur_state[-1] - prev_state[-1]
        dpsi = cur_state[3] - prev_state[3]
        
        # dpsi = np.tan(delta)*(x[2]*dt/0.257)
        delta = np.arctan(dpsi/(prev_state[2]*dt/0.257))/0.37

        if abs(delta)>1:
            return
        
        full_state = self.global_to_full_state(prev_state)
        
        # [x, y, v, psi, progress, lateral, slope, curvature, d_heading, timestamp]
        # we interested in local state [v, lateral error, heading error, curvature]
        local_state = np.array([full_state[2], full_state[5], full_state[-3], full_state[-2]])
        self.state_action_buffer.put((local_state, delta))
        
    def learning_thread(self):
        
        itr = 0 
        while not rospy.is_shutdown():
            
            if not self.learning:
                rospy.sleep(0.1)
                continue
            
            self.learning_lock.acquire()
            while not self.state_action_buffer.empty():
                state, action = self.state_action_buffer.get()
                self.state_vec.append(state)
                self.action_vec.append(action)
            
            if len(self.action_vec)>64:
                loss = self.neural_network.train_step(np.array(self.state_vec), np.array(self.action_vec))
                rospy.loginfo(f"itr: {itr}, loss: {loss}")
                itr += 1
            
            self.learning_lock.release()
            
            
            rospy.sleep(0.1)