#!/usr/bin/env python

import threading
import rospy
import copy
import numpy as np
from .util import RealtimeBuffer, get_ros_param, GeneratePwm, extract_state_odom, RefPath
from visualization_msgs.msg import Marker
from racecar_msgs.msg import ServoMsg

from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg # used to display the trajectory on RVIZ
import queue

# ! IMPORTANT: You do not need to modify this file for the final project

class BossPlanner():
    '''
    Main class for the Receding Horizon trajectory planner
    '''
    def __init__(self, v_ref = 0.5):
        # Indicate if the planner is used to generate a new trajectory
        self.update_lock = threading.Lock()
        self.latency = 0.0
        self.v_ref = v_ref
        
        # set up buffer 
        self.pose_buffer = RealtimeBuffer()
        self.path_buffer = RealtimeBuffer()
        
        self.pwm_converter = GeneratePwm()

        self.read_parameters()
                
        self.setup_sub_pub_srv()
        
        # start planning and control thread
        threading.Thread(target=self.control_thread).start()

    def read_parameters(self):
        '''
        This function reads the parameters from the parameter server
        '''        
        # Read ROS topic names to subscribe 
        self.odom_topic = get_ros_param('~boss_odom_topic', '/Boss/Pose')
        
        # Read ROS topic names to publish
        self.control_topic = get_ros_param('~boss_control_topic', '/Boss/servo_control')
        
        # Read the simulation flag, 
        # if the flag is true, we are in simulation 
        # and no need to convert the throttle and steering angle to PWM
        self.simulation = get_ros_param('~simulation', True)
        
    def setup_sub_pub_srv(self):
        '''
        This function sets up the publisher for the trajectory
        '''
        # Publisher for the control command
        self.control_pub = rospy.Publisher(self.control_topic, ServoMsg, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odometry_callback, queue_size=10)

    def odometry_callback(self, odom_msg):
        '''
        Subscriber callback function of the robot pose
        '''
        # Add the current state to the buffer
        # Controller thread will read from the buffer
        # Then it will be processed and add to the planner buffer 
        # inside the controller thread
        self.pose_buffer.writeFromNonRT(odom_msg)
        
    def get_cur_pose(self) -> np.ndarray:
        '''
        Get the current pose from the buffer
        [x, y, v, psi, timestamp]
        '''
        return extract_state_odom(self.pose_buffer.readFromRT())

    def update_path(self, path):
        '''
        This function updates the path
        '''
        self.path_buffer.writeFromNonRT(path)
    
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
            if self.pose_buffer.new_data_available:
                state_cur = self.get_cur_pose()
                
                t_slam = state_cur[-1]
                
                u = np.zeros(3)
                u[-1] = t_slam
                while not u_queue.empty() and u_queue.queue[0][-1] < t_slam:
                    u = u_queue.get() # remove old control commands

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
            accel, steer = self.pure_pursuit(state_cur)

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

    def pure_pursuit(self, state):
        '''
        This function is responsible for generating the plan command
        '''
        if state is None:
            return 0, 0
        
        ref_path: RefPath
        ref_path = self.path_buffer.readFromRT()
        if ref_path is None:
            return 0, 0

        _, _, s = ref_path.get_closest_pts(state[:2])
        
        # calculate control
        # look ahead 1 second
        look_ahead, _ = ref_path.interp(s*ref_path.length + 0.5)
        look_ahead_x = look_ahead[0,0]
        look_ahead_y = look_ahead[1,0]
        
        # HACK: if the car is too close to the end of the path, stop
        if (1-s[0])*ref_path.length < 0.2:
            v_ref = 0
        else:
            v_ref = self.v_ref
        
        # apply pure pursuit
        alpha = np.arctan2(look_ahead_y - state[1], look_ahead_x - state[0]) - state[3]
        ld = np.sqrt((look_ahead_x - state[0])**2 + (look_ahead_y - state[1])**2)
        steer = np.arctan2(2*0.257*np.sin(alpha), ld)
        accel = (v_ref - state[2])*1
        if state[2]<0.5 and accel > 0: # add boost for low speed
            accel *= 2
        return accel, steer
    
    def visualize_car(self, ns, id, color, marker_array):    
        msg = self.pose_buffer.readFromRT()
        if msg is None:
            return    
        # Create the vehicle marker
        cubiod = Marker()
        cubiod.header = msg.header
        cubiod.ns = ns
        cubiod.id = id
        cubiod.type = 1 # CUBE
        cubiod.action = 0 # ADD/modify
        cubiod.scale.x = 0.42
        cubiod.scale.y = 0.19
        cubiod.scale.z = 0.188
        
        cubiod.pose = copy.deepcopy(msg.pose.pose)
        
        q = [cubiod.pose.orientation.x, cubiod.pose.orientation.y,
                cubiod.pose.orientation.z, cubiod.pose.orientation.w]
        
        yaw = euler_from_quaternion(q)[-1]
        cubiod.pose.position.x += 0.1285*np.cos(yaw)
        cubiod.pose.position.y += 0.1285*np.sin(yaw)
        cubiod.pose.position.z = 0 
        
        # ORANGE
        cubiod.color.r = color[0]
        cubiod.color.g = color[1]
        cubiod.color.b = color[2]
        cubiod.color.a = 0.5
        cubiod.lifetime = rospy.Duration(0)
        marker_array.markers.append(cubiod)
        
        # Create the arrow marker
        arrow = Marker()
        arrow.header = msg.header
        arrow.ns =  ns+"arrow"
        arrow.id = id
        arrow.type = 0 # ARROW
        arrow.action = 0 # ADD/modify
        arrow.pose = copy.deepcopy(msg.pose.pose)
        
        arrow.scale.x = 0.3
        arrow.scale.y = 0.02
        arrow.scale.z = 0.02
        
        arrow.color.r = 255/255.0
        arrow.color.g = 255/255.0
        arrow.color.b = 255/255.0
        arrow.color.a = 1.0
        
        arrow.lifetime = rospy.Duration(0)
        marker_array.markers.append(arrow)
