#!/usr/bin/env python

import threading
import rospy
import numpy as np
from .utils import RealtimeBuffer, get_ros_param, State2D, GeneratePwm

from racecar_msgs.msg import ServoMsg 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class PurePursuitController():
    '''
    Main class for the controller
    '''

    def __init__(self):
        
        self.state_buffer = RealtimeBuffer()
        self.goal_buffer = RealtimeBuffer()
        self.pwm_converter = GeneratePwm()

        # Read Parameters from the parameter server
        self.read_parameters()

        # Setup the publisher and subscriber
        self.setup_publisher()
        self.setup_subscriber()

        # start planning thread
        # We use a thread to run the planning loop so that we can continuously read the odometry
        # and goal message in the callback function without being blocked by the planning loop
        threading.Thread(target=self.planning_thread).start()

    def read_parameters(self):
        '''
        This function reads the parameters from the parameter server
        '''
        # Read ROS topic names to subscribe 
        self.odom_topic = get_ros_param('~odom_topic', '/slam_pose')
        
        # Read ROS topic names to publish
        self.control_topic = get_ros_param('~control_topic', '/control/servo_control')
        
        # Read controller parameters
        self.throttle_gain = get_ros_param('~throttle_gain', 0.05)
        self.ld_max = get_ros_param('~ld_max', 2)
        
        # Read controller thresholds
        self.max_steer = get_ros_param('~max_steer', 0.35)
        self.max_vel = get_ros_param('~max_vel', 0.5)
        self.stop_distance = get_ros_param('~stop_distance', 0.5)
        self.wheel_base = get_ros_param('~wheel_base', 0.257)
        self.simulation = get_ros_param('~simulation', True)
        
    def setup_publisher(self):
        '''
        This function sets up the publisher for the trajectory
        '''
        ################## TODO: 1. Set up a publisher for the ServoMsg message###################
        # Create a publiser - self.control_pub:
        #   - subscribes to the topic <self.control_topic>
        #   - has message type <ServoMsg> (racecar_msgs.msg.Odometry) 
        #   - with queue size 1
        self.control_pub = None # TO BE FILLED
        ########################### END OF TODO 1#################################
        
            
    def setup_subscriber(self):
        '''
        This function sets up the subscriber for the odometry and path
        '''
        # This set up a subscriber for the goal you click on the rviz
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        ################## TODO: 2. Set up a subscriber for the odometry message###################
        # Create a subscriber:
        #   - subscribes to the topic <self.odom_topic>
        #   - has message type <Odometry> (nav_msgs.msg.Odometry) 
        #   - with callback function <self.odometry_callback>, which has already been implemented
        #   - with queue size 1
        ########################### END OF TODO 2#################################
        
    def odometry_callback(self, odom_msg: Odometry):
        """
        Subscriber callback function of the robot pose
        
        Parameters:
            odom_msg: Odometry message with type nav_msgs.msg.Odometry
        """
        # Retrieve state needed for planning from the odometry message
        # [x, y, v, w, delta]
        state_cur = State2D(odom_msg = odom_msg)

        # Add the current state to the buffer
        # Planning thread will read from the buffer
        self.state_buffer.writeFromNonRT(state_cur)

    def goal_callback(self, goal_msg: PoseStamped):
        """
        Subscriber callback function of the robot goal point
        
        Parameters:
            goal_msg: goal message with type geometry_msgs.msg.PoseStamped
        """
        ############## TODO: 3. Fill in the subscriber callback function ###################
        # 1. Inspect the data structure of the goal message <geometry_msgs.msg.PoseStamped>
        #   Hint: check the message data structure using the command
        #       rosmsg show geometry_msgs/PoseStamped
        # 2. Retrieve the goal from the goal message 
        #   and create a 3-dim numpy array [x,y,1]
        # 3. add the goal to the buffer (self.goal_buffer)
        
        goal_x = np.nan # TO BE FILLED
        goal_y = np.nan # TO BE FILLED
        
        ########################### END OF TODO 3 #################################
        rospy.loginfo(f"Received a new goal [{np.round(goal_x, 3)}, {np.round(goal_y,3)}]")
        
    def publish_control(self, accel, steer, state):
        
        # If we are in simulation,
        # the throttle and steering angle are acceleration and steering angle
        if self.simulation:
            throttle = accel
        else:
            # If we are in real truck,
            # the throttle and steering angle needs to convert to PWM signal
            throttle, steer = self.pwm_converter.convert(accel, steer, state)
            
        ########################## TODO: 4. Construct and publish a ROS message ###################
        # 1. Create an empty ServoMsg message
        #   Hint: check the message data structure using the command
        #       rosmsg show racecar_msgs/ServoMsg
        # 2. Set the header time to the current time
        # 3. Set the throttle and steering angle to the servo message
        # 4. Publish the servo message
        
        ########################### END OF TODO 4 #################################

    def planning_thread(self):
        rospy.loginfo("Planning thread started waiting for ROS service calls...")
        while not rospy.is_shutdown():
            # determine if we need to replan
            if self.state_buffer.new_data_available:
                state_cur = self.state_buffer.readFromRT()
                goal_cur = self.goal_buffer.readFromRT()
                # current longitudinal velocity
                vel_cur = state_cur.v_long 

                # check if a goal is available
                if goal_cur is not None:
                    # First, transform the goal to the robot frame
                    goal_robot = np.linalg.inv(state_cur.transformation_matrix()).dot(goal_cur)
                    # relative heading angle of the goal wrt the car
                    alpha = np.arctan2(goal_robot[1], goal_robot[0])
                    # relative distance between the car and the goal
                    dis2goal = np.sqrt(goal_robot[0]**2 + goal_robot[1]**2)

                
                    ########################## TODO: 5. Finish the pure pursuit controlle ###################
                    # 1. Check if the goal is close enough
                    #
                    # 2. if that is the case, stop the car by apply a negative acceleration (eg: -1 m/s^2)
                    #   and zero steering angle. Then, continue to the next iteration
                    #
                    # 3. If the target is behind the car, apply maximum steering angle 
                    #   and set the reference_velocity to vel_max
                    # 
                    # 4. Otherwise, 
                    #   - apply a pure pursuit controller for steering by assuming 
                    #       the reference path a straight line between the car and the goal
                    #   - Hint: look-ahead distance l_d is min(self.ld_max, dis2goal)
                    #   - set the reference_velocity to the minimum of vel_max and (dis2goal-self.stop_distance)
                    #   Detail Explanation: 
                    #   https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html
                    #
                    # 5. clip the steering angle
                    # 6. apply the simple proportional controller for the acceleration to track the reference_velocity
                    
                    accel = 0 # TO BE FILLED 
                    steer = 0 # TO BE FILLED
                    ########################### END OF TODO 5 ###########################################
                    
                    # publish the control
                    self.publish_control(accel, steer, state_cur)
