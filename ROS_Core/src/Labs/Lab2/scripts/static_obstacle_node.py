#!/usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from tagslam_ros.msg import AprilTagDetectionArray
from lab2_utils import get_ros_param, pose2T
import message_filters
from tf.transformations import quaternion_from_matrix

class StaticObstacle():
    def __init__(self):
        # Read ROS topic names to subscribe 
        self.odom_topic = get_ros_param('~odom_topic', '/slam_pose')
        self.static_obs_tag_topic = get_ros_param('~static_tag_topic', '/static_tag')
        self.static_obs_size = get_ros_param('~static_obs_size', 0.2)
        self.static_obs_topic = get_ros_param('~static_obs_topic', '/Obstacles/Static')

        self.T_rob2cam = np.array([[1.0, 0.0, 0.0, -0.357], # camera center offset
                                    [0.0, 1.0, 0.0, -0.06],
                                    [0.0, 0.0, 1.0, 0], #wheelbase
                                    [0.0, 0.0, 0.0, 1.0]]) # camera to rear axis transform
        self.T_cam2rob = np.linalg.inv(self.T_base2cam)

        self.T_obs2tag = np.array([[1.0, 0.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0, -self.obstacle_size/2.0],
                                    [0.0, 0.0, 0.0, 1.0]])  
        
        self.static_obs_publisher = rospy.Publisher(self.static_obs_topic, MarkerArray, queue_size=1)
        
        pose_sub = message_filters.Subscriber(self.odom_topic, Odometry)
        # This subscribe to the 2D Nav Goal in RVIZ
        tag_sub = message_filters.Subscriber(self.static_obs_tag_topic, AprilTagDetectionArray)
        self.static_obs_detection = message_filters.ApproximateTimeSynchronizer([pose_sub, tag_sub], 10, 0.1)
        self.static_obs_detection.registerCallback(self.detect_obs)

    def detect_obs(self, odom_msg, tag_list):
        '''
        This function detects the static obstacle
        '''
        detected_tag = []
        static_obs_msg = MarkerArray()
        # Get the pose of the robot
        T_rob2world = pose2T(odom_msg.pose.pose)
        T_cam2world = T_rob2world@self.T_cam2rob
        for tag in tag_list.detections:
            if tag.id in detected_tag:
                continue
            else:
                detected_tag.append(tag.id)

            T_tag2cam = pose2T(tag.pose)
            T_tag2world = T_cam2world@T_tag2cam
            T_obs2world = T_tag2world@self.T_obs2tag
            
            marker = Marker()
            marker.header = odom_msg.header
            marker.ns = 'static_obs'
            marker.id = tag.id
            marker.type = 1 # cube
            marker.action = 0 # add
            marker.pose.position.x = T_obs2world[0,3]
            marker.pose.position.y = T_obs2world[1,3]
            marker.pose.position.z = T_obs2world[2,3]
            
            q = quaternion_from_matrix(T_obs2world)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            
            marker.scale.x = self.static_obs_size
            marker.scale.y = self.static_obs_size
            marker.scale.z = self.static_obs_size
            
            marker.color.r = 0
            marker.color.g = 0
            marker.color.b = 153/255.0
            marker.color.a = 0.8
            
            static_obs_msg.markers.append(marker)
        
        self.static_obs_publisher.publish(static_obs_msg)

if __name__ == '__main__':
    rospy.init_node('static_obstacle_node')
    rospy.loginfo("Start static obstacle node")
    static_obstacle = StaticObstacle()
    rospy.spin()
