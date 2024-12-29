#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from traj_planner import TrajectoryPlanner

if __name__ == '__main__':
    rospy.init_node('traj_planning_node')
    rospy.loginfo("Start trajectory planning node")

    planner = TrajectoryPlanner()
    rospy.spin()
