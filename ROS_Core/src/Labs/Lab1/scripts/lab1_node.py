#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from controller import PurePursuitController


def main():
    rospy.init_node('lab1_node')
    rospy.loginfo("Start Lab 1 node")

    lab1_controller  = PurePursuitController()
    rospy.spin()


if __name__ == '__main__':
    main()
