#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from controller import PurePursuitController


def main():
    rospy.init_node('lab0_node')
    rospy.loginfo("Start Lab 0 node")

    lab0_controller  = PurePursuitController()
    rospy.spin()


if __name__ == '__main__':
    main()
