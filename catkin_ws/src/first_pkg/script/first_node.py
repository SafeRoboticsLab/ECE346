#!/usr/bin/env python
import rospy

def main():
    rospy.init_node('first_node')
    rospy.loginfo("Start my first ros node")
    
    message = rospy.get_param('~message', 'Hello World')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo(message)
        rate.sleep()
        

if __name__ == '__main__':
    main()
