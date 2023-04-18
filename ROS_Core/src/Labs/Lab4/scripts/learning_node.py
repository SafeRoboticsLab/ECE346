#!/usr/bin/env python
import rospy
from imitation_learn import ImitationLearning

if __name__ == '__main__':
    # Safe guard for GPU memory
    rospy.init_node('imitation_learning_node')
    rospy.loginfo("Start imitation_learning node")

    ImitationLearning()
    rospy.spin()
