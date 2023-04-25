#!/usr/bin/env python
import rospy
import sys, os
from task2_world.swift_haul import SwiftHaul

# ! IMPORTANT: You do not need to modify this file for the final project

if __name__ == '__main__':
    # Safe guard for GPU memory
    rospy.init_node('ece346_final_task2_node')
    SwiftHaul()
    rospy.spin()
