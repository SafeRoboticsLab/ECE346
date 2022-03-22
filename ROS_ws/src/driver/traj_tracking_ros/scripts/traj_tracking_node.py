#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from Tracking_Stanley import Tracking_Stanley
import sys, os


def main():
    rospy.init_node('traj_tracking_node')

    ## read parameters
    TrajTopic = rospy.get_param("~TrajTopic")
    PoseTopic = rospy.get_param("~PoseTopic")
    ControllerTopic = rospy.get_param("~ControllerTopic")
    ParamsFile = rospy.get_param("~TrackParamsFile")

    Tracking_Stanley(
            vicon_pose=False,
            pose_topic=PoseTopic,
            ref_traj_topic=TrajTopic,
            controller_topic=ControllerTopic,
            params_file=ParamsFile
        )
    rospy.spin()

if __name__ == '__main__':
    main()
