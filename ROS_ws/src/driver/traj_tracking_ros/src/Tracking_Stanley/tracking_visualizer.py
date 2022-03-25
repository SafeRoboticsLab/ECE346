#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from traj_msgs.msg import TrajMsg
from std_msgs.msg import Bool

from IPython import display
from threading import Lock
import matplotlib.pyplot as plt
from Tracking_Stanley.tracking_stanley import Trajectory
from scipy.spatial.transform import Rotation
import math
import numpy as np


class TrackingVisualizeNode:

    def __init__(self):
        self.x_traj = []
        self.y_traj = []
        self.ref_pos = [0.0, 0.0]
        self.directional_arrow = [0.0, 0.0]  # x, y
        self.start_listening()
        self.course = None
        self.traj_received = False
        self.triggered = True
        self.t_0 = None

        # TODO: Read this from yaml
        self.d_f = 0.257

        self.lock = Lock()

    def start_listening(self):
        rospy.init_node("tracking_visualize_node", anonymous=True)
        rospy.Subscriber("/zed2/zed_node/odom", Odometry, self.callback_odom)
        rospy.Subscriber("/planning/trajectory", TrajMsg,
                         self.callback_trajectory)

    def callback_trajectory(self, data):
        self.lock.acquire()
        self.course = Trajectory(msg=data)
        self.traj_received = True
        self.lock.release()

    def callback_odom(self, data):
        cur_t = data.header.stamp
        if self.traj_received:
            self.ref_pos = self.course.get_pos_ref(cur_t)
        # postion
        x_value = data.pose.pose.position.x
        y_value = data.pose.pose.position.y

        # pose
        r = Rotation.from_quat([
            data.pose.pose.orientation.x, data.pose.pose.orientation.y,
            data.pose.pose.orientation.z, data.pose.pose.orientation.w
        ])

        rot_vec = r.as_rotvec()
        current_yaw = rot_vec[2]

        x_f = x_value + np.cos(current_yaw) * self.d_f
        y_f = y_value + np.sin(current_yaw) * self.d_f

        endx = x_value + math.cos(current_yaw)
        endy = y_value + math.sin(current_yaw)

        self.lock.acquire()
        self.x_traj.append(x_f)
        self.y_traj.append(y_f)
        self.directional_arrow = [endx, endy]

        while len(self.x_traj) > 10:
            self.x_traj.pop(0)
            self.y_traj.pop(0)

        self.lock.release()


if __name__ == "__main__":
    listener = TrackingVisualizeNode()
    plt.ion()
    plt.show()
    plt.figure()

    while not rospy.is_shutdown():
        display.clear_output(wait=True)
        display.display(plt.gcf())
        plt.clf()
        listener.lock.acquire()
        if listener.course is not None:
            plt.plot(listener.course.data[0],
                     listener.course.data[1],
                     "-r",
                     label="course",
                     linewidth=1.0)
            try:
                plt.scatter(listener.ref_pos[0],
                            listener.ref_pos[1],
                            color="k",
                            zorder=10,
                            alpha=0.6)
                plt.plot(listener.x_traj,
                         listener.y_traj,
                         linewidth=2.0,
                         alpha=0.6)
                # plt.scatter(listener.x_traj[-1],
                #             listener.y_traj[-1],
                #             color="k",
                #             zorder=10,
                #             alpha=0.6)
                plt.plot([listener.x_traj[-1], listener.directional_arrow[0]],
                         [listener.y_traj[-1], listener.directional_arrow[1]],
                         "g",
                         alpha=0.2,
                         linewidth=2.0)
            except IndexError:
                print("Index error in tracking_visualizer")
        listener.lock.release()
        plt.xlim((-3, 4))
        plt.ylim((-1, 6))
        plt.pause(0.001)
