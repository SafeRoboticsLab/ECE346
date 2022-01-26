#!/usr/bin/env python
import rospy
from IPython import display
import matplotlib.pyplot as plt
# TODO: import the message type for the subscribed topic

class PoseSubscriber:
    '''
    This class subscribes to the ros "/zed2/zed_node/pose" topic, and 
    save the most recent 50 position [x,y] in to lists

    TODO: Here you need to finish the rest of your subscriber class
    As we have done before, you may need to create some objects and 
    add some functions
    '''
    def __init__(self):
        self.x_traj = []
        self.y_traj = []
        
if __name__ == "__main__":
    listener = PoseSubscriber()

    # initalize the trajectory plot
    plt.ion()
    plt.show()
    plt.figure(figsize=(5, 5))

    # plotting the trajectory at 100HZ
    while not rospy.is_shutdown():
        display.clear_output(wait = True)
        display.display(plt.gcf())
        plt.clf()
        
        plt.scatter(listener.x_traj, listener.y_traj)
        plt.xlim((-5, 5))
        plt.ylim((-5, 5))
        plt.pause(0.001)
