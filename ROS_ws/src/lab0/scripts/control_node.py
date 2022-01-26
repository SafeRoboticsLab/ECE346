#!/usr/bin/env python
import rospy
# TODO: Import the correct message for subscribing and publishing

class ControlNode:
    '''
    This class subscribe to "/cmd_vel" topics with type geometry_msgs/Twist
    from the keyboard control node.
    Then it translates the keyboard's message into a rc_control_msgs/RCControl message,
    and publish it with topic "/control/rc_control"
    '''
    def __init__(self):

        rospy.init_node("keyboard_control_listener", anonymous=True)

        '''
        TODO: Define your pubshiler and sublisher here
        '''

        rospy.spin()
        
    def callback(self, data):
        '''
        TODO: This is the callback function of you subscriber to the 
        keyboard control topic. 
        1. You will need to extract the 
            - linear.x as our throttle input
            - angular.z as our steering input
           from the subscribed message
        2. You need to construct a RCControl message and publish it
        '''
        pass

if __name__ == "__main__":
    talker = ControlNode()
