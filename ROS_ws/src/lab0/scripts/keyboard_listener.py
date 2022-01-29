!/usr/bin/env python
import rospy
# TODO: Import the correct message type for subscribing and publishing
# checkout the section 2.5.2 in lab0.pdf for example

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


import rospy
from rc_control_msgs.msg import RCControl
from geometry_msgs.msg import Twist

class ControlNode:
    def __init__(self):

        rospy.init_node("keyboard_control_listener", anonymous=True)

        self.pub = rospy.Publisher("/control/servo_control", RCControl, queue_size=50)
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.callback)
        rospy.spin()
        
    def callback(self, data):
        msg2pub = RCControl()
        msg2pub.header.stamp = rospy.get_rostime()
        msg2pub.throttle = data.linear.x
        msg2pub.steer = data.angular.z
        self.pub.publish(msg2pub)
    

if __name__ == "__main__":
    talker = ControlNode()

