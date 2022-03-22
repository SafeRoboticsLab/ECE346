#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

class SimpleTrigger(object):
    def __init__(self):
        rospy.init_node("simple_trigger_broadcaster", anonymous=True)
        self.pub = rospy.Publisher("/simple_trigger", Bool, queue_size=1)
        self.run()
    
    def run(self):
        while not rospy.is_shutdown():
            data = input("Press enter to trigger")
            msg = Bool()
            msg.data = True
            self.pub.publish(msg)

if __name__=="__main__":
    trigger = SimpleTrigger()