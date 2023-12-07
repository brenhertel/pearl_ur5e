#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Bool
from pearl_ur5e.msg import gripper_pos

def talker():
    pub = rospy.Publisher('vibrate', Bool, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2) # 2hz
    while not rospy.is_shutdown():
        pub.publish(True)
        rate.sleep()
        pub.publish(False)
        rate.sleep()


class VibrateController(object):

    def __init__(self):
        self.threshold = 450
        self.pub = rospy.Publisher('vibrate', Bool, queue_size=10)
        self.sub = rospy.Subscriber('/gripper_sensors', gripper_pos, self.check_force, queue_size=1)
        rospy.spin()
        
    def check_force(self, msg):
        if msg.gripper_pos > self.threshold:
            self.pub.publish(True)
        else:
            self.pub.publish(False)

if __name__ == '__main__':
    rospy.init_node('vibrate_controller', anonymous=True)
    vc = VibrateController()
