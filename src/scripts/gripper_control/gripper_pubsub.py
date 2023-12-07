#!/usr/bin/env python
import robotiq_gripper
import rospy
from pearl_ur5e.msg import gripper_pos
from std_msgs.msg import Int32
import std_msgs

ip = "192.168.50.3"

class PubSub(object):

    def __init__(self):
        rospy.logwarn("Creating gripper...")
        self.gripper = robotiq_gripper.RobotiqGripper()
        rospy.logwarn("Connecting to gripper on ip " + ip + " ...")
        self.gripper.connect(ip, 63352)
        rospy.logwarn("Activating gripper...")
        self.gripper.activate()
        rospy.logwarn("Gripper ready! Publishing and Subscribing...")
        self.listener()
        self.talker()
        rospy.spin()
        
    def listener(self):
        rospy.Subscriber('/gripper_sends/position', Int32, self.set_position)
        
    def set_position(self, data):
        rospy.logwarn('data recieved ' + str(data.data))
        pos = int(255 * data.data / 100)
        self.gripper.move_and_wait_for_pos(pos, 255, 255)
        
    def talker(self):
        pub = rospy.Publisher('/gripper_data/position', gripper_pos, queue_size=100)
        while not rospy.is_shutdown():
                h = std_msgs.msg.Header()
                h.stamp = rospy.Time.now()
                msg = gripper_pos()
                msg.header = h
                msg.gripper_pos = int(100 * self.gripper.get_current_position() / 255)
                pub.publish(msg)
                rospy.sleep(0.001)
    
if __name__ == '__main__':
    rospy.init_node('gripper_pose_pubsub', anonymous=True)
    pb = PubSub()
