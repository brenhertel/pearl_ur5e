#!/usr/bin/env python
import robotiq_gripper
import rospy
from pearl_ur5e.msg import gripper_pos
import std_msgs

ip = "192.168.50.3"

def talker():
    pub = rospy.Publisher('/gripper_data/position', gripper_pos, queue_size=100)
    print("Creating gripper...")
    gripper = robotiq_gripper.RobotiqGripper()
    print("Connecting to gripper...")
    gripper.connect(ip, 63352)
    print("Activating gripper...")
    gripper.activate()
    print("Gripper ready! Publishing...")
    while not rospy.is_shutdown():
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            msg = gripper_pos()
            msg.header = h
            msg.gripper_pos = int(100 * gripper.get_current_position() / 255)
            pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('gripper_pose_pub', anonymous=True)
    talker()
    
