#!/usr/bin/env python
import robotiq_gripper
import rospy
from pearl_ur5e.msg import gripper_pos
from std_msgs.msg import Int32

ip = "192.168.50.3"

def set_position(data, gripper):
    rospy.logwarn('data recieved ' + str(data.data))
    pos = int(255 * data.data / 100)
    gripper.move_and_wait_for_pos(pos, 255, 255)

def listener():
    print("Creating gripper...")
    gripper = robotiq_gripper.RobotiqGripper()
    print("Connecting to gripper...")
    gripper.connect(ip, 63352)
    print("Activating gripper...")
    gripper.activate()
    print("Gripper ready! Subscribing...")
    rospy.Subscriber('/gripper_sends/position', Int32, set_position, gripper)
    rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node('gripper_pose_sub', anonymous=True)
    listener()
    
