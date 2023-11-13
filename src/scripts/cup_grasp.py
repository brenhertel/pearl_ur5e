#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from dexterous_picking.srv import gripper_move_motors
from std_msgs.msg import Float64MultiArray
'''
def callback(data):
    rospy.loginfo("recieved: " + str(data.data))
    if any(element == 0 for element in data.data):
        rospy.loginfo("Making a service call.")
        call_gripper_service()
        
        
        
def call_gripper_service():
       
    move_fingers = '/gripper/move_motors'
    rospy.wait_for_service(move_fingers, timeout = 10)
    try:
        gripper_service = 
        
        gripper_service([0.0, 0.0, 0.0, 0.0])
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)    
'''

class Gripper_Caller(object):
    
    def __init__(self):
        rospy.wait_for_service('/gripper/move_motors', timeout = 10)
        self.gripper_service = rospy.ServiceProxy('/gripper/move_motors', gripper_move_motors)
        rospy.Subscriber("/pressure_sensor_readings", Float64MultiArray, self.callback, queue_size=1)
        self.top_finger_position = 0.0
        self.left_finger_position = 0.0
        self.right_finger_position = 0.0
        self.gripper_service([0.0, self.left_finger_position, self.right_finger_position, self.top_finger_position])
        
    def callback(self, data):
        rospy.loginfo("recieved: " + str(data.data))
        fingers = np.array(data.data)
        if np.any(fingers < 25):
            rospy.loginfo("Making a service call.")
            self.call_gripper_service(fingers)
            
    def call_gripper_service(self, fingers):
        
        try:
            if fingers[0] <= 25:
                self.top_finger_position = self.top_finger_position + 0.01
            if fingers[1] <= 25:
                self.left_finger_position = self.left_finger_position + 0.01
            if fingers[2] <= 25:
                self.right_finger_position = self.right_finger_position + 0.01
            
            self.gripper_service([0.0, self.left_finger_position, self.right_finger_position, self.top_finger_position])
                
            
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)  

 
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('array_subscriber', anonymous=True)

    gc = Gripper_Caller()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
