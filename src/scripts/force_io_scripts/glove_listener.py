#!/usr/bin/env python  
import numpy as np
import matplotlib.pyplot as plt
import h5py
import rospy

from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray

#arduino map function (linear interpolation)
def my_map(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

def constrain(x, in_min, in_max):
    if x < in_min:
        x = in_min
    if x > in_max:
        x = in_max
    return x

class GloveController(object):

    def __init__(self):
    
        rospy.init_node('glove_controller_node')  # Initialize ROS node
        self.cur_pos = 0

        self.pub = rospy.Publisher('/gripper_sends/position', Int32, queue_size=1)
        rospy.sleep(0.1)
        self.new_msg = Int32()
        self.new_msg.data = self.cur_pos
        self.pub.publish(self.new_msg)
        
        self.open_avg = None
        self.close_avg = None
        
        input('Press [Enter] to get starting position')
        self.get_open_pos()
        input('Press [Enter] to get closed position')
        self.get_close_pos()
        print('Starting position: ', self.open_avg)
        print('Closed position: ', self.close_avg)
        input('Press [Enter] to begin')
        
        try:
            rospy.Subscriber("fingersPos", Float32MultiArray, self.check_pos, queue_size=1)
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.logwarn('Shutting down')
            self.new_msg.data = 0
            self.pub.publish(self.new_msg)
        except KeyboardInterrupt:
            rospy.logwarn('Shutting down')
            self.new_msg.data = 0
            self.pub.publish(self.new_msg)
        
    def get_open_pos(self):
        open_pos = []
        for i in range(100):
            open_pos.append(rospy.wait_for_message("fingersPos", Float32MultiArray).data)
        self.open_avg = np.mean(open_pos, axis=0)
                
    def get_close_pos(self):
        close_pos = []
        for i in range(100):
            close_pos.append(rospy.wait_for_message("fingersPos", Float32MultiArray).data)
        self.close_avg = np.mean(close_pos, axis=0)
                
        
    def check_pos(self, msg):
    	map_to_gripper = my_map(msg.data, self.open_avg, self.close_avg, 0, 100)
    	gripper_pos_avg = np.mean(map_to_gripper)
    	gripper_pos_constrained = constrain(gripper_pos_avg, 0, 100)
    	self.new_msg.data = int(gripper_pos_constrained)
    	self.pub.publish(self.new_msg)
    	rospy.sleep(0.1)
        
if __name__ == '__main__':
    GC = GloveController()
