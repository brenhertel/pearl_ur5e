#!/usr/bin/env python

# Teleop Project, Soft Robotics Lab,WPI

import roslib
import rospy

import sys
import numpy as np
import random
import itertools
import csv
import os

import serial

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray

import actionlib
from control_msgs.msg import GripperCommandActionGoal

import matplotlib.pyplot as plt
# disable scientific notation
np.set_printoptions(suppress=True)

global glove_array
glove_array = []

# def onShutdown():
#     global glove_array
#     for i in range(0, len(glove_array)):
#         lbl = "array " + str(i)
#         plt.plot(glove_array[i], label=lbl)
#     plt.legend()
#     plt.show()
#     print("PLOTTED")


if __name__ == '__main__':

    rospy.init_node('glove_driver')
    # rospy.on_shutdown(onShutdown)

    pub_fingers = rospy.Publisher("fingersPos", Float32MultiArray, queue_size=1000)
    
    # for i in range(0, 16):
    #     glove_array.append([])

    rate = rospy.Rate(100)

    msg_fingers = Float32MultiArray()
    msg_gripper = GripperCommandActionGoal()

    grip = 1 #open grip
    counter = 0

    with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
        while not rospy.is_shutdown():
        
            line = ser.readline().decode('utf-8')  # data is encoded in UTF-8

            array = line.split(',', 30)[0:16]
            
            if len(array) >= 16:
                for i in range(0, len(array)):
                    array[i] = float(array[i][2:])
                    # glove_array[i].append(array[i])
                
                msg_fingers.data = [round(array[1], 2), round(array[5], 2), round(array[8], 2), round(array[11],2), round(array[14],2)]

                print(np.array(msg_fingers.data).round(3))
                # print(np.array(array).round(2))    
                pub_fingers.publish(msg_fingers)               


            rate.sleep()
