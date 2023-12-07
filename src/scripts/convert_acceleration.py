#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np

def get_Rz(theta):
    return np.array([[1,              0,             0], 
                     [0,  np.cos(theta), np.sin(theta)],
                     [0, -np.sin(theta), np.cos(theta)]])
                     
def get_Ry(theta):
    return np.array([[np.cos(theta), 0, -np.sin(theta)], 
                     [0,             1,              0],
                     [np.sin(theta), 0,  np.cos(theta)]])

def get_Rx(theta):
    return np.array([[ np.cos(theta), np.sin(theta), 0], 
                     [-np.sin(theta), np.cos(theta), 0],
                     [             0,             0, 1]])
                     
class IMU_Listener(object):

    def __init__(self):
        self.time_step = 0.01  # 0.01 second delay (Arduino)
        self.acceleration_values = np.array([0.0, 0.0, 0.0]).reshape((3, 1))
        self.velocities = [0.0, 0.0, 0.0]
        self.position_values = [0.0, 0.0, 0.0]
        self.gyro_values = [0.0, 0.0, 0.0]
        self.orientations = [0.0, 0.0, 0.0]
        try:
            self.fp = open("stationary.csv", 'w')
            rospy.Subscriber("MPU", String, self.save_callback)
            rospy.spin()
        except rospy.ROSInterruptException:
            self.fp.close()
        
    def save_callback(self, data):
        self.fp.write(data.data + '\n')
        #rospy.loginfo(data)
        values = data.data.split(',')
        for i in range(3, 6):
            self.gyro_values[i-3] = int(values[i]) * 2 * np.pi / (360 * 131.0)
            self.orientations[i-3] = self.gyro_values[i-3] * self.time_step + self.orientations[i-3]
        R = get_Rz(self.orientations[2]) @ get_Ry(self.orientations[1]) @ get_Rx(self.orientations[0])
        for i in range(3):
            self.acceleration_values[i] = (int(values[i]) / 16384.0)
        accels = R @ self.acceleration_values
        accels[2] = accels[2] + 1.0
        for i in range(3):
            self.velocities[i] = accels[i, 0] * self.time_step + self.velocities[i]
            self.position_values[i] = self.velocities[i] * self.time_step + self.position_values[i]
        print(self.orientations)
        print(self.acceleration_values)
        print(accels)
        rospy.loginfo(rospy.get_caller_id() + " X: " + str(round(self.position_values[0], 5)) + " Y: " + str(round(self.position_values[1], 5)) + " Z: " + str(round(self.position_values[2], 5)))

    def callback(self, data):
        #rospy.loginfo(data)
        values = data.data.split(',')
        for i in range(3, 6):
            self.gyro_values[i-3] = int(values[i]) * 2 * np.pi / (360 * 131.0)
            self.orientations[i-3] = self.gyro_values[i-3] * self.time_step + self.orientations[i-3]
        R = get_Rz(self.orientations[2]) @ get_Ry(self.orientations[1]) @ get_Rx(self.orientations[0])
        for i in range(3):
            self.acceleration_values[i] = (int(values[i]) / 16384.0)
        accels = R @ self.acceleration_values
        accels[2] = accels[2] + 1.0
        for i in range(3):
            self.velocities[i] = accels[i, 0] * self.time_step + self.velocities[i]
            self.position_values[i] = self.velocities[i] * self.time_step + self.position_values[i]
        print(self.orientations)
        print(self.acceleration_values)
        print(accels)
        rospy.loginfo(rospy.get_caller_id() + " X: " + str(round(self.position_values[0], 5)) + " Y: " + str(round(self.position_values[1], 5)) + " Z: " + str(round(self.position_values[2], 5)))

def listener():
    rospy.init_node('listener', anonymous=True)
    imu = IMU_Listener()

if __name__ == '__main__':
    listener()

