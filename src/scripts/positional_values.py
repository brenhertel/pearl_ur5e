#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import numpy as np
from scipy.signal import butter


class IMU_Listener(object):

    def __init__(self):
        self.time_step = 0.01  # 0.01 second delay (Arduino)
        self.acceleration_values = np.array([0.0, 0.0, 0.0])
        self.velocities = [0.0, 0.0, 0.0]
        self.position_values = [0.0, 0.0, 0.0]
        self.gyro_values = [0.0, 0.0, 0.0]
        rospy.Subscriber("MPU", String, self.callback)
        

    def callback(self, data):
        values = data.data.split(',')
        self.acceleration_values = [float(values[i]) for i in range(3)]
        self.gyro_values = [float(values[i]) for i in range(3, 6)]
        
        rospy.loginfo(rospy.get_caller_id() + " Accel: " + str(self.acceleration_values) + " Gyro: " + str(self.gyro_values))

def listener():
    rospy.init_node('listener', anonymous=True)
    imu = IMU_Listener()
    rospy.spin()

if __name__ == '__main__':
    listener()


def detect_stationary_periods(accX, accY, accZ, samplePeriod):
    # Compute accelerometer magnitude
    acc_mag = np.sqrt(accX**2 + accY**2 + accZ**2)

    # HP filter accelerometer data
    filtCutOff_hp = 0.001
    b_hp, a_hp = butter(1, (2*filtCutOff_hp) / (1/samplePeriod), 'high')
    acc_magFilt_hp = filtfilt(b_hp, a_hp, acc_mag)

    # Compute absolute value
    acc_magFilt_hp = np.abs(acc_magFilt_hp)

    # LP filter accelerometer data
    filtCutOff_lp = 5
    b_lp, a_lp = butter(1, (2*filtCutOff_lp) / (1/samplePeriod), 'low')
    acc_magFilt_lp = filtfilt(b_lp, a_lp, acc_magFilt_hp)

    # Threshold detection
    stationary = acc_magFilt_lp < 0.05
    return stationary

