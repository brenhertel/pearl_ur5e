#!/usr/bin/env python3

#all necessary imports
import rospy
import roslib
import message_filters
import tf
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from tf.msg import tfMessage
from fetch_driver_msgs.msg import GripperState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

import time
import h5py
import numpy as np

# create subscribers for joint state, tf, wrench
#each subscriber writes data and timestamps to file (txt)
#if demo is chosen to be saved, read from the txt files
#compare timestamps
#if timestamps match, write to an array
#save array to h5 file

def gr_callback(grmsg, gr_file):
    gr_file.write(str(grmsg.header.stamp.secs) + ', ' + str(grmsg.header.stamp.nsecs) + ', ' + str(grmsg.joints[0].position) + '\n')

def imu_callback(imumsg, imu_file):
    imu_file.write(str(imumsg.header.stamp.secs) + ', ' + str(imumsg.header.stamp.nsecs) + ', ' + str(imumsg.orientation.x) + ', ' + str(imumsg.orientation.y) + ', ' + str(imumsg.orientation.z) + ', ' + str(imumsg.orientation.w) + ', ' + str(imumsg.angular_velocity.x) + ', ' + str(imumsg.angular_velocity.y)+ ', ' + str(imumsg.angular_velocity.z) + ', ' + str(imumsg.linear_acceleration.x)+ ', ' + str(imumsg.linear_acceleration.y) + ', ' + str(imumsg.linear_acceleration.z) + '\n')
	

def tf_callback(tfmsg, tf_file):	
    tf_file.write(str(tfmsg.header.stamp.secs) + ', ' + str(tfmsg.header.stamp.nsecs) + ', ' + str(tfmsg.transform.translation.x) + ', ' + str(tfmsg.transform.translation.y) + ', ' + str(tfmsg.transform.translation.z) + ', ' + str(tfmsg.transform.rotation.x) + ', ' + str(tfmsg.transform.rotation.y) + ', ' + str(tfmsg.transform.rotation.z) + ', ' + str(tfmsg.transform.rotation.w) + '\n')

def js_callback(jsmsg, js_file):
    if len(jsmsg.velocity) > 2:
        js_file.write(str(jsmsg.header.stamp.secs) + ', ' + str(jsmsg.header.stamp.nsecs).replace(')', '').replace('(', '') + ', ' + str(jsmsg.position).replace(')', '').replace('(', '') + ', ' + str(jsmsg.velocity).replace(')', '').replace('(', '') + ', ' + str(jsmsg.effort).replace(')', '').replace('(', '') + '\n')
 
def getline_data(fp):
    return np.array([float(i) for i in fp.readline().split(', ')])
   
def save_demo():
    js_fp = open('joint_data.txt', 'r')
    tf_fp = open('tf_data.txt', 'r')
    imu_fp = open('imu_data.txt', 'r')
    gr_fp = open('gripper_data.txt', 'r')
    js_time_arr = np.zeros((1, 2))
    js_pos_arr = np.zeros((1, 7))
    js_vel_arr = np.zeros((1, 7))
    js_eff_arr = np.zeros((1, 7))
    tf_time_arr = np.zeros((1, 2))
    tf_pos_arr = np.zeros((1, 3))
    tf_rot_arr = np.zeros((1, 4))
    imu_time_arr = np.zeros((1, 2))
    imu_rot_arr = np.zeros((1, 4))
    imu_lin_arr = np.zeros((1, 3))
    imu_ang_arr = np.zeros((1, 3))
    gr_time_arr = np.zeros((1, 2))
    gr_pos_arr = np.zeros((1, 1))

	
    try:
        js_data = getline_data(js_fp)
        tf_data = getline_data(tf_fp)
        imu_data = getline_data(imu_fp)
        gr_data = getline_data(gr_fp)
        while True:
            #print(js_data)
            #print(tf_data)
            #print(wr_data)
            js_time = js_data[0] + (js_data[1] * 10.0**-9)
            tf_time = tf_data[0] + (tf_data[1] * 10.0**-9)
            imu_time = imu_data[0] + (imu_data[1] * 10.0**-9)
            gr_time = gr_data[0] + (gr_data[1] * 10.0**-9)
            ctime_arr = [js_time, tf_time, imu_time, gr_time]
            print(ctime_arr)
            if max(ctime_arr) - min(ctime_arr) < 0.01:
                print('found collective record at t=' + str(js_time))
                #record
                #print(len(js_data[0:2]))
                js_time_arr = np.vstack((js_time_arr, js_data[0:2]))
                #print(len(js_data[2:8]))
                js_pos_arr = np.vstack((js_pos_arr, js_data[2:9]))
                #print(len(js_data[8:14]))
                js_vel_arr = np.vstack((js_vel_arr, js_data[9:16]))
                #print(len(js_data[14:20]))
                js_eff_arr = np.vstack((js_eff_arr, js_data[16:23]))
                
                tf_time_arr = np.vstack((tf_time_arr, tf_data[0:2]))
                tf_pos_arr = np.vstack((tf_pos_arr, tf_data[2:5]))
                tf_rot_arr = np.vstack((tf_rot_arr, tf_data[5:9]))
                
                imu_time_arr = np.vstack((imu_time_arr, imu_data[0:2]))
                imu_rot_arr = np.vstack((imu_rot_arr, imu_data[2:6]))
                imu_lin_arr = np.vstack((imu_lin_arr, imu_data[6:9]))
                imu_ang_arr = np.vstack((imu_ang_arr, imu_data[9:12]))
                
                gr_time_arr = np.vstack((gr_time_arr, gr_data[0:2]))
                gr_pos_arr = np.vstack((gr_pos_arr, gr_data[2]))
                
                #print('reading js')
                js_data = getline_data(js_fp)
                #print('reading tf')
                tf_data = getline_data(tf_fp)
                #print('reading wr')
                imu_data = getline_data(imu_fp)
                #print('reading gr')
                gr_data = getline_data(gr_fp)
            else:
                if min(ctime_arr) == js_time:
                    #print('reading js')
                    js_data = getline_data(js_fp)
                elif min(ctime_arr) == tf_time:
                    #print('reading tf')
                    tf_data = getline_data(tf_fp)
                elif min(ctime_arr) == imu_time:
                    #print('reading wr')
                    imu_data = getline_data(imu_fp)
                elif min(ctime_arr) == gr_time:
                    #print('reading gr')
                    gr_data = getline_data(gr_fp)
                else:
                    rospy.loginfo('Should never get here')
    except ValueError:
        rospy.loginfo('Finished demo recording')
        
    js_fp.close()
    tf_fp.close()
    imu_fp.close()
    gr_fp.close()
    
    #delete first row of 0's
    js_time_arr = np.delete(js_time_arr, 0, 0)
    js_pos_arr = np.delete(js_pos_arr, 0, 0)
    js_vel_arr = np.delete(js_vel_arr, 0, 0)
    js_eff_arr = np.delete(js_eff_arr, 0, 0)
    
    tf_time_arr = np.delete(tf_time_arr, 0, 0)
    tf_pos_arr = np.delete(tf_pos_arr, 0, 0)
    tf_rot_arr = np.delete(tf_rot_arr, 0, 0)
    
    imu_time_arr = np.delete(imu_time_arr, 0, 0)
    imu_rot_arr = np.delete(imu_rot_arr, 0, 0)
    imu_lin_arr = np.delete(imu_lin_arr, 0, 0)
    imu_ang_arr = np.delete(imu_ang_arr, 0, 0)
    
    gr_time_arr = np.delete(gr_time_arr, 0, 0)
    gr_pos_arr = np.delete(gr_pos_arr, 0, 0)
    
    print("Data Recorded: " + str(len(js_time_arr)))
    
    if len(js_time_arr) > 0:
    	name = 'h5_files/fetch_recorded_demo_' + str(int(js_time_arr[0][0])) + '.h5'
    	fp = h5py.File(name, 'w')
    	
    	dset_jt = fp.create_dataset('/joint_state_info/joint_time', data=js_time_arr)
    	dset_jp = fp.create_dataset('/joint_state_info/joint_positions', data=js_pos_arr)
    	dset_jv = fp.create_dataset('/joint_state_info/joint_velocities', data=js_vel_arr)
    	dset_je = fp.create_dataset('/joint_state_info/joint_effort', data=js_eff_arr)
    	
    	dset_tt = fp.create_dataset('/transform_info/transform_time', data=tf_time_arr)
    	dset_tp = fp.create_dataset('/transform_info/transform_positions', data=tf_pos_arr)
    	dset_tr = fp.create_dataset('/transform_info/transform_orientations', data=tf_rot_arr)
    	
    	dset_it = fp.create_dataset('/imu_info/imu_time', data=imu_time_arr)
    	dset_ir = fp.create_dataset('/imu_info/imu_orientation', data=imu_rot_arr)
    	dset_il = fp.create_dataset('/imu_info/imu_linear_acceleration', data=imu_lin_arr)
    	dset_ia = fp.create_dataset('/imu_info/imu_angular_acceleration', data=imu_ang_arr)
    	
    	dset_gt = fp.create_dataset('/gripper_info/gripper_time', data=gr_time_arr)
    	dset_gp = fp.create_dataset('/gripper_info/gripper_position', data=gr_pos_arr)
    	
    	fp.close()
    else:
    	print("No data recorded!")
    
def end_record(js_file, tf_file, imu_file, gr_file):
    print(time.time())
    
    js_file.close()
    tf_file.close()
    imu_file.close()
    gr_file.close()
    
    save = 'y' #input('Would you like to save this demo? (y/n)')
    rospy.loginfo("You entered: %s", save)  
    if (save == 'y'):
    	save_demo()
   
def demo_recorder():
    try:
        #create joint states file
        js_fp = open('joint_data.txt', 'w')
        #create tf data file
        tf_fp = open('tf_data.txt', 'w')
        #create wrench data file
        imu_fp = open('imu_data.txt', 'w')
        #create gripper data file
        gr_fp = open('gripper_data.txt', 'w')
    	
        rospy.init_node('demo_recorder', anonymous=True)
        
        #print('Press [Enter] to start recording')
        #input()
        print(time.time())
    	
        #create subscribers to topics
    	
        sub_js = rospy.Subscriber("/joint_states", JointState, js_callback, js_fp)
    		
        sub_tf = rospy.Subscriber("/tf_fixed", TransformStamped, tf_callback, tf_fp)
		
        sub_wr = rospy.Subscriber("/imu", Imu, imu_callback, imu_fp)
        
        sub_gr = rospy.Subscriber("/gripper_state", GripperState, gr_callback, gr_fp)
    	
    		
        rospy.loginfo('Recording has started')
    		
        rospy.spin()
        end_record(js_fp, tf_fp, imu_fp, gr_fp)
        
    except rospy.ROSInterruptException:
        end_record(js_fp, tf_fp, imu_fp, gr_fp)
    except KeyboardInterrupt:
        end_record(js_fp, tf_fp, imu_fp, gr_fp)
        
if __name__ == '__main__':
    demo_recorder()


