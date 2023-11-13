#!/usr/bin/env python3

#all necessary imports
import rospy
import roslib
import message_filters
import tf
from geometry_msgs.msg import TransformStamped

import time
import h5py
import numpy as np

# create subscribers for tf_listener
#each subscriber writes data and timestamps to file (txt)
#if demo is chosen to be saved, read from the txt files
#compare timestamps
#if timestamps match, write to an array
#save array to h5 file
last_nsecs = 0
last_secs = 0


def tf_callback(tfmsg, tf_file):
	global last_nsecs
	global last_secs
	if tfmsg.header.stamp.secs != last_secs and tfmsg.header.stamp.nsecs != last_nsecs:
		tf_file.write(str(tfmsg.header.stamp.secs) + ', ' + str(tfmsg.header.stamp.nsecs) + ', ' + str(tfmsg.transform.translation.x) + ', ' + str(tfmsg.transform.translation.y) + ', ' + str(tfmsg.transform.translation.z) + ', ' + str(tfmsg.transform.rotation.x) + ', ' + str(tfmsg.transform.rotation.y) + ', ' + str(tfmsg.transform.rotation.z) + ', ' + str(tfmsg.transform.rotation.w) + '\n')
		last_secs = tfmsg.header.stamp.secs
		last_nsecs = tfmsg.header.stamp.nsecs
	

def getline_data(fp):
	return np.array([float(i) for i in fp.readline().split(', ')])
   
def save_demo():
	tf_fp = open('tf_data.txt', 'r')
	tf_time_arr = np.zeros((1, 2))
	tf_pos_arr = np.zeros((1, 3))
	tf_rot_arr = np.zeros((1, 4))
	
	
	try:
	    tf_data = getline_data(tf_fp)
	    while True:
	        tf_time = tf_data[0] + (tf_data[1] * 10.0**-9)
	            
	        tf_time_arr = np.vstack((tf_time_arr, tf_data[0:2]))
	        tf_pos_arr = np.vstack((tf_pos_arr, tf_data[2:5]))
	        tf_rot_arr = np.vstack((tf_rot_arr, tf_data[5:9]))
	            
	        tf_data = getline_data(tf_fp)
	except ValueError:
	    rospy.loginfo('Finished demo recording')
	    
	tf_fp.close()
	
	
	tf_time_arr = np.delete(tf_time_arr, 0, 0)
	tf_pos_arr = np.delete(tf_pos_arr, 0, 0)
	tf_rot_arr = np.delete(tf_rot_arr, 0, 0)
	
	
	if len(tf_time_arr) > 0:
		name = 'h5_files/recorded_demo ' + time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + '.h5'
		fp = h5py.File(name, 'w')
		
		dset_tt = fp.create_dataset('/transform_info/transform_time', data=tf_time_arr)
		dset_tp = fp.create_dataset('/transform_info/transform_positions', data=tf_pos_arr)
		dset_tr = fp.create_dataset('/transform_info/transform_orientations', data=tf_rot_arr)
		
		fp.close()
	else:
		rospy.loginfo('I Think Something went wrong...no data recorded.')
	
def end_record(tf_file):
    print(time.time())
    
    tf_file.close()
    
    save = input('Would you like to save this demo? (y/n)')
    rospy.loginfo("You entered: %s", save)  
    if (save == 'y'):
    	save_demo()

    cont = input('Would you like to start another demo? (y/n)')
    rospy.loginfo("You entered: %s", cont)
    if (cont == 'y'):
        demo_recorder()
   
def demo_recorder():
    try:
        #create tf data file
        tf_fp = open('tf_data.txt', 'w')
    	
        rospy.init_node('demo_recorder', anonymous=True)
        
        print('Press [Enter] to start recording')
        input()
        print(time.time())
    	
        #create subscribers to topics
    	
    		
        sub_tf = rospy.Subscriber("/tf_listener", TransformStamped, tf_callback, tf_fp)
			
    	
    		
        rospy.loginfo('Recording has started')
        rospy.loginfo('Press [ctrl]+C at any time to stop recording.')
    		
        rospy.spin()
        end_record(tf_fp)
        
    except rospy.ROSInterruptException:
        end_record(tf_fp)
    except KeyboardInterrupt:
        end_record(tf_fp)
        
if __name__ == '__main__':
	demo_recorder()


