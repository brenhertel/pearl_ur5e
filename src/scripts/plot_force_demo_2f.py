#!/usr/bin/env python

import numpy as np
import h5py
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from scipy.interpolate import UnivariateSpline

NUM_JOINTS = 6

def read_data(fname):
    hf = h5py.File(fname, 'r')
    print(list(hf.keys()))
    js = hf.get('joint_state_info')
    joint_time = np.array(js.get('joint_time'))
    joint_pos = np.array(js.get('joint_positions'))
    joint_vel = np.array(js.get('joint_velocities'))
    joint_eff = np.array(js.get('joint_effort'))
    joint_data = [joint_time, joint_pos, joint_vel, joint_eff]
    
    tf = hf.get('transform_info')
    tf_time = np.array(tf.get('transform_time'))
    tf_pos = np.array(tf.get('transform_positions'))
    tf_rot = np.array(tf.get('transform_orientations'))
    tf_data = [tf_time, tf_pos, tf_rot]
    print(tf_pos)
    
    wr = hf.get('wrench_info')
    wrench_time = np.array(wr.get('wrench_time'))
    wrench_frc = np.array(wr.get('wrench_force'))
    wrench_trq = np.array(wr.get('wrench_torque'))
    wrench_data = [wrench_time, wrench_frc, wrench_trq]
    
    gp = hf.get('gripper_info')
    gripper_time = np.array(gp.get('gripper_time'))
    gripper_pos = np.array(gp.get('gripper_vals'))
    force_time = np.array(gp.get('force_time'))
    forces = np.array(gp.get('forces'))
    gripper_data = [gripper_time, gripper_pos, force_time, forces]
    
    hf.close()
    
    return joint_data, tf_data, wrench_data, gripper_data
    
def read_data_rs(fname):
    NUM_RESAMPLE = 1000
    SMOOTH = 0.1

    hf = h5py.File(fname, 'r')
    print(list(hf.keys()))
        
    tf = hf.get('transform_info')
    tf_time = np.array(tf.get('transform_time'))
    tf_pos = np.array(tf.get('transform_positions'))
    tf_rot = np.array(tf.get('transform_orientations'))
    tf_data = [tf_time, tf_pos, tf_rot]
    tf_time = tf_time[:, 0] + tf_time[:, 1] * (10.0**-9)
    
    js = hf.get('joint_state_info')
    joint_time = np.array(js.get('joint_time'))
    joint_pos = np.array(js.get('joint_positions'))
    joint_vel = np.array(js.get('joint_velocities'))
    joint_eff = np.array(js.get('joint_effort'))
    joint_data = [joint_time, joint_pos, joint_vel, joint_eff]
    joint_time = joint_time[:, 0] + joint_time[:, 1] * (10.0**-9)

    
    wr = hf.get('wrench_info')
    wrench_time = np.array(wr.get('wrench_time'))
    wrench_frc = np.array(wr.get('wrench_force'))
    wrench_trq = np.array(wr.get('wrench_torque'))
    wrench_data = [wrench_time, wrench_frc, wrench_trq]
    wrench_time = wrench_time[:, 0] + wrench_time[:, 1] * (10.0**-9)
    
    gp = hf.get('gripper_info')
    gripper_time = np.array(gp.get('gripper_time'))
    gripper_pos = np.array(gp.get('gripper_vals'))
    force_time = np.array(gp.get('force_time'))
    forces = np.array(gp.get('forces'))
    gripper_data = [gripper_time, gripper_pos, force_time, forces]
    gripper_time = gripper_time[:, 0] + gripper_time[:, 1] * (10.0**-9)
    force_time = force_time[:, 0] + force_time[:, 1] * (10.0**-9)
    
    hf.close()
    
    times = [tf_time, joint_time, wrench_time, gripper_time, force_time]
    min_time = max([min(time) for time in times])
    max_time = min([max(time) for time in times])
    new_time = np.linspace(min_time, max_time, NUM_RESAMPLE)
    zero_time = new_time - min_time
    
    tf_data_rs = [zero_time]
    for data in [tf_pos, tf_rot]:
        tf_rs = np.zeros((NUM_RESAMPLE, np.shape(data)[1]))
        for i in range(np.shape(data)[1]):
            sptp = UnivariateSpline(tf_time, data[:, i])
            sptp.set_smoothing_factor(SMOOTH)
            tp = sptp(new_time)
            tf_rs[:, i] = tp
        tf_data_rs.append(tf_rs)
        
    joint_data_rs = [zero_time]
    for data in [joint_pos, joint_vel, joint_eff]:
        tf_rs = np.zeros((NUM_RESAMPLE, np.shape(data)[1]))
        for i in range(np.shape(data)[1]):
            sptp = UnivariateSpline(joint_time, data[:, i])
            sptp.set_smoothing_factor(SMOOTH)
            tp = sptp(new_time)
            tf_rs[:, i] = tp
        joint_data_rs.append(tf_rs)
    
    wrench_data_rs = [zero_time]
    for data in [wrench_frc, wrench_trq]:
        tf_rs = np.zeros((NUM_RESAMPLE, np.shape(data)[1]))
        for i in range(np.shape(data)[1]):
            sptp = UnivariateSpline(wrench_time, data[:, i])
            sptp.set_smoothing_factor(SMOOTH)
            tp = sptp(new_time)
            tf_rs[:, i] = tp
        wrench_data_rs.append(tf_rs)
    
    
    gpps = UnivariateSpline(gripper_time, gripper_pos)
    gpps.set_smoothing_factor(SMOOTH)
    gp = gpps(new_time)
    gpfr = UnivariateSpline(force_time, forces)
    gpfr.set_smoothing_factor(SMOOTH)
    gf = gpfr(new_time)
    gripper_data_rs = [zero_time, gp, zero_time, gf]
    
    return joint_data, tf_data, wrench_data, gripper_data, joint_data_rs, tf_data_rs, wrench_data_rs, gripper_data_rs

def display_data(joint_data, tf_data, wrench_data, gripper_data):
    print('joint_time: ' + str(np.shape(joint_data[0])))
    print('joint_positions: ' + str(np.shape(joint_data[1])))
    print('joint_velocities: ' + str(np.shape(joint_data[2])))
    print('joint_effort: ' + str(np.shape(joint_data[3])))
    
    print('transform_time: ' + str(np.shape(tf_data[0])))
    print('transform_positions: ' + str(np.shape(tf_data[1])))
    print('transform_orientations: ' + str(np.shape(tf_data[2])))
    
    print('wrench_time: ' + str(np.shape(wrench_data[0])))
    print('wrench_force: ' + str(np.shape(wrench_data[1])))
    print('wrench_torque: ' + str(np.shape(wrench_data[2])))
    
    print('gripper_time: ' + str(np.shape(gripper_data[0])))
    print('gripper_vals: ' + str(np.shape(gripper_data[1])))
    print('force_time: ' + str(np.shape(gripper_data[2])))
    print('force_vals: ' + str(np.shape(gripper_data[3])))
    return

def plot_joint_data(joint_data, resampled=False):
	js_fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
	js_fig.suptitle('Joints')
	if resampled:
	    time = joint_data[0]
	else:
	    time = joint_data[0][:, 0] + joint_data[0][:, 1] * (10.0**-9)
	for ax, data in [(ax1, joint_data[1]), (ax2, joint_data[2]), (ax3, joint_data[3])]:
		for i in range(NUM_JOINTS):
			ax.plot(time, data[:, i], label= 'joint' + str(i))
		ax.legend()
	ax3.set_xlabel('time')
	ax1.set_ylabel('positions (rad)')
	ax2.set_ylabel('velocities (rad/s)')
	ax3.set_ylabel('effort')
	
def plot_tf_data(tf_data, resampled=False):
	tf_fig, (ax1, ax2) = plt.subplots(2, 1)
	tf_fig.suptitle('tf')
	if resampled:
	    time = tf_data[0]
	else:
	    time = tf_data[0][:, 0] + tf_data[0][:, 1] * (10.0**-9)
	ax1.plot(time, tf_data[1][:, 0], label='x')
	ax1.plot(time, tf_data[1][:, 1], label='y')
	ax1.plot(time, tf_data[1][:, 2], label='z')
	ax2.plot(time, tf_data[2][:, 0], label='x')
	ax2.plot(time, tf_data[2][:, 1], label='y')
	ax2.plot(time, tf_data[2][:, 2], label='z')
	ax2.plot(time, tf_data[2][:, 3], label='w')
	
	ax1.legend()
	ax2.legend()
	
	ax2.set_xlabel('time')
	ax1.set_ylabel('position')
	ax2.set_ylabel('orientation')
	
	fig = plt.figure()
	fig.suptitle('Trajectory')
	ax = plt.axes(projection='3d')
	ax.plot3D(tf_data[1][:, 0], tf_data[1][:, 1], tf_data[1][:, 2], 'k')
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')
	
	
def plot_wrench_data(wrench_data, resampled=False):
	wr_fig, (ax1, ax2) = plt.subplots(2, 1)
	wr_fig.suptitle('Wrench')
	if resampled:
	    time = wrench_data[0]
	else:
	    time = wrench_data[0][:, 0] + wrench_data[0][:, 1] * (10.0**-9)
	
	ax1.plot(time, wrench_data[1][:, 0], label='x')
	ax1.plot(time, wrench_data[1][:, 1], label='y')
	ax1.plot(time, wrench_data[1][:, 2], label='z')
	ax2.plot(time, wrench_data[2][:, 0], label='x')
	ax2.plot(time, wrench_data[2][:, 1], label='y')
	ax2.plot(time, wrench_data[2][:, 2], label='z')
	
	ax1.legend()
	ax2.legend()
	
	ax2.set_xlabel('time')
	ax1.set_ylabel('force')
	ax2.set_ylabel('torque')
	
def plot_gripper_data(gripper_data, resampled=False):
	gp_fig, (ax1, ax2) = plt.subplots(2, 1)
	gp_fig.suptitle('Gripper')
	if resampled:
	    time = gripper_data[0]
	else:
	    time = gripper_data[0][:, 0] + gripper_data[0][:, 1] * (10.0**-9)
	
	ax1.plot(time, gripper_data[1][:], label="position")
	
	if resampled:
	    ftime = gripper_data[0]
	else:
	    ftime = gripper_data[2][:, 0] + gripper_data[2][:, 1] * (10.0**-9)
	ax2.plot(ftime, gripper_data[3][:], label="force")
	
	ax1.legend()
	ax2.legend()
	
	ax2.set_xlabel('time')
	ax1.set_ylabel('vals')
	ax2.set_ylabel('forces')
	
def plot_data(fname, resampled=False):
    if resampled:
        _, _, _, _, joint_data, tf_data, wrench_data, gripper_data = read_data_rs(fname)
    else:
        joint_data, tf_data, wrench_data, gripper_data = read_data(fname)
    display_data(joint_data, tf_data, wrench_data, gripper_data)
    plot_joint_data(joint_data, resampled)
    plot_tf_data(tf_data, resampled)
    plot_wrench_data(wrench_data, resampled)
    plot_gripper_data(gripper_data, resampled)
    #plt.show()
    return

def main():
    filename = '/home/pearl/catkin_ws/src/pearl_ur5e/src/scripts/h5_files/3-1 demos/recorded_demo 2024-03-01 15:42:38.h5'#raw_input('Enter the filename of the .h5 demo: ')
    plot_data(filename, True)
    plot_data(filename, False)
    plt.show()



if __name__ == '__main__':
  main()

