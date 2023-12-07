#!/usr/bin/env python

import numpy as np
import h5py
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from downsampling import *

NUM_JOINTS = 6

def read_data(fname):
    hf = h5py.File(fname, 'r')
    print(list(hf.keys()))
        
    tf = hf.get('transform_info')
    tf_time = np.array(tf.get('transform_time'))
    tf_pos = np.array(tf.get('transform_positions'))
    tf_rot = np.array(tf.get('transform_orientations'))
    tf_data = [tf_time, tf_pos, tf_rot]
    tf_pos_ds, inds = DouglasPeuckerPoints2(tf_pos, 1000)
    tf_data_ds = [tf_time[inds, :], tf_pos[inds, :], tf_rot[inds, :]]
    print(tf_pos)
    
    js = hf.get('joint_state_info')
    joint_time = np.array(js.get('joint_time'))
    joint_pos = np.array(js.get('joint_positions'))
    joint_vel = np.array(js.get('joint_velocities'))
    joint_eff = np.array(js.get('joint_effort'))
    joint_data = [joint_time, joint_pos, joint_vel, joint_eff]
    joint_data_ds = [joint_time[inds, :], joint_pos[inds, :], joint_vel[inds, :], joint_eff[inds, :]]

    
    wr = hf.get('wrench_info')
    wrench_time = np.array(wr.get('wrench_time'))
    wrench_frc = np.array(wr.get('wrench_force'))
    wrench_trq = np.array(wr.get('wrench_torque'))
    wrench_data = [wrench_time, wrench_frc, wrench_trq]
    wrench_data_ds = [wrench_time[inds, :], wrench_frc[inds, :], wrench_trq[inds, :]]
    
    gp = hf.get('gripper_info')
    gripper_time = np.array(gp.get('gripper_time'))
    gripper_pos = np.array(gp.get('gripper_vals'))
    force_time = np.array(gp.get('force_time'))
    forces = np.array(gp.get('forces'))
    gripper_data = [gripper_time, gripper_pos, force_time, forces]
    gripper_data_ds = [gripper_time[inds, :], gripper_pos[inds, :], force_time[inds, :], forces[inds, :]]
    
    hf.close()
    
    return joint_data, tf_data, wrench_data, gripper_data, joint_data_ds, tf_data_ds, wrench_data_ds, gripper_data_ds

def display_data(joint_data, tf_data, wrench_data, gripper_data, joint_data_ds, tf_data_ds, wrench_data_ds, gripper_data_ds):
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
    
    print('joint_time_ds: ' + str(np.shape(joint_data_ds[0])))
    print('joint_positions_ds: ' + str(np.shape(joint_data_ds[1])))
    print('joint_velocities_ds: ' + str(np.shape(joint_data_ds[2])))
    print('joint_effort_ds: ' + str(np.shape(joint_data_ds[3])))
    
    print('transform_time_ds: ' + str(np.shape(tf_data_ds[0])))
    print('transform_positions_ds: ' + str(np.shape(tf_data_ds[1])))
    print('transform_orientations_ds: ' + str(np.shape(tf_data_ds[2])))
    
    print('wrench_time_ds: ' + str(np.shape(wrench_data_ds[0])))
    print('wrench_force_ds: ' + str(np.shape(wrench_data_ds[1])))
    print('wrench_torque_ds: ' + str(np.shape(wrench_data_ds[2])))
    
    print('gripper_time_ds: ' + str(np.shape(gripper_data_ds[0])))
    print('gripper_vals_ds: ' + str(np.shape(gripper_data_ds[1])))
    print('force_time_ds: ' + str(np.shape(gripper_data_ds[2])))
    print('force_vals_ds: ' + str(np.shape(gripper_data_ds[3])))
    return

def plot_joint_data(joint_data, joint_data_ds):
	js_fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
	js_fig.suptitle('Joints')
	time = joint_data[0][:, 0] + joint_data[0][:, 1] * (10.0**-9)
	time_ds = joint_data_ds[0][:, 0] + joint_data_ds[0][:, 1] * (10.0**-9)
	for ax, data in [(ax1, joint_data[1]), (ax2, joint_data[2]), (ax3, joint_data[3])]:
		for i in range(NUM_JOINTS):
			ax.plot(time, data[:, i], label= 'joint' + str(i))
	for ax, data in [(ax1, joint_data_ds[1]), (ax2, joint_data_ds[2]), (ax3, joint_data_ds[3])]:
		for i in range(NUM_JOINTS):
			ax.plot(time_ds, data[:, i], '--', label= 'DS joint' + str(i))
		ax.legend()
	ax3.set_xlabel('time')
	ax1.set_ylabel('positions (rad)')
	ax2.set_ylabel('velocities (rad/s)')
	ax3.set_ylabel('effort')
	
def plot_tf_data(tf_data, tf_data_ds):
	tf_fig, (ax1, ax2) = plt.subplots(2, 1)
	tf_fig.suptitle('tf')
	time = tf_data[0][:, 0] + tf_data[0][:, 1] * (10.0**-9)
	time_ds = tf_data_ds[0][:, 0] + tf_data_ds[0][:, 1] * (10.0**-9)
	ax1.plot(time, tf_data[1][:, 0], label='x')
	ax1.plot(time, tf_data[1][:, 1], label='y')
	ax1.plot(time, tf_data[1][:, 2], label='z')
	ax2.plot(time, tf_data[2][:, 0], label='x')
	ax2.plot(time, tf_data[2][:, 1], label='y')
	ax2.plot(time, tf_data[2][:, 2], label='z')
	ax2.plot(time, tf_data[2][:, 3], label='w')
	
	ax1.plot(time_ds, tf_data_ds[1][:, 0], '--', label='DS x')
	ax1.plot(time_ds, tf_data_ds[1][:, 1], '--', label='DS y')
	ax1.plot(time_ds, tf_data_ds[1][:, 2], '--', label='DS z')
	ax2.plot(time_ds, tf_data_ds[2][:, 0], '--', label='DS x')
	ax2.plot(time_ds, tf_data_ds[2][:, 1], '--', label='DS y')
	ax2.plot(time_ds, tf_data_ds[2][:, 2], '--', label='DS z')
	ax2.plot(time_ds, tf_data_ds[2][:, 3], '--', label='DS w')
	
	ax1.legend()
	ax2.legend()
	
	ax2.set_xlabel('time')
	ax1.set_ylabel('position')
	ax2.set_ylabel('orientation')
	
	fig = plt.figure()
	fig.suptitle('Trajectory')
	ax = plt.axes(projection='3d')
	ax.plot3D(tf_data[1][:, 0], tf_data[1][:, 1], tf_data[1][:, 2], 'k')
	ax.plot3D(tf_data_ds[1][:, 0], tf_data_ds[1][:, 1], tf_data_ds[1][:, 2], 'r--')
	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')
	
	
def plot_wrench_data(wrench_data, wrench_data_ds):
	wr_fig, (ax1, ax2) = plt.subplots(2, 1)
	wr_fig.suptitle('Wrench')
	time = wrench_data[0][:, 0] + wrench_data[0][:, 1] * (10.0**-9)
	time_ds = wrench_data_ds[0][:, 0] + wrench_data_ds[0][:, 1] * (10.0**-9)
	
	ax1.plot(time, wrench_data[1][:, 0], label='x')
	ax1.plot(time, wrench_data[1][:, 1], label='y')
	ax1.plot(time, wrench_data[1][:, 2], label='z')
	ax2.plot(time, wrench_data[2][:, 0], label='x')
	ax2.plot(time, wrench_data[2][:, 1], label='y')
	ax2.plot(time, wrench_data[2][:, 2], label='z')
	
	ax1.plot(time_ds, wrench_data_ds[1][:, 0], '--', label='DS x')
	ax1.plot(time_ds, wrench_data_ds[1][:, 1], '--', label='DS y')
	ax1.plot(time_ds, wrench_data_ds[1][:, 2], '--', label='DS z')
	ax2.plot(time_ds, wrench_data_ds[2][:, 0], '--', label='DS x')
	ax2.plot(time_ds, wrench_data_ds[2][:, 1], '--', label='DS y')
	ax2.plot(time_ds, wrench_data_ds[2][:, 2], '--', label='DS z')
	
	ax1.legend()
	ax2.legend()
	
	ax2.set_xlabel('time')
	ax1.set_ylabel('force')
	ax2.set_ylabel('torque')
	
def plot_gripper_data(gripper_data, gripper_data_ds):
	gp_fig, (ax1, ax2) = plt.subplots(2, 1)
	gp_fig.suptitle('Gripper')
	time = gripper_data[0][:, 0] + gripper_data[0][:, 1] * (10.0**-9)
	time_ds = gripper_data_ds[0][:, 0] + gripper_data_ds[0][:, 1] * (10.0**-9)
	
	ax1.plot(time, gripper_data[1][:], label="position")
	ax1.plot(time_ds, gripper_data_ds[1][:], '--', label="DS position")
	
	ftime = gripper_data[2][:, 0] + gripper_data[2][:, 1] * (10.0**-9)
	ftime_ds = gripper_data_ds[2][:, 0] + gripper_data_ds[2][:, 1] * (10.0**-9)
	ax2.plot(ftime, gripper_data[3][:], label="force")
	ax2.plot(ftime_ds, gripper_data_ds[3][:], '--', label="DS force")
	
	ax1.legend()
	ax2.legend()
	
	ax2.set_xlabel('time')
	ax1.set_ylabel('vals')
	ax2.set_ylabel('forces')
	
def plot_data(fname):
    joint_data, tf_data, wrench_data, gripper_data, joint_data_ds, tf_data_ds, wrench_data_ds, gripper_data_ds = read_data(fname)
    display_data(joint_data, tf_data, wrench_data, gripper_data, joint_data_ds, tf_data_ds, wrench_data_ds, gripper_data_ds)
    plot_joint_data(joint_data, joint_data_ds)
    plot_tf_data(tf_data, tf_data_ds)
    plot_wrench_data(wrench_data, wrench_data_ds)
    plot_gripper_data(gripper_data, gripper_data_ds)
    plt.show()
    return

def main():
    filename = '/home/pearl/catkin_ws/src/pearl_ur5e/src/scripts/h5_files/3-1 demos/recorded_demo 2024-03-01 15:42:38.h5'#raw_input('Enter the filename of the .h5 demo: ')
    plot_data(filename)



if __name__ == '__main__':
  main()

