import numpy as np
import h5py
import csv
from downsampling import *

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

fp = h5py.File('Grasping_Demos_DS.h5', 'w')

with open('Demo Recordings - Sheet1.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',', quotechar=' ')
    for row in spamreader:
        print(row)
        joint_data, tf_data, wrench_data, gripper_data, joint_data_ds, tf_data_ds, wrench_data_ds, gripper_data_ds = read_data('h5_files/3-1 demos/recorded_demo 2024-03-01 ' + row[0] + '.h5')
        prefix = '/' + row[4] + '/' + row[3] + '/' + 'Position' + row[5]
        dset_jt = fp.create_dataset(prefix + '/joint_state_info/joint_time', data=joint_data_ds[0])
        dset_jp = fp.create_dataset(prefix + '/joint_state_info/joint_positions', data=joint_data_ds[1])
        dset_jv = fp.create_dataset(prefix + '/joint_state_info/joint_velocities', data=joint_data_ds[2])
        dset_je = fp.create_dataset(prefix + '/joint_state_info/joint_effort', data=joint_data_ds[3])
        	
        dset_tt = fp.create_dataset(prefix + '/transform_info/transform_time', data=tf_data_ds[0])
        dset_tp = fp.create_dataset(prefix + '/transform_info/transform_positions', data=tf_data_ds[1])
        dset_tr = fp.create_dataset(prefix + '/transform_info/transform_orientations', data=tf_data_ds[2])
        	
        dset_wt = fp.create_dataset(prefix + '/wrench_info/wrench_time', data=wrench_data_ds[0])
        dset_wf = fp.create_dataset(prefix + '/wrench_info/wrench_force', data=wrench_data_ds[1])
        dset_wm = fp.create_dataset(prefix + '/wrench_info/wrench_torque', data=wrench_data_ds[2])
        	
        dset_gt = fp.create_dataset(prefix + '/gripper_info/gripper_time', data=gripper_data_ds[0])
        dset_gp = fp.create_dataset(prefix + '/gripper_info/gripper_vals', data=gripper_data_ds[1])
        dset_ft = fp.create_dataset(prefix + '/gripper_info/force_time', data=gripper_data_ds[2])
        dset_fp = fp.create_dataset(prefix + '/gripper_info/forces', data=gripper_data_ds[3])
fp.close()
