import numpy as np
import h5py
import csv
from downsampling import *

from scipy.interpolate import UnivariateSpline

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
    
def read_data_ds(fname):
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

def make_dataset_rs():
    fp = h5py.File('Grasping_Demos_RS.h5', 'w')

    with open('Demo Recordings - Sheet1.csv', newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar=' ')
        for row in spamreader:
            print(row)
            joint_data, tf_data, wrench_data, gripper_data, joint_data_ds, tf_data_ds, wrench_data_ds, gripper_data_ds = read_data_rs('h5_files/3-1 demos/recorded_demo 2024-03-01 ' + row[0] + '.h5')
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
    
def make_dataset_ds():
    fp = h5py.File('Grasping_Demos_DS.h5', 'w')

    with open('Demo Recordings - Sheet1.csv', newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar=' ')
        for row in spamreader:
            print(row)
            joint_data, tf_data, wrench_data, gripper_data, joint_data_ds, tf_data_ds, wrench_data_ds, gripper_data_ds = read_data_ds('h5_files/3-1 demos/recorded_demo 2024-03-01 ' + row[0] + '.h5')
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



def make_dataset():
    fp = h5py.File('Grasping_Demos.h5', 'w')

    with open('Demo Recordings - Sheet1.csv', newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar=' ')
        for row in spamreader:
            print(row)
            joint_data, tf_data, wrench_data, gripper_data = read_data('h5_files/3-1 demos/recorded_demo 2024-03-01 ' + row[0] + '.h5')
            prefix = '/' + row[4] + '/' + row[3] + '/' + 'Position' + row[5]
            dset_jt = fp.create_dataset(prefix + '/joint_state_info/joint_time', data=joint_data[0])
            dset_jp = fp.create_dataset(prefix + '/joint_state_info/joint_positions', data=joint_data[1])
            dset_jv = fp.create_dataset(prefix + '/joint_state_info/joint_velocities', data=joint_data[2])
            dset_je = fp.create_dataset(prefix + '/joint_state_info/joint_effort', data=joint_data[3])
            	
            dset_tt = fp.create_dataset(prefix + '/transform_info/transform_time', data=tf_data[0])
            dset_tp = fp.create_dataset(prefix + '/transform_info/transform_positions', data=tf_data[1])
            dset_tr = fp.create_dataset(prefix + '/transform_info/transform_orientations', data=tf_data[2])
            	
            dset_wt = fp.create_dataset(prefix + '/wrench_info/wrench_time', data=wrench_data[0])
            dset_wf = fp.create_dataset(prefix + '/wrench_info/wrench_force', data=wrench_data[1])
            dset_wm = fp.create_dataset(prefix + '/wrench_info/wrench_torque', data=wrench_data[2])
            	
            dset_gt = fp.create_dataset(prefix + '/gripper_info/gripper_time', data=gripper_data[0])
            dset_gp = fp.create_dataset(prefix + '/gripper_info/gripper_vals', data=gripper_data[1])
            dset_ft = fp.create_dataset(prefix + '/gripper_info/force_time', data=gripper_data[2])
            dset_fp = fp.create_dataset(prefix + '/gripper_info/forces', data=gripper_data[3])
    fp.close()
    
def pull_data(object_type, user_num, position_num, downsampled=True, resampled=False):
    if downsampled:
        hf = h5py.File('h5_files/Grasping_Demos_DS.h5', 'r')
    elif resampled:
        hf = h5py.File('h5_files/Grasping_Demos_RS.h5', 'r')
    else: 
        hf = h5py.File('h5_files/Grasping_Demos.h5', 'r')
    print(list(hf.keys()))
    obj = hf.get(object_type)
    print(list(obj.keys()))
    user = obj.get('User' + str(user_num))
    print(list(user.keys()))
    pos = user.get('Position' + str(position_num))
    print(list(pos.keys()))
    js = pos.get('joint_state_info')
    jt, jp, jv, je = np.array(js.get('joint_time')), np.array(js.get('joint_positions')), np.array(js.get('joint_velocities')), np.array(js.get('joint_effort'))
    tf = pos.get('transform_info')
    tt, tp, tr = np.array(tf.get('transform_time')), np.array(tf.get('transform_positions')), np.array(tf.get('transform_orientations'))
    wr = pos.get('wrench_info')
    wt, wf, wm = np.array(wr.get('wrench_time')), np.array(wr.get('wrench_force')), np.array(wr.get('wrench_torque'))
    gf = pos.get('gripper_info')
    gt, gp, ft, fp = np.array(gf.get('gripper_time')), np.array(gf.get('gripper_vals')), np.array(gf.get('force_time')), np.array(gf.get('forces'))
    hf.close()
    return [[jt, jp, jv, je], [tt, tp, tr], [wt, wf, wm], [gt, gp, ft, fp]]

def pull_data_across_users(object_type, position_num, downsampled=True, resampled=False):
    data = []
    for user_num in range(1, 5):
        data.append(pull_data(object_type, user_num, position_num, downsampled, resampled))
    return data
    
if __name__ == '__main__':
    data = pull_data_across_users('Plate', 1, False, True)
    print(data)
