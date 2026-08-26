import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import h5py

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
    gripper_pos = np.array(gp.get('gripper_position'))
    #gripper_time = []
    #gripper_pos = []
    gripper_data = [gripper_time, gripper_pos]
    
    
    
    hf.close()
    
    return joint_data, tf_data, wrench_data, gripper_data
    
def plot_with_slider_1D(data):
    
    cur_idx = 0
    
    fig, ax = plt.subplots()
    plt.subplots_adjust(bottom=0.35)
    
    idxs = []
    
    for d in range(len(data)):
        plt.plot(data[d], lw=3, label='Data ' + str(d))
        idxs.append(plt.plot(cur_idx, data[d][cur_idx], '.', ms=9, label='Data ' + str(d) + ' index ' + str(cur_idx)))
    
    axidx = plt.axes([0.25, 0.15, 0.65, 0.03])
    idx_slider = Slider(axidx, 'Index', 0, len(data[0])-1, valinit=cur_idx)

    def update(val):
        cur_idx = int(idx_slider.val)
        for d in range(len(data)):
            idxs[d][0].set_xdata([cur_idx])
            idxs[d][0].set_ydata(data[d][cur_idx])
        
    idx_slider.on_changed(update)
    ax.legend()
    ax.set_xlabel('indices')
    plt.show()
    
def plot_with_slider_3D(data):
    cur_idx = 0
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    plt.subplots_adjust(bottom=0.35)
    ax.plot3D(data[:, 0], data[:, 1], data[:, 2], 'k', lw=3)
    dot, = ax.plot3D(data[cur_idx, 0], data[cur_idx, 1], data[cur_idx, 2], 'r.', ms=9)
    
    axidx = plt.axes([0.25, 0.15, 0.65, 0.03])
    idx_slider = Slider(axidx, 'Index', 0, len(data)-1, valinit=cur_idx)

    def update(val):
        cur_idx = int(idx_slider.val)
        dot.set_xdata([data[cur_idx, 0]])
        dot.set_ydata([data[cur_idx, 1]])
        dot.set_3d_properties([data[cur_idx, 2]])
        #fig.canvas.draw()
        
    idx_slider.on_changed(update)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
    
def main_traj():
    joint_data, tf_data, wrench_data, gripper_data = read_data('/home/pearl/catkin_ws/src/pearl_ur5e/src/scripts/h5_files/recorded_demo 2026-06-04 15:02:03.h5')
    #inds = np.loadtxt('inds13_38_50.txt').astype('int')
    #demo = full_demo[inds]
    plot_with_slider_3D(tf_data[1])
    
def main_demo():
    x = np.linspace(0, 10).reshape((50, 1))
    y = np.sin(x) + 3*np.cos(x)**2
    z = 0.005*(x-5)**4 - 0.05*(x-5)**2
    traj_list = [x, y, z]
    traj = np.hstack((x, y, z))
    plot_with_slider_1D(traj_list)
    plot_with_slider_3D(traj)
    
if __name__ == '__main__':
    main_traj()
