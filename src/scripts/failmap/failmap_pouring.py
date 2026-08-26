import numpy as np
import matplotlib.pyplot as plt

from shapely import box, Point
from shapely.plotting import plot_polygon
import matplotlib as mpl
from scipy.spatial.transform import Rotation

from failmap import FailMap
from utils import *
from downsampling import *

cmap = plt.get_cmap("RdYlGn")

fpath_prefix = '../h5_files/recorded_demo 2026-01-07 '
demo_names = ['12_40_30', '12_49_14', '13_34_23', '13_38_50', '13_42_20', '14_00_23']
num_beans_transferred = [151, 265, 151, 121, 0, 39]
num_beans_spilled = [114, 62, 21, 9, 594, 143]

idx = 627

def main_eulers():
    demos = []
    weights = []
    for i in range(len(demo_names)):
        full_demo = read_recorded_demo_orientations(fpath_prefix + demo_names[i] + '.h5')
        #demo, inds = DouglasPeuckerPoints2(full_demo, 1000)
        #np.savetxt('inds' + demo_names[i] + '.txt', inds)
        inds = np.loadtxt('inds' + demo_names[i] + '.txt').astype('int')
        demo = full_demo[inds]
        new_demo = np.zeros((1000, 3))
        for j in range(len(demo)):
            rot = Rotation.from_quat(demo[j, :])
            new_demo[j, :] = rot.as_euler('xyz')
        print(new_demo[0, :])
        print(np.shape(demo))
        score = (num_beans_transferred[i] - num_beans_spilled[i]) / (num_beans_transferred[i] + num_beans_spilled[i])
        print(score)
        dweights = score * np.ones((len(new_demo),))
        demos.append(new_demo)
        weights.append(dweights)
    all_weights = np.hstack((weights))
    elmap1 = FailMap(demos, n=100, weights=all_weights, downsampling='distance', init_lambda=0.001, init_mu=0.001, c_lambda=1.01, c_mu=1.01)
    new_traj = elmap1.optimize_map([demos[3][0, :].flatten(), demos[3][idx, :].flatten(), demos[3][-1, :].flatten()], [0, int(100 * idx / 1000), 100-1])
    
    
    fig, axs = plt.subplots(3, 1)
    for i in range(len(demo_names)):
        norm_weight = (weights[i][0] + 1) / 2
        for j in range(3):
            axs[j].plot(np.linspace(0, 1, 1000), demos[i][:, j], color=cmap(norm_weight), lw=5, alpha=0.8)
    for j in range(3):
        axs[j].plot(np.linspace(0, 1, 100), new_traj[:, j], 'tab:brown', lw=5, alpha=0.8)
    plt.show
    
    out_traj = np.zeros((100, 4))
    for i in range(len(new_traj)):
        rot = Rotation.from_euler('xyz', new_traj[i, :])
        out_traj[i, :] = rot.as_quat()
    np.savetxt('failmap_pouring_eulerquattraj.txt', out_traj)

def main_quats():
    demos = []
    weights = []
    for i in range(len(demo_names)):
        full_demo = read_recorded_demo_orientations(fpath_prefix + demo_names[i] + '.h5')
        #demo, inds = DouglasPeuckerPoints2(full_demo, 1000)
        #np.savetxt('inds' + demo_names[i] + '.txt', inds)
        inds = np.loadtxt('inds' + demo_names[i] + '.txt').astype('int')
        demo = full_demo[inds]
        print(np.shape(demo))
        score = (num_beans_transferred[i] - num_beans_spilled[i]) / (num_beans_transferred[i] + num_beans_spilled[i])
        print(score)
        dweights = score * np.ones((len(demo),))
        demos.append(demo)
        weights.append(dweights)
    all_weights = np.hstack((weights))
    elmap1 = FailMap(demos, n=100, weights=all_weights, downsampling='distance', init_lambda=0.001, init_mu=0.001, c_lambda=1.01, c_mu=1.01)
    new_traj = elmap1.optimize_map([demos[3][0, :].flatten(), demos[3][idx, :].flatten(), demos[3][-1, :].flatten()], [0, int(100 * idx / 1000), 100-1])
    
    for i in range(len(new_traj)):
        new_traj[i, :] = new_traj[i, :] / np.linalg.norm(new_traj[i, :])
    
    fig, axs = plt.subplots(4, 1)
    for i in range(len(demo_names)):
        norm_weight = (weights[i][0] + 1) / 2
        for j in range(4):
            axs[j].plot(np.linspace(0, 1, 1000), demos[i][:, j], color=cmap(norm_weight), lw=5, alpha=0.8)
    for j in range(4):
        axs[j].plot(np.linspace(0, 1, 100), new_traj[:, j], 'tab:brown', lw=5, alpha=0.8)
    
    plt.show()
    np.savetxt('failmap_pouring_quattraj.txt', new_traj)

def main_joints():

    NUM_JOINTS = 6

    demos = []
    weights = []
    for i in range(len(demo_names)):
        full_demo = read_recorded_demo_joints(fpath_prefix + demo_names[i].replace('_', ':') + '.h5')
        #demo, inds = DouglasPeuckerPoints2(full_demo, 1000)
        #np.savetxt('inds' + demo_names[i] + '.txt', inds)
        inds = np.loadtxt('inds' + demo_names[i] + '.txt').astype('int')
        demo = full_demo[inds]
        times = np.linspace(0, 1, 1000).reshape((1000, 1))
        demo_t = np.hstack((times, demo))
        print(np.shape(demo_t))
        dweights = ((num_beans_transferred[i] - num_beans_spilled[i]) / (num_beans_transferred[i] + num_beans_spilled[i])) * np.ones((len(demo),))
        demos.append(demo)
        weights.append(dweights)
    all_weights = np.hstack((weights))
    elmap1 = FailMap(demos, n=100, weights=all_weights, downsampling='naive', init_lambda=0.001, init_mu=0.01, c_lambda=1.01, c_mu=1.01)
    new_traj = elmap1.optimize_map([demos[3][0, :].flatten(), demos[3][100, :].flatten(), demos[3][200, :].flatten(), demos[3][300, :].flatten(), demos[3][400, :].flatten(), demos[3][500, :].flatten(), demos[3][600, :].flatten(), demos[3][700, :].flatten(), demos[3][800, :].flatten(), demos[3][900, :].flatten(), demos[3][-1, :].flatten()], [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100-1])
    
    fig, axs = plt.subplots(6, 1)
    for i in range(len(demo_names)):
        norm_weight = (weights[i][0] + 1) / 2
        for j in range(NUM_JOINTS):
            axs[j].plot(np.linspace(0, 1, 1000), demos[i][:, j], color=cmap(norm_weight), lw=5, alpha=0.8)
    for j in range(NUM_JOINTS):
        axs[j].plot(np.linspace(0, 1, 100), new_traj[:, j], 'tab:brown', lw=5, alpha=0.8)
    
    plt.show()
    np.savetxt('failmap_pouring_jointtraj.txt', new_traj)

def main():
    demos = []
    weights = []
    for i in range(len(demo_names)):
        full_demo = read_recorded_demo(fpath_prefix + demo_names[i] + '.h5')
        #demo, inds = DouglasPeuckerPoints2(full_demo, 1000)
        #np.savetxt('inds' + demo_names[i] + '.txt', inds)
        inds = np.loadtxt('inds' + demo_names[i] + '.txt').astype('int')
        demo = full_demo[inds]
        print(np.shape(demo))
        dweights = ((num_beans_transferred[i] - num_beans_spilled[i]) / (num_beans_transferred[i] + num_beans_spilled[i])) * np.ones((len(demo),))
        demos.append(demo)
        weights.append(dweights)
    all_weights = np.hstack((weights))
    elmap1 = FailMap(demos, n=100, weights=all_weights, downsampling='distance', init_lambda=0.04, init_mu=0.001, c_lambda=1.01, c_mu=1.01)
    new_traj = elmap1.optimize_map([np.array([demos[3][0, 0], demos[3][0, 1], demos[3][0, 2]]).flatten(), np.array([demos[3][idx, 0], demos[3][idx, 1], demos[3][idx, 2]]).flatten(), np.array([demos[3][-1, 0], demos[3][-1, 1], demos[3][-1, 2]]).flatten()], [0, int(100 * idx / 1000), 100-1])
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i in range(len(demo_names)):
        norm_weight = (weights[i][0] + 1) / 2
        plt.plot(demos[i][:, 0], demos[i][:, 1], demos[i][:, 2], color=cmap(norm_weight), lw=5, alpha=0.8)
    plt.plot(new_traj[:, 0], new_traj[:, 1], new_traj[:, 2], 'tab:brown', lw=5, alpha=0.8)
    
    plt.plot(demos[3][0, 0], demos[3][0, 1], demos[3][0, 2], 'k', marker='o', lw=5)
    plt.plot(demos[3][idx, 0], demos[3][idx, 1], demos[3][idx, 2], 'k', marker='x', lw=5)
    plt.plot(demos[3][-1, 0], demos[3][-1, 1], demos[3][-1, 2], 'k', marker='x', lw=5)
    plt.show()
    np.savetxt('failmap_pouring_traj.txt', new_traj)
    
if __name__ == '__main__':
    main_joints()
