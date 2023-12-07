import h5py
import numpy as np
import cvxpy as cp
from scipy.interpolate import UnivariateSpline
import matplotlib.pyplot as plt 
from compile_demos import *

def get_data(fname):
    hf = h5py.File(fname, 'r')
    tf = hf.get('transform_info')
    tf_time = np.array(tf.get('transform_time'))
    tf_pos = np.array(tf.get('transform_positions'))
    
    gr = hf.get('gripper_info')
    gr_time = np.array(gr.get('force_time'))
    gr_force = np.array(gr.get('forces'))
    
    
    tf_time2 = tf_time[:, 0] + (tf_time[:, 1] * 1e-9)
    gr_time2 = gr_time[:, 0] + (gr_time[:, 1] * 1e-9)
    
    hf.close()
    
    min_time = max(min(tf_time2), min(gr_time2))
    max_time = min(max(tf_time2), max(gr_time2))
    
    N = 100
    
    new_time = np.linspace(min_time, max_time, N)
    spgr = UnivariateSpline(gr_time2, gr_force, s=0)
    spx = UnivariateSpline(tf_time2, tf_pos[:, 0], s=0)
    spy = UnivariateSpline(tf_time2, tf_pos[:, 1], s=0)
    spz = UnivariateSpline(tf_time2, tf_pos[:, 2], s=0)
    
    new_gr = spgr(new_time)
    new_gr = np.reshape(new_gr, (N, 1))
    
    new_x = spx(new_time)
    new_y = spy(new_time)
    new_z = spz(new_time)
    new_x = np.reshape(new_x, (N, 1))
    new_y = np.reshape(new_y, (N, 1))
    new_z = np.reshape(new_z, (N, 1))
    
    #plt.figure()
    #plt.title('gripper')
    #plt.plot(gr_time2, gr_force, 'k')
    #plt.plot(new_time, new_gr, 'r')
    #plt.figure()
    #plt.title('x')
    #plt.plot(tf_time2, tf_pos[:, 0], 'k')
    #plt.plot(new_time, new_x, 'r')
    #plt.figure()
    #plt.title('y')
    #plt.plot(tf_time2, tf_pos[:, 1], 'k')
    #plt.plot(new_time, new_y, 'r')
    #plt.figure()
    #plt.title('z')
    #plt.plot(tf_time2, tf_pos[:, 2], 'k')
    #plt.plot(new_time, new_z, 'r')
    #plt.show()
    return new_time, np.hstack((new_x, new_y, new_z)), new_gr

class ElMap_CVX(object):

    def __init__(self, traj, stretch=0.01, bend=0.001):
        self.tgt_traj = traj
        self.n_pts, self.n_dims = np.shape(self.tgt_traj)
        self.num_points = np.size(self.tgt_traj)
        if self.n_dims == 1:
            self.traj_x = self.tgt_traj
            self.traj_stacked = self.traj_x
        elif self.n_dims == 2:
            self.traj_x = np.reshape(self.tgt_traj[:, 0], (self.n_pts, 1))
            self.traj_y = np.reshape(self.tgt_traj[:, 1], (self.n_pts, 1))
            self.traj_stacked = np.vstack((self.traj_x, self.traj_y))
        elif self.n_dims == 3:
            self.traj_x = np.reshape(self.tgt_traj[:, 0], (self.n_pts, 1))
            self.traj_y = np.reshape(self.tgt_traj[:, 1], (self.n_pts, 1))
            self.traj_z = np.reshape(self.tgt_traj[:, 2], (self.n_pts, 1))
            self.traj_stacked = np.vstack((self.traj_x, self.traj_y, self.traj_z))
            self.n_pts2 = 2*self.n_pts
        elif self.n_dims == 4:
            self.traj_x = np.reshape(self.tgt_traj[:, 0], (self.n_pts, 1))
            self.traj_y = np.reshape(self.tgt_traj[:, 1], (self.n_pts, 1))
            self.traj_z = np.reshape(self.tgt_traj[:, 2], (self.n_pts, 1))
            self.traj_w = np.reshape(self.tgt_traj[:, 3], (self.n_pts, 1))
            self.traj_stacked = np.vstack((self.traj_x, self.traj_y, self.traj_z, self.traj_w))
            self.n_pts2 = 2*self.n_pts
            self.n_pts3 = 3*self.n_pts
        else:
            print("Too many dimensions! n_dims must be <= 4")
            exit()
        self.stretch_const = stretch
        self.bend_const = bend
        
    def setup_problem(self):
        K = np.diag(np.ones(self.num_points))
        I = np.eye(self.num_points)
        e1 = np.diag(-1*np.ones(self.num_points-1))
        e2 = np.diag(np.ones(self.num_points-1))
        E = np.zeros((self.num_points-1, self.num_points))
        E[:,0:self.num_points-1]+= e1
        E[:,1:self.num_points] += e2
        if self.n_dims >= 2:
            E[self.n_pts-1, self.n_pts-1] = 0
            E[self.n_pts-1, self.n_pts] = 0
        if self.n_dims >= 3:
            E[self.n_pts2-1, self.n_pts2-1] = 0
            E[self.n_pts2-1, self.n_pts2] = 0
        if self.n_dims >= 4:
            E[self.n_pts3-1, self.n_pts3-1] = 0
            E[self.n_pts3-1, self.n_pts3] = 0
        r1 = np.diag(np.ones(self.num_points-2))
        r2 = -2*np.diag(np.ones(self.num_points-2))
        R = np.zeros((self.num_points-2, self.num_points))
        R[:,0:self.num_points-2] += r1
        R[:,1:self.num_points-1] += r2
        R[:,2:self.num_points] += r1
        if self.n_dims >= 2:
            R[self.n_pts-2, self.n_pts-2] = 0
            R[self.n_pts-2, self.n_pts-1] = 0
            R[self.n_pts-2, self.n_pts] = 0
            R[self.n_pts-1, self.n_pts-1] = 0
            R[self.n_pts-1, self.n_pts] = 0
            R[self.n_pts-1, self.n_pts+1] = 0
        if self.n_dims >= 3:
            R[self.n_pts2-2, self.n_pts2-2] = 0
            R[self.n_pts2-2, self.n_pts2-1] = 0
            R[self.n_pts2-2, self.n_pts2] = 0
            R[self.n_pts2-1, self.n_pts2-1] = 0
            R[self.n_pts2-1, self.n_pts2] = 0
            R[self.n_pts2-1, self.n_pts2+1] = 0  
        if self.n_dims >= 4:
            R[self.n_pts3-2, self.n_pts3-2] = 0
            R[self.n_pts3-2, self.n_pts3-1] = 0
            R[self.n_pts3-2, self.n_pts3] = 0
            R[self.n_pts3-1, self.n_pts3-1] = 0
            R[self.n_pts3-1, self.n_pts3] = 0
            R[self.n_pts3-1, self.n_pts3+1] = 0  
        
        #self.K = K
        #self.I = I
        #self.E = E
        #self.R = R
        
        #print(R[self.n_pts-3:self.n_pts+3, self.n_pts-3:self.n_pts+3])
           
        #ex = E @ self.traj_stacked
        #rx = R @ self.traj_stacked
            
        #print(ex[self.n_pts-3:self.n_pts+3])
        #print(rx[self.n_pts-3:self.n_pts+3])
            
        #print(np.shape(self.traj_stacked))
        self.x = cp.Variable(np.shape(self.traj_stacked))
        #self.objective = cp.Minimize(cp.norm(cp.square(K @ self.traj_stacked - self.x), 2) 
        #                            + self.stretch_const * cp.norm(cp.square(E @ self.x), 2)
        #                            + self.bend_const * cp.norm(cp.square(R @ self.x), 2))
        self.objective = cp.Minimize(cp.sum_squares(I @ self.x - K @ self.traj_stacked) 
                                    + self.stretch_const * cp.sum_squares(E @ self.x)
                                    + self.bend_const * cp.sum_squares(R @ self.x))
        return self.x
        
    def solve_problem(self, constraints=[], disp=True):
        self.consts = constraints
        
        self.problem = cp.Problem(self.objective, self.consts)
        self.problem.solve(verbose=disp)
        
        if disp:
            print("status:", self.problem.status)
            print("optimal value", self.problem.value)
            #print("optimal var", x.value)
            for i in range(len(self.consts)):
                print("dual value for constraint " + str(i), ": ", constraints[i].dual_value)
            
        if self.n_dims == 1:
            self.sol = self.x.value
        elif self.n_dims == 2:
            self.sol = np.hstack((self.x.value[:self.n_pts], self.x.value[self.n_pts:]))
        elif self.n_dims == 3:
            self.sol = np.hstack((self.x.value[:self.n_pts], self.x.value[self.n_pts:self.n_pts2], self.x.value[self.n_pts2:]))
        elif self.n_dims == 4:
            self.sol = np.hstack((self.x.value[:self.n_pts], self.x.value[self.n_pts:self.n_pts2], self.x.value[self.n_pts2:self.n_pts3], self.x.value[self.n_pts3:]))
        
        return self.sol
        
    def plot_solved_problem(self):
        if self.n_dims == 1:
            fig = plt.figure()
            plt.plot(self.tgt_traj, 'k', lw=5, label="Demo")
            plt.plot(self.sol, 'r', lw=5, label="Repro")
            return fig
        if self.n_dims == 2:
            fig = plt.figure()
            plt.plot(self.traj_x, self.traj_y, 'k', lw=3, label="Demo")
            plt.plot(self.sol[:, 0], self.sol[:, 1], 'r', lw=3, label="Repro")
            return fig
        if self.n_dims == 3:
            print("3D PLOTTING NOT IMPLEMENTED YET")
        return
        
def main():
    #time, pos_data, gripper_data = get_data('h5_files/recorded_demo 2023-11-28 12:16:17.h5')
    data = pull_data_across_users('Plate', 1)
    data_usr1 = data[0]
    [joint_data, tf_data, wrench_data, gripper_data] = data_usr1
    time = tf_data[0]
    time = time[:, 0] + time[:, 1] * 1e-9
    print(np.shape(time), time)
    pos_data = tf_data[1]
    gripper_data = gripper_data[3]
    all_data = np.hstack((pos_data, gripper_data))
    print(np.shape(all_data))
    emcvx = ElMap_CVX(all_data, stretch=10.0, bend=10.0)
    x = emcvx.setup_problem()
    new_traj = emcvx.solve_problem()
    
    gr_force = new_traj[:, 3]
    tf_traj = new_traj[:, :3]
    
    
    plt.figure()
    plt.title('force')
    plt.plot(time, gripper_data, 'k')
    plt.plot(time, gr_force, 'r')
    plt.figure()
    plt.title('x')
    plt.plot(time, pos_data[:, 0], 'k')
    plt.plot(time, tf_traj[:, 0], 'r')
    plt.figure()
    plt.title('y')
    plt.plot(time, pos_data[:, 1], 'k')
    plt.plot(time, tf_traj[:, 1], 'r')
    plt.figure()
    plt.title('z')
    plt.plot(time, pos_data[:, 2], 'k')
    plt.plot(time, tf_traj[:, 2], 'r')
    
    fig = plt.figure()
    fig.suptitle('Trajectory')
    ax = plt.axes(projection='3d')
    ax.plot3D(pos_data[:, 0], pos_data[:, 1], pos_data[:, 2], 'k')
    ax.plot3D(tf_traj[:, 0], tf_traj[:, 1], tf_traj[:, 2], 'r')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
    
    return tf_traj, gr_force
    
    

if __name__ == '__main__':
    main()

