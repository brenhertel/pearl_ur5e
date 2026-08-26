import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp

from scipy.optimize import minimize

#from utils import *
from downsampling import *

from plot_demo_arm_only import read_data

def cluster(x, y):
    n_pts, n_dims = np.shape(x)
    n_nodes, _ = np.shape(y)
    clusters = [[] for j in range(n_nodes)]
    # cluster to closest node
    for i in range(n_pts):
        dist = float('inf')
        idx = -1
        for j in range(n_nodes):
            new_dist = np.linalg.norm(x[i] - y[j])
            if new_dist < dist:
                dist = new_dist
                idx = j
        if idx != -1:
            #print(i, j, dist)
            clusters[idx].append(i)
        else:
            print('No Cluster Found!!!')
    return clusters
    
def gen_laplacian(n):
    l1 = 0.5*np.diag(np.ones(n-1))
    l2 = -1*np.eye(n)
    L = np.zeros((n, n))
    L[1:,0:n-1] += l1
    L += l2
    L[:-1,1:n] += l1
    L[0, 1] = 1
    L[-1, -2] = 1
    #print(L)
    return L
    
def gen_tangent(n):
    t1 = np.diag(np.ones(n-1))
    t2 = -1*np.diag(np.ones(n-1))
    T = np.zeros((n, n))
    T[1:,0:n-1] += t2
    T[1:,1:n] += t1
    #print(T)
    return T
    
class MC_elmap(object):

    def __init__(self, trajs, weighting='custom', spatial=1.0, tangent=1.0, shape=1.0, stretch=0.1, bend=0.01, n=100, init_nodes=None):
        self.trajs = trajs
        self.num_trajs = len(self.trajs)
        self.n_pts, self.n_dims = np.shape(self.trajs[0])
        self.n_nodes = n
        self.n_size = self.n_nodes * self.n_dims
        
        if init_nodes is not None:
            self.nodes = init_nodes
        else:
            self.nodes = DouglasPeuckerPoints(self.trajs[0], self.n_nodes)
        self.nodes_stacked = np.reshape(self.nodes, (self.n_size, 1), order='F')
        
        self.weighting_scheme = weighting
        
        self.spatial_alpha = spatial
        self.spatial_beta = 1.0
        self.tangent_alpha = tangent
        self.tangent_beta = 1.0
        self.shape_alpha = shape
        self.shape_beta = 1.0
        
        self.calc_weights()
        
        self.stretch_const = stretch
        self.bend_const = bend
        
        self.transform_tangent()
        self.transform_laplacian()
        self.stack_demos()
        
        e1 = np.diag(-1*np.ones(self.n_nodes-1), -1)
        e2 = np.diag(np.ones(self.n_nodes))
        self.E = np.zeros((self.n_nodes, self.n_nodes))
        self.E += e1
        self.E += e2
        self.E[0, 0] = 0
        
        self.E_cvx = np.zeros((self.n_size, self.n_size))
        for i in range(self.n_dims):
            self.E_cvx[self.n_nodes*i : self.n_nodes*(i + 1), self.n_nodes*i : self.n_nodes*(i + 1)] = self.E
            
        r1 = 0.5*np.diag(np.ones(self.n_nodes-1))
        r2 = -1*np.diag(np.ones(self.n_nodes))
        self.R = np.zeros((self.n_nodes, self.n_nodes))
        self.R[1:,0:self.n_nodes-1] += r1
        self.R += r2
        self.R[:-1,1:self.n_nodes] += r1
        self.R[0, 1] = 1
        self.R[-1, -2] = 1
        
        self.R_cvx = np.zeros((self.n_size, self.n_size))
        for i in range(self.n_dims):
            self.R_cvx[self.n_nodes*i : self.n_nodes*(i + 1), self.n_nodes*i : self.n_nodes*(i + 1)] = self.R
            
        #print(self.E_cvx)
        #print(self.R_cvx)
        
        self.set_constraints()
        
    def calc_weights(self):
        self.spatial_const = self.spatial_alpha / self.spatial_beta
        self.tangent_const = self.tangent_alpha / self.tangent_beta
        self.shape_const = self.shape_alpha / self.shape_beta
        
    def transform_tangent(self):
        T = gen_tangent(self.n_pts)
        self.tan_trajs = []
        for i in range(self.num_trajs):
            self.tan_trajs.append(T @ self.trajs[i])
        
    def transform_laplacian(self):
        L = gen_laplacian(self.n_pts)
        self.lap_trajs = []
        for i in range(self.num_trajs):
            self.lap_trajs.append(L @ self.trajs[i])
            
    def stack_demos(self):
        self.demos_stacked = np.zeros((self.num_trajs * self.n_pts * self.n_dims, 1))
        self.tan_demos_stacked = np.zeros((self.num_trajs * self.n_pts * self.n_dims, 1))
        self.lap_demos_stacked = np.zeros((self.num_trajs * self.n_pts * self.n_dims, 1))
        for j in range(self.n_dims):
            for i in range(self.num_trajs):
                #print(np.shape(self.demos_stacked[(i*self.n_pts) + (j * self.num_trajs * self.n_pts): ((i+1)*self.n_pts) + (j * self.num_trajs * self.n_pts)]))
                #print(np.shape(self.trajs[i][:, j]))
                self.demos_stacked[(i*self.n_pts) + (j * self.num_trajs * self.n_pts): ((i+1)*self.n_pts) + (j * self.num_trajs * self.n_pts)] = np.reshape(self.trajs[i][:, j], (self.n_pts, 1))
                self.tan_demos_stacked[(i*self.n_pts) + (j * self.num_trajs * self.n_pts): ((i+1)*self.n_pts) + (j * self.num_trajs * self.n_pts)] = np.reshape(self.tan_trajs[i][:, j], (self.n_pts, 1))
                self.lap_demos_stacked[(i*self.n_pts) + (j * self.num_trajs * self.n_pts): ((i+1)*self.n_pts) + (j * self.num_trajs * self.n_pts)] = np.reshape(self.lap_trajs[i][:, j], (self.n_pts, 1))
      
    def calc_scaling_factors(self):
        spatial_error = 0
        tangent_error = 0
        laplacian_error = 0
        for i in range(self.num_trajs):
            clusters_i = cluster(self.trajs[i], self.nodes)
            A_i = np.zeros((self.n_nodes*self.n_dims, self.n_nodes*self.n_dims))
            B_i = np.zeros((self.n_nodes*self.n_dims, self.n_pts*self.n_dims))
            for j in range(self.n_nodes):
                for pt in clusters_i[j]:
                    for k in range(self.n_dims):
                        B_i[j + self.n_nodes*k, pt + self.n_pts*k] = 1 #change for variable weighting
                for k in range(self.n_dims):
                    A_i[j + self.n_nodes*k, j + self.n_nodes*k] = np.sum(B_i[j + self.n_nodes*k, :])
                    
            spatial_error = spatial_error + np.linalg.norm(A_i @ self.nodes_stacked - B_i @ np.reshape(self.trajs[i], (self.n_pts * self.n_dims), order='F'))
            tangent_error = tangent_error + np.linalg.norm(A_i @ (self.E_cvx @ self.nodes_stacked) - B_i @ np.reshape(self.tan_trajs[i], (self.n_pts * self.n_dims), order='F'))
            laplacian_error = laplacian_error + np.linalg.norm(A_i @ (self.R_cvx @ self.nodes_stacked) - B_i @ np.reshape(self.lap_trajs[i], (self.n_pts * self.n_dims), order='F'))
            
        sum = spatial_error + tangent_error + laplacian_error
        return [spatial_error / sum, tangent_error / sum, laplacian_error / sum]
      
    def smoothing_autotuning(self, A, B):
        approx_cost = self.spatial_const * np.linalg.norm(A @ self.nodes_stacked - B @ self.demos_stacked) + self.tangent_const * np.linalg.norm(A @ (self.E_cvx @ self.nodes_stacked) - B @ self.tan_demos_stacked) + self.shape_const * np.linalg.norm(A @ (self.R_cvx @ self.nodes_stacked) - B @ self.lap_demos_stacked) 
        stretch_cost = np.linalg.norm(self.E_cvx @ self.nodes_stacked)
        bend_cost = np.linalg.norm(self.R_cvx @ self.nodes_stacked)
        
        #experimentally determined
        #stretch_mult = 1.0
        #bend_mult = 1.0
        
        stretch_mult = 0.1
        bend_mult = 0.01
        
        self.stretch_const = stretch_mult * approx_cost / stretch_cost
        self.bend_const = bend_mult * approx_cost / bend_cost
        if approx_cost <= 0:
            self.stretch_const = 1
            self.bend_const = 1
        print("Smoothing Parameters: " + str([self.stretch_const, self.bend_const]))
        return
        
    def calc_parameters(self, x, A, B):
        x_repro = cp.Variable((self.n_size, 1))
        x_repro.value = self.nodes_stacked
        objective = cp.Minimize((x[0] / self.spatial_beta)   * cp.sum_squares(A @ x_repro - B @ self.demos_stacked) 
                        + (x[1] / self.tangent_beta) * cp.sum_squares(A @ (self.E_cvx @ x_repro) - B @ self.tan_demos_stacked) 
                        + (x[2] / self.shape_beta)   * cp.sum_squares(A @ (self.R_cvx @ x_repro) - B @ self.lap_demos_stacked) 
                        + self.stretch_const * cp.sum_squares(self.E_cvx @ x_repro)
                        + self.bend_const    * cp.sum_squares(self.R_cvx @ x_repro))
        problem = cp.Problem(objective)
        problem.solve(solver=cp.OSQP, warm_start=True, verbose=False, max_iter=1000000)
        if x_repro.value is not None:
            repro = x_repro.value
            objective_val = 0
            for i in range(self.num_trajs):
                clusters_i = cluster(self.trajs[i], self.nodes)
                A_i = np.zeros((self.n_nodes*self.n_dims, self.n_nodes*self.n_dims))
                B_i = np.zeros((self.n_nodes*self.n_dims, self.n_pts*self.n_dims))
                for j in range(self.n_nodes):
                    for pt in clusters_i[j]:
                        for k in range(self.n_dims):
                            B_i[j + self.n_nodes*k, pt + self.n_pts*k] = 1 #change for variable weighting
                    for k in range(self.n_dims):
                        A_i[j + self.n_nodes*k, j + self.n_nodes*k] = np.sum(B_i[j + self.n_nodes*k, :])
                        
                objective_val = objective_val + np.linalg.norm(A_i @ self.nodes_stacked - B_i @ np.reshape(self.trajs[i], (self.n_pts * self.n_dims), order='F'))
        else:
            print("REPRO IS NONE")
            objective_val = 1e20
        return objective_val
        
    def setup_problem(self):
        self.clusters = cluster(np.vstack(self.trajs), self.nodes)
        A = np.zeros((self.n_nodes*self.n_dims, self.n_nodes*self.n_dims))
        B = np.zeros((self.n_nodes*self.n_dims, self.num_trajs*self.n_pts*self.n_dims))
        for i in range(self.n_nodes):
            for pt in self.clusters[i]:
                for j in range(self.n_dims):
                    B[i + self.n_nodes*j, pt + (self.num_trajs*self.n_pts)*j] = 1 #change for variable weighting
            for j in range(self.n_dims):
                A[i + self.n_nodes*j, i + self.n_nodes*j] = np.sum(B[i + self.n_nodes*j, :])
                
        if self.weighting_scheme == 'auto':
            # get scaling factors (betas)
            scaling_factors = self.calc_scaling_factors()
            self.spatial_beta = scaling_factors[0]
            self.tangent_beta = scaling_factors[1]
            self.shape_beta = scaling_factors[2]
            print("Betas found: " + str(scaling_factors))
            
            # get importance (alphas)
            res = minimize(self.calc_parameters, np.array([self.spatial_alpha, self.tangent_alpha, self.shape_alpha]), args=((A, B)), method='SLSQP', bounds=((0, 1), (0, 1), (0, 1)), constraints=({'type': 'eq', 'fun' : lambda x: sum(x) - 1}), options={'disp' : True})
            print("Alphas found: " + str(res.x))
            self.spatial_alpha = res.x[0]
            self.tangent_alpha = res.x[1]
            self.shape_alpha = res.x[2]
            self.calc_weights()
            self.smoothing_autotuning(A, B)
        if self.weighting_scheme == 'half-auto':
            self.smoothing_autotuning(A, B)
        
        self.x = cp.Variable((self.n_size, 1))
        self.x.value = self.nodes_stacked
        self.objective = cp.Minimize(self.spatial_const   * cp.sum_squares(A @ self.x - B @ self.demos_stacked) 
                                + self.tangent_const * cp.sum_squares(A @ (self.E_cvx @ self.x) - B @ self.tan_demos_stacked) 
                                + self.shape_const   * cp.sum_squares(A @ (self.R_cvx @ self.x) - B @ self.lap_demos_stacked) 
                                + self.stretch_const * cp.sum_squares(self.E_cvx @ self.x)
                                + self.bend_const    * cp.sum_squares(self.R_cvx @ self.x))
        return self.x
        
    def solve_problem(self, iters=10, disp=True):
        
        for i in range(iters):
            self.calc_consts()
            self.problem = cp.Problem(self.objective, self.consts)
            self.problem.solve(solver=cp.OSQP, warm_start=True, verbose=disp, max_iter=1000000)
            
            if disp:
                print("status:", self.problem.status)
                print("optimal value", self.problem.value)
                for i in range(len(self.consts)):
                    print("dual value for constraint " + str(i), ": ", self.consts[i].dual_value)
                
            self.solx = self.x
            if self.n_dims == 1:
                self.sol = self.x.value
            elif self.n_dims == 2:
                self.sol = np.hstack((self.x.value[:self.n_nodes], self.x.value[self.n_nodes:]))
            elif self.n_dims == 3:
                self.sol = np.hstack((self.x.value[:self.n_nodes], self.x.value[self.n_nodes:self.n_nodes*2], self.x.value[self.n_nodes*2:]))
            elif self.n_dims == 4:
                self.sol = np.hstack((self.x.value[:self.n_nodes], self.x.value[self.n_nodes:self.n_nodes*2], self.x.value[self.n_nodes*2:self.n_nodes*3], self.x.value[self.n_nodes*3:]))
                
            #next round with updated solution
            self.nodes = self.sol
            self.nodes_stacked = np.reshape(self.nodes, (self.n_size, 1), order='F')
            
            self.setup_problem()
        return self.sol
        
    def set_constraints(self, inds=[], csts=[]):
        self.cst_inds = inds
        self.cst_pts = csts
        
    def calc_consts(self):
        self.consts = []
        for i in range(len(self.cst_inds)):
            for j in range(self.n_dims):
                self.consts.append( cp.abs(self.x[self.cst_inds[i] + (self.n_nodes * j)][0] - self.cst_pts[i][j]) <= 0 )
    
def main():
    filenames = ['h5_files/recorded_demo 2025-02-05 12:57:17.h5', 'h5_files/recorded_demo 2025-02-05 12:58:30.h5', 'h5_files/recorded_demo 2025-02-05 12:59:46.h5', 'h5_files/recorded_demo 2025-02-05 13:01:00.h5']
    trajs = []
    for i in range(len(filenames)):
        #filename = filenames[i]
        #joint_data, tf_data, wrench_data = read_data(filename)
        #traj = tf_data[1]
        #traj_ds, inds = DouglasPeuckerPoints2(traj, 1000)
        #time = tf_data[0][:, 0] + tf_data[0][:, 1] * (10.0**-9)
        #time_ds = time[inds].reshape((1000, 1))
        #traj_with_time = np.hstack((time_ds, traj_ds))
        #print(np.shape(traj_ds))
        #trajs.append(traj_with_time)
        #np.savetxt('traj_time_ds' + str(i+1) + '.txt', traj_with_time)
        traj = np.loadtxt('traj_time_ds' + str(i+1) + '.txt')
        trajs.append(traj)
        
    cst_inds = [0, 99]
    cst_pts = [trajs[0][0, :], trajs[0][-1, :]]

    mc = MC_elmap(trajs, weighting='custom', spatial=1.0, tangent=1.0, shape=1.0, stretch=100.0, bend=100.0, n=100, init_nodes=None)
    x = mc.setup_problem()
    mc.set_constraints(inds=cst_inds, csts=cst_pts)

    repro = mc.solve_problem(iters=1)
    
    np.savetxt('mc_elmap_repro.txt', repro[:, 1:])
    #repro = np.loadtxt('mc_elmap_repro.txt')
    
    plt.figure()
    ax = plt.axes(projection='3d')
    for traj in trajs:
        ax.plot3D(traj[:, 1], traj[:, 2], traj[:, 3], 'k')
    ax.plot3D(repro[:, 1], repro[:, 2], repro[:, 3], 'r')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()
    
    
if __name__ == '__main__':
    main()
