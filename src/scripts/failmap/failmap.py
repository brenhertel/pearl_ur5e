import numpy as np
import matplotlib.pyplot as plt
from downsampling import *
from utils import *

class FailMap(object):

    def __init__(self, demos, n=100, downsampling="naive", weights=None, init_lambda=1.0, init_mu=1.0, c_lambda=1.1, c_mu=1.1):
        self.demos = demos
        self.data = np.vstack(demos)
        self.n_pts, self.n_dims = np.shape(self.data)
        self.n_nodes = n
        
        #set up weights
        self.weights = np.ones((self.n_pts,))
        if weights is not None:
            self.weights = weights
            
        #if all weights negative, explore 
        if np.all(self.weights <= 0):
            self.negative_exploration()
           
        #set up initial map
        self.nodes = downsample_traj(self.demos[0], self.n_nodes)
        if downsampling == 'distance':
            self.set_nodes(db_downsample(self.demos[0], self.n_nodes))
        if downsampling == 'douglaspeucker':
            self.set_nodes(DouglasPeuckerPoints(self.demos[0], self.n_nodes))
            
        #set up later for A, E, R matrices for optimization
        self.lmdas = init_lambda * np.ones((self.n_nodes,))
        self.mus = init_mu * np.ones((self.n_nodes,))
        self.c_lmda = c_lambda
        self.c_mu = c_mu
            
        self.weight_sum = np.sum(np.abs(self.weights))
        
        self.E = np.zeros((self.n_nodes, self.n_nodes))
        self.R = np.zeros((self.n_nodes, self.n_nodes))
        self.A = np.zeros((self.n_nodes, self.n_nodes))
        
        #change to see output of optimization
        self.DEBUG = False

    def set_weights(self, new_weights):
        self.weights = new_weights
        
    def set_nodes(self, new_nodes):
        self.nodes = new_nodes
        
    def negative_exploration(self):
        print('Only negative examples giving, using exploration')
        new_data = np.copy(self.data)
        new_weights = np.copy(self.weights)
        crv_len = curve_length(self.data)
        noise = np.random.normal(loc=0.0, scale=(crv_len / self.n_pts), size=(self.n_pts, self.n_dims))
        new_data += noise
        self.data = np.vstack((self.data, new_data))
        self.n_pts, self.n_dims = np.shape(self.data)
        self.weights = np.hstack((self.weights, new_weights * -0.5))
        print(self.weights)
        
    #cluster nodes (Expectation step)
    def associate(self, consts, inds):
        self.C = np.zeros((self.n_nodes, self.n_dims))
        self.W = np.zeros((self.n_nodes,))
        self.K = np.zeros((self.n_nodes,))
        
        for i in range(len(inds)):
            self.nodes[inds[i]] = np.array(consts[i])
            
        #association & partial C matrix calculation
        
        #self.clusters = np.array([-1 for _ in range(self.n_pts)])
        #for i in range(self.n_pts):
        #    self.clusters[i] = np.argmin([np.linalg.norm(self.data[i] - self.nodes[j]) for j in range(self.n_nodes)])
        #    self.C[self.clusters[i]] = self.C[self.clusters[i]] + self.weights[i] * self.data[i]
        
        #alternative faster version, less readable:
        dist = (np.sum(self.data ** 2, axis=1,keepdims=1)+np.sum(self.nodes**2, axis=1,keepdims=1).T) - 2 * (self.data @ self.nodes.T) 
        self.clusters = np.argmin(dist, axis=1) 
        
        for i in range(self.n_pts):
            self.C[self.clusters[i]] = self.C[self.clusters[i]] + self.weights[i] * self.data[i]
            self.W[self.clusters[i]] = self.W[self.clusters[i]] + self.weights[i]
            self.K[self.clusters[i]] = self.K[self.clusters[i]] + 1
        
        return self.clusters
    
    def calc_E(self):
        self.E = np.zeros((self.n_nodes, self.n_nodes))
        for i in range(self.n_nodes-1):
            self.E[i, i] += self.lmdas[i]
            self.E[i+1, i+1] += self.lmdas[i]
            self.E[i, i+1] -= self.lmdas[i]
            self.E[i+1, i] -= self.lmdas[i]        
        return
        
    def calc_R(self):
        self.R = np.zeros((self.n_nodes, self.n_nodes))
        for i in range(self.n_nodes-2):
            self.R[i+1, i+1] += self.mus[i]
            self.R[i, i] += (self.mus[i] / 4)
            self.R[i+2, i+2] += (self.mus[i] / 4)
            self.R[i, i+2] += (self.mus[i] / 4)
            self.R[i+2, i] += (self.mus[i] / 4)
            self.R[i, i+1] -= (self.mus[i] / 2)
            self.R[i+1, i] -= (self.mus[i] / 2)
            self.R[i+2, i+1] -= (self.mus[i] / 2)
            self.R[i+1, i+2] -= (self.mus[i] / 2)
        return
    
    #calculate A matrix for optimization
    def calc_A(self):
        self.calc_E()
        self.calc_R()
        #print(self.E)
        #print(self.R)
        self.A = self.E + self.R
        for i in range(self.n_nodes):
            self.A[i, i] = self.A[i, i] + (np.sum(self.weights[self.clusters == i]) / self.weight_sum)
        return self.A
        
    def tighten_parameters(self):
        for i in range(self.n_nodes-1):
            if self.K[i] > 0 and i not in self.const_inds and i+1 not in self.const_inds:
                self.lmdas[i] = self.c_lmda * (self.lmdas[i] - (1 / self.weight_sum) * ( (self.W[i] - self.W[i+1]) / (self.K[i] + self.K[i+1]) ) )
            else:
                self.lmdas[i] = self.c_lmda * self.lmdas[i]
        for i in range(self.n_nodes-2):
            if self.K[i] > 0 and i not in self.const_inds and i+1 not in self.const_inds and i+2 not in self.const_inds:
                self.mus[i] = self.c_mu * (self.mus[i] - (1 / self.weight_sum) * ( (self.W[i] - self.W[i+1] + self.W[i+2]) / (self.K[i] + self.K[i+1] + self.K[i+2]) ) )
            else:
                self.mus[i] = self.c_mu * self.mus[i]
        return
        
    #incorporate any given constraints by giving high weight to those data points
    def add_consts(self, consts=[], inds=[]):
        for i in range(len(inds)):
            self.nodes[inds[i]] = np.array(consts[i])
            self.data = np.vstack((self.data, np.array(consts[i]).reshape((1, self.n_dims))))
            self.weights = np.append(self.weights, np.array([np.size(self.data)]))
        self.weight_sum = np.sum(np.abs(self.weights))
        self.n_pts, self.n_dims = np.shape(self.data)
        return
        
    #add consts directly to C and A matrices -- avoid confusion in weight tuning
    def add_consts_new(self, consts=[], inds=[]):
        for i in range(len(inds)):
            self.nodes[inds[i]] = np.array(consts[i])
            self.C[inds[i]] = self.C[inds[i]] + np.size(self.data)**2 * np.array(consts[i])
            #self.A[inds[i], inds[i]] = self.A[inds[i], inds[i]] + (np.size(self.data)**2 / (self.weight_sum + len(inds)*np.size(self.data)**2) )
        return        
    
    #optimize elastic map, return solution
    def optimize_map(self, consts=[], inds=[]):
        self.const_inds = inds
        self.add_consts(consts, inds)
    
        # EXPECTATION
        clusters = self.associate(consts, inds)
        C = self.C / self.weight_sum
        
        # MAXIMIZATION
        self.calc_A()
        #self.add_consts_new(consts, inds)
        self.nodes = np.linalg.lstsq(self.A, C, rcond=None)[0]
        for i in range(len(inds)):
            self.nodes[inds[i]] = np.array(consts[i])
        
        # TIGHTENING
        self.tighten_parameters()
        
        # REPEAT
        new_clusters = self.associate(consts, inds)
        C = self.C / self.weight_sum #must recalculate C every time
        
        iter = 1
        if self.DEBUG:
            print(iter, self.calc_nrg())
        
        #repeat EM until convergence or certain number of iterations
        while (np.any(new_clusters != clusters)) and (iter < 50):
            clusters = new_clusters
            
            self.calc_A()
            #self.add_consts_new(consts, inds)
            #print('A')
            #print(self.A)
            #print('C')
            #print(C)
            self.nodes = np.linalg.lstsq(self.A, C, rcond=None)[0]
            for i in range(len(inds)):
                #print(inds[i], consts[i])
                self.nodes[inds[i]] = np.array(consts[i])
        
            self.tighten_parameters()
            
            new_clusters = self.associate(consts, inds)
            C = self.C / self.weight_sum
            
            iter = iter + 1
            if self.DEBUG:
                print(iter, self.calc_nrg())
            
        return self.nodes

    def calc_nrg(self): #Note: This is not for the optimization, just utility
        Uy = 0.
        for i in range(self.n_pts):
            Uy = Uy + self.weights[i] * np.linalg.norm(self.data[i] - self.nodes[self.clusters[i]])**2
            
        Ue = 0.
        for i in range(self.n_nodes - 1):
            Ue = Ue + np.linalg.norm(self.nodes[i] - self.nodes[i+1])**2
            
        Ur = 0.
        for i in range(self.n_nodes - 2):
            Ur = Ur + np.linalg.norm(self.nodes[i] - 2*self.nodes[i+1] + self.nodes[i+2])**2
            
        return (Uy / self.weight_sum) + (self.lmda * Ue) + (self.mu * Ur)
        
    def plot(self):
        if self.n_dims != 2:
            print('Plotting not implemented!')
            return
        fig, ax = plt.subplots()
        lines = colored_line(self.data[:-len(self.const_inds), 0], self.data[:-len(self.const_inds), 1], c=self.weights[:-len(self.const_inds)], ax=ax, cmap=plt.get_cmap("RdYlGn"), lw=4)
        plt.plot(self.nodes[:, 0], self.nodes[:, 1], 'tab:orange', lw=4)
        fig.colorbar(lines)
        plt.show()
        

def main2D():
    # demonstration
    num_points = 1000
    t = np.linspace(0, 10, num_points).reshape((num_points, 1))
    x_demo = np.sin(t) + 0.01 * t**2 - 0.05 * (t-5)**2
    y_demo = np.cos(t) - 0.01 * t - 0.03 * t**2
    
    traj = np.hstack((x_demo, y_demo))
    
    elmap1 = FailMap([traj], n=100, weights=None, downsampling='distance', init_lambda=0.01, init_mu=0.01, c_lambda=0.99, c_mu=0.99)
    new_traj = elmap1.optimize_map([np.array([x_demo[0], y_demo[0]]).flatten(), np.array([x_demo[-1], y_demo[-1]]).flatten()], [0, 100-1])
    
    elmap2 = FailMap([traj], n=100, weights=None, downsampling='distance', init_lambda=0.01, init_mu=0.01, c_lambda=0.99, c_mu=0.99)
    new_traj2 = elmap2.optimize_map([np.array([x_demo[0]+0.5, y_demo[0]-0.2]).flatten(), np.array([x_demo[-1], y_demo[-1]]).flatten()], [0, 100-1])
    
    elmap3 = FailMap([traj], n=100, weights=None, downsampling='distance', init_lambda=0.01, init_mu=0.01, c_lambda=0.99, c_mu=0.99)
    new_traj3 = elmap3.optimize_map([np.array([x_demo[0]+0.5, y_demo[0]-0.2]).flatten(), np.array([x_demo[400]-0.1, y_demo[400]-0.3]).flatten(), np.array([x_demo[-1], y_demo[-1]+0.2]).flatten()], [0, 40, 100-1])
    
    plt.rcParams['figure.figsize'] = (6.5, 6.5)
    fig, axs = plt.subplots(2, 2)
    axs[0][0].set_title('Demonstration')
    axs[1][0].set_title('Same Constraints')
    axs[0][1].set_title('New Initial Point')
    axs[1][1].set_title('New Initial, Final and Viapoint')
    axs[0][0].plot(traj[:, 0], traj[:, 1], 'k', lw=3)
    axs[1][0].plot(traj[:, 0], traj[:, 1], 'k', lw=3)
    axs[0][1].plot(traj[:, 0], traj[:, 1], 'k', lw=3)
    axs[1][1].plot(traj[:, 0], traj[:, 1], 'k', lw=3)
    
    axs[1][0].plot(new_traj[:, 0], new_traj[:, 1], 'm', lw=3)
    axs[1][0].plot(x_demo[0], y_demo[0], 'k.', ms=10)
    axs[1][0].plot(x_demo[-1], y_demo[-1], 'k.', ms=10)
    axs[1][0].plot(x_demo[0], y_demo[0], 'rx', ms=10, mew=2)
    axs[1][0].plot(x_demo[-1], y_demo[-1], 'rx', ms=10, mew=2)
    
    axs[0][1].plot(new_traj2[:, 0], new_traj2[:, 1], 'g', lw=3)
    axs[0][1].plot(x_demo[0], y_demo[0], 'k.', ms=10)
    axs[0][1].plot(x_demo[-1], y_demo[-1], 'k.', ms=10)
    axs[0][1].plot(x_demo[0]+0.5, y_demo[0]-0.2, 'rx', ms=10, mew=2)
    axs[0][1].plot(x_demo[-1], y_demo[-1], 'rx', ms=10, mew=2)
    
    axs[1][1].plot(new_traj3[:, 0], new_traj3[:, 1], 'b', lw=3)
    axs[1][1].plot(x_demo[0], y_demo[0], 'k.', ms=10)
    axs[1][1].plot(x_demo[400], y_demo[400], 'k.', ms=10)
    axs[1][1].plot(x_demo[-1], y_demo[-1], 'k.', ms=10)
    axs[1][1].plot(x_demo[0]+0.5, y_demo[0]-0.2, 'rx', ms=10, mew=2)
    axs[1][1].plot(x_demo[400]-0.1, y_demo[400]-0.3, 'rx', ms=10, mew=2)
    axs[1][1].plot(x_demo[-1], y_demo[-1]+0.2, 'rx', ms=10, mew=2)
    
    plt.show()
    
if __name__ == '__main__':
    main2D()