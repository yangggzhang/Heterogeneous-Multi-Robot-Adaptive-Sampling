#!/usr/bin/env python

# reference: http://krasserm.github.io/2018/03/19/gaussian-processes/


import numpy as np
import rospkg
import rospy
from numpy.linalg import inv
from numpy.linalg import cholesky, det, lstsq
from scipy.optimize import minimize
import operator

class RBF_kernel:
    def __init__(self, l=0.5, sigma_f=0.5):
        self.l = l
        self.sigma_f = sigma_f
    
    def Compute(self, X1, X2):
        sqdist = np.sum(X1**2, 1).reshape(-1, 1) + np.sum(X2**2, 1) - 2 * np.dot(X1, X2.T)
        return self.sigma_f ** 2 * np.exp(-0.5 / self.l ** 2 * sqdist)
    
    def UpdateKernel(self, l, sigma_f):
        self.l = l
        self.sigma_f = sigma_f

    def ComputeKernel(self, X1, X2, l, sigma_f):
        sqdist = np.sum(X1**2, 1).reshape(-1, 1) + np.sum(X2**2, 1) - 2 * np.dot(X1, X2.T)
        return sigma_f ** 2 * np.exp(-0.5 / l ** 2 * sqdist)
    
    def GetHyperparam(self):
        return self.l, self.sigma_f

class GP:
    def __init__(self, l=0.5, sigma_f=0.5, sigma_y=0.1):
        self.kernel = RBF_kernel(l, sigma_f)
        self.sigma_y = sigma_y
        self.X_train = None
        self.Y_train = None
    
    def UpdateData(self, X_train, Y_train):
        self.X_train = X_train
        self.Y_train = Y_train
    
    def PosteriorPredict(self, X_s, X_train=None, Y_train=None, P= None):
        '''  
        Computes the suffifient statistics of the GP posterior predictive distribution 
        from m training data X_train and Y_train and n new inputs X_s.
        
        Args:
            X_s: New input locations (n x d).
            X_train: Training locations (m x d).
            Y_train: Training targets (m x 1).
            l: Kernel length parameter.
            sigma_f: Kernel vertical variation parameter.
            sigma_y: Noise parameter.
        
        Returns:
            Posterior mean vector (n x d) and covariance matrix (n x n).
        '''
        if X_train is not None:
            self.UpdateData(X_train, Y_train)

        K = self.kernel.Compute(self.X_train, self.X_train) + self.sigma_y**2 * np.eye(len(self.X_train))
        K_s = self.kernel.Compute(self.X_train, X_s)
        K_ss = self.kernel.Compute(X_s, X_s) + 1e-8 * np.eye(len(X_s))
        if P is not None:
            K_ss += self.sigma_y**2 * np.diag(1/P)
        K_inv = inv(K)
        
        mu_s = K_s.T.dot(K_inv).dot(self.Y_train)

        cov_s = K_ss - K_s.T.dot(K_inv).dot(K_s)
        
        return mu_s, np.diag(cov_s)
    
    def OptimizeKernel(self, noise, X_train=None, Y_train=None, p = None):
        '''
        Returns a function that Computes the negative log marginal
        likelihood for training data X_train and Y_train and given 
        noise level.
        
        Args:
            X_train: training locations (m x d).
            Y_train: training targets (m x 1).
            noise: known noise level of Y_train. 
        '''
        if X_train is not None:
            self.UpdateData(X_train, Y_train)
        def nnl_stable(theta):
            K = self.kernel.ComputeKernel(X_train, X_train, l=theta[0], sigma_f=theta[1]) +  noise**2 * np.eye(len(Y_train))
            L = cholesky(K)
            return np.sum(np.log(np.diagonal(L))) + \
                0.5 * Y_train.T.dot(lstsq(L.T, lstsq(L, Y_train,rcond=None)[0],rcond=None)[0]) + \
                0.5 * len(X_train) * np.log(2*np.pi)
        l_init, sigma_f_init = self.kernel.GetHyperparam()

        res = minimize(nnl_stable, [l_init, sigma_f_init], method='L-BFGS-B')
        updated_l, updated_sigma_f = res.x
        self.kernel.UpdateKernel(updated_l, updated_sigma_f)    

# from gp_util import plot_gp

# # Finite number of points
# X = np.arange(-5, 5, 0.2).reshape(-1, 1)

# test_gp = GP(kernel=RBF_kernel(l=1.0, sigma_f=1.0), sigma_y=3)
# noise = 0.4

# # Noisy training data
# X_train = np.arange(-3, 4, 1).reshape(-1, 1)
# Y_train = np.sin(X_train) + noise * np.random.randn(*X_train.shape)



# test_gp.OptimizeKernel(X_train, Y_train, noise)

# res = minimize(nll_fn(X_train, Y_train, noise), [1, 1], 
#                bounds=((1e-5, None), (1e-5, None)),
#                method='L-BFGS-B')

# mu_s, cov_s = test_gp.PosteriorPredict(X, X_train, Y_train)
# samples = np.random.multivariate_normal(mu_s.ravel(), cov_s, 3)
# plot_gp(mu_s, cov_s, X, X_train=X_train, Y_train=Y_train, samples=samples)
