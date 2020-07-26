#!/usr/bin/env python

# reference: http://krasserm.github.io/2018/03/19/gaussian-processes/

import numpy as np
from scipy.stats import norm
from gp import RBF_kernel, GP

class MixtureGaussianProcess:
    def __init__(self, num_gp=3, gps = [GP() for i in range(3)], gating_gps = [GP() for i in range(3)], noise=0.1, epsilon=0.05, max_iter=100):
        self.num_gp = num_gp
        self.noise = noise
        self.gps = gps
        self.gating_gps = gating_gps
        self.X_train = None
        self.Y_train = None
        self.P = None
        self.epsilon= epsilon
        self.max_iter=max_iter
    
    def Expectation(self, pred_mean, pred_var, Y_train, P):
        R = np.zeros((len(Y_train), self.num_gp))
        for i in range(self.num_gp):
            R[:, i] = norm(loc=pred_mean[:, i], scale=pred_var[:, i]).pdf(Y_train[:,0])
        P = P * R
        P = P / P.sum(axis=1, dtype='float')[:,None]
        P = P + 1e-6
        return P
    
    def Maximization(self, X_train, Y_train, P):
        pred_mean = np.zeros((len(Y_train), self.num_gp))
        pred_var = np.zeros_like(pred_mean)
        for i in range(self.num_gp):
            pred_mean[:, [i]], pred_var[:, i] = self.gps[i].PosteriorPredict(X_s=X_train, X_train=X_train, Y_train=Y_train, P=P[:, i])
        return pred_mean, pred_var

    def Optimizate(self, X_train, Y_train, noise, P):
        for i in range(self.num_gp):
            self.gps[i].OptimizeKernel(noise=noise, X_train=X_train, Y_train=Y_train, p=P[:,[i]])
    
    def EMOptimize(self):
        for i in range(self.max_iter):
            prev_P = self.P
            pred_mean, pred_var = self.Maximization(self.X_train, self.Y_train, self.P)
            self.P = self.Expectation(pred_mean, pred_var, self.Y_train, self.P)
            self.Optimizate(self.X_train, self.Y_train, self.noise, self.P)
            diff_P = np.abs(self.P - prev_P)
            if (diff_P.max() <= self.epsilon):
                break
        pred_mean = pred_mean * self.P
        pred_var = pred_var * self.P
        return pred_mean.sum(axis=1), pred_var.sum(axis=1), self.P

    def AddSample(self, X_train, Y_train):
        if self.X_train is None:
            self.X_train = X_train
            self.Y_train = Y_train
            self.P = np.random.random((len(Y_train), self.num_gp))
            self.P = self.P / self.P.sum(axis=1, dtype='float')[:,None]
        else:
            self.X_train = np.v_stack((self.X_train, X_train))
            self.Y_train = np.v_stack((self.Y_train, Y_train))
            new_P = np.random.random((len(Y_train), self.num_gp))
            new_P = new_P / new_P.sum(axis=1, dtype='float')[:,None]
            self.P = np.v_stack((self.P, new_P))
    
    def FitGatingFunction(self, X_train, P):
        for i in range(self.num_gp):
            self.gating_gps[i].OptimizeKernel(noise=0.0, X_train=X_train, Y_train=P[:, [i]])
    
    def PredictGatingFunction(self, X_test):
        P = np.zeros((X_test.shape[0], self.num_gp))
        for i in range(self.num_gp):
            p, _ = self.gating_gps[i].PosteriorPredict(X_test)
            P[:, [i]] = p
        return P
    
    def OptimizeModel(self):
        _, _, P = self.EMOptimize()
        self.FitGatingFunction(self.X_train, P)
    
    def Predict(self, X_test, X_train=None, Y_train=None):
        if X_train is None:
            X_train = self.X_train
            Y_train = self.Y_train
        P = self.PredictGatingFunction(X_test)
        pred_mean = np.zeros((X_test.shape[0], self.num_gp))
        pred_var = np.zeros_like(pred_mean)
        for i in range(self.num_gp):
            pred_mean[:, [i]], pred_var[:, i] = self.gps[i].PosteriorPredict(X_s=X_test, X_train=X_train, Y_train=Y_train, P=P[:, i])
        mean = pred_mean * P
        var = pred_mean * P
        return mean.sum(axis=1), var.sum(axis=1)

# from gp_util import plot_gp

# # Finite number of points
# X = np.arange(-5, 5, 0.2).reshape(-1, 1)

# test_gp = mixture_gp()
# noise = 0.4

# # Noisy training data
# X_train = np.arange(-3, 4, 1).reshape(-1, 1)
# Y_train = np.sin(X_train) + noise * np.random.randn(*X_train.shape)
# test_gp.AddSample(X_train, Y_train)
# mu_s, var_s, P = test_gp.EMOptimize()
# test_gp.FitGatingFunction(X_train, P)
# pred_P = test_gp.PredictGatingFunction(X_train)
