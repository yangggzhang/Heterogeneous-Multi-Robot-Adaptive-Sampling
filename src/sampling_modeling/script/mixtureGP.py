#!/usr/bin/env python
import numpy as np
from scipy.stats import norm
from gp import RBF_kernel, GP

class MixtureGP:
    def __init__(self, num_gp=3, noise=0.1):
        self.num_gp = num_gp
        self.noise = noise
        self.gps = [GP() for i in range(num_gp)]
    
    def expectation(self, pred_mean, pred_var, Y_train, P):
        R = np.zeros(pred_mean.shape[1], self.num_gp))
        for i in range(self.num_gp):
            R[:, i] = norm(loc=pred_mean[:, i], scale=pred_var[:, i]).pdf(Y_train)
        P = P * R
        P = P / P.sum(axis=1, dtype='float')
        return P
    
    def maximization(self, X_train, Y_train, P):
        pred_mean = np.zeros((len(Y_train), self.num_gp))
        pred_var = np.zeros((len(Y_train), self.num_gp))
        for i in range(self.num_gp):
            pred_mean[:, i], pred_var[:, i] = self.gps[i].posterior_predictive(X_s=X_train, X_train=X_train, Y_train=Y_train, P=P[:, i])
        return pred_mean, pred_var

    def optimizate(self, X_train, Y_train, noise, P):
        for i in range(self.num_gp):
            self.gps[i].optimize_kernel(X_train=X_train, Y_train=Y_train, noise=noise, p=P[i,:])
    
    def EM_optimize(self, X_train, Y_train):
        P = np.random.random((len(Y_train), self.num_gp))
        while 1:
            pred_mean, pred_var = self.maximization(X_train, Y_train, P)
            P = self.expectation(pred_mean, pred_var, Y_train, P)
            self.optimizate(X_train, Y_train, self.noise, P)
        return pred_mean, pred_var

