#!/usr/bin/env python

# reference: http://krasserm.github.io/2018/03/19/gaussian-processes/

import numpy as np
from scipy.stats import norm
from gp import RBF_kernel, GP
from sklearn.preprocessing import normalize


class MixtureGaussianProcess:
    def __init__(self, num_gp=3, gps = [GP() for i in range(3)], gating_gps = [GP() for i in range(3)], epsilon=0.05, max_iter=100):
        self.num_gp = num_gp
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
            R[:, i] = norm(loc=pred_mean[:, i], scale=np.std(pred_var[:, i])).pdf(Y_train)
        P = P * R + 1e-6
        P = P / np.sum(P, axis = 1)[:,np.newaxis]
        return P
    
    def Maximization(self, X_train, Y_train, P):
        pred_mean = np.zeros((len(Y_train), self.num_gp))
        pred_var = np.zeros_like(pred_mean)
        for i in range(self.num_gp):
            pred_mean[:, i], pred_var[:, i] = self.gps[i].PosteriorPredict(X_test=X_train, X_train=X_train, Y_train=Y_train, P=P[:, i])
        return pred_mean, pred_var

    def Optimizate(self, X_train, Y_train, P):
        for i in range(self.num_gp):
            self.gps[i].OptimizeKernel(X_train=X_train, Y_train=Y_train, p=P[:,[i]])
    
    def EMOptimize(self, optimize_kernel = False):
        for i in range(self.max_iter):
            prev_P = self.P
            pred_mean, pred_var = self.Maximization(self.X_train, self.Y_train, self.P)
            self.P = self.Expectation(pred_mean, pred_var, self.Y_train, self.P)
            diff_P = np.abs(self.P - prev_P)
            if (diff_P.max() <= self.epsilon):
                break
        if optimize_kernel == True:
            self.Optimizate(self.X_train, self.Y_train, self.P)
        pred_mean = pred_mean * self.P
        pred_var = pred_var * self.P
        return pred_mean.sum(axis=1), pred_var.sum(axis=1), self.P

    def AddSample(self, X_train, Y_train):
        if self.X_train is None:
            self.X_train = np.asarray(X_train)
            self.Y_train = np.asarray(Y_train)
            self.P = np.random.random((len(Y_train), self.num_gp))
            self.P = self.P / np.sum(self.P, axis = 1)[:,np.newaxis]
        else:
            self.X_train = np.concatenate((self.X_train, X_train), axis=0)
            self.Y_train = np.concatenate((self.Y_train, Y_train)).reshape(-1)
            new_P = np.random.random((len(Y_train), self.num_gp))
            new_P = new_P / np.sum(new_P, axis = 1)[:,np.newaxis]
            self.P = np.concatenate((self.P, new_P), axis=0)
        # self.X_train, I = np.unique(self.X_train, axis=0, return_index=True)
        # self.Y_train = self.Y_train[I]
        # self.P = self.P[I,:]
    
    def FitGatingFunction(self, X_train, P):
        for i in range(self.num_gp):
            self.gating_gps[i].OptimizeKernel( X_train=X_train, Y_train=P[:, [i]])
    
    def PredictGatingFunction(self, X_test, X_train, P):
        P_prediction = np.zeros((X_test.shape[0], self.num_gp))
        for i in range(self.num_gp):
            P_prediction[:, [i]], _ = self.gating_gps[i].PosteriorPredict(X_test, X_train=X_train, Y_train=P[:, [i]])
        return P_prediction
    
    def OptimizeModel(self, optimize_kernel = False):
        _, _, P = self.EMOptimize(optimize_kernel)
        if optimize_kernel == True:
            self.FitGatingFunction(self.X_train, P)
    
    def Predict(self, X_test, X_train=None, Y_train=None):
        if X_train is None:
            X_train = self.X_train
            Y_train = self.Y_train
        P = self.PredictGatingFunction(X_test, self.X_train, self.P)
        pred_mean = np.zeros((X_test.shape[0], self.num_gp))
        pred_var = np.zeros_like(pred_mean)
        for i in range(self.num_gp):
            pred_mean[:, i], pred_var[:, i] = self.gps[i].PosteriorPredict(X_test=X_test, X_train=X_train, Y_train=Y_train, P=self.P[:, i])
        mean = pred_mean * P
        var = pred_var * P
        return mean.sum(axis=1), var.sum(axis=1)