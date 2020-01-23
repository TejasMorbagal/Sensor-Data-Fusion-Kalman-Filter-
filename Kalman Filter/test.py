# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 19:09:29 2020

@author: Tejas
"""
import numpy as np

def rts_smoother(Xs, Ps, F, Q):
    n, dim_x, _ = Xs.shape
    
    # smoother gain
    K = np.zeros((n,dim_x, dim_x))
    x, P = Xs.copy(), Ps.copy()

    for k in range(n-2,-1,-1):
        P_pred = np.dot(np.dot(F, P[k]),(F.T)) + Q

        K[k]  = np.dot(np.dot(P[k], F.T),(np.inv(P_pred)))
        x[k] += np.dot(np.dot(K[k], (x[k+1] - F), x[k]))
        P[k] += np.dot(np.dot(K[k], P[k+1] - P_pred)(K[k].T))
    return (x, P, K)

#import numpy as np
from numpy import random
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
#from kf_book import book_plots as book_plots
#import book_plots as bp

#from code.book_plots import interactive_plo

def plot_rts(noise, Q=0.001, show_velocity=False):
    random.seed(123)
    fk = KalmanFilter(dim_x=2, dim_z=1)

    fk.x = np.array([0., 1.])      # initial state (location and velocity)

    fk.F = np.array([[1., 1.],
                     [0., 1.]])    # state transition matrix

    fk.H = np.array([[1., 0.]])    # Measurement function
    fk.P = 10.                     # covariance matrix
    fk.R = noise                   # state uncertainty
    fk.Q = Q                       # process uncertainty

    # create noisy data
    zs = np.asarray([t + random.randn()*noise for t in range (20)])

    # filter data with Kalman filter, than run smoother on it
    mu, cov, _, _ = fk.batch_filter(zs)
    M,P,C,D = fk.rts_smoother(mu, cov)

    # plot data
    if show_velocity:
        index = 1
        print('gu')
    else:
        index = 0
    if not show_velocity:
        #bp.plot_measurements(zs, lw=1)
        print("if")
    plt.plot(M[:, index], c='b', label='RTS')
    plt.plot(mu[:, index], c='g', linestyle='--', label='KF output')
    if not show_velocity:
        plt.plot([0, len(zs)], [0, len(zs)], 'k', linewidth=2, label='track') 
    plt.legend(loc=4)
    plt.show()
    
plot_rts(7.)