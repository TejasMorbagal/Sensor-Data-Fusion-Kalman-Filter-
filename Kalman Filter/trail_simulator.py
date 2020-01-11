# -*- coding: utf-8 -*-
"""
Created on Mon Jan  6 18:28:19 2020

@author: Tejas
"""

import numpy as np
import matplotlib.pyplot as plt

    
    

class MotionModel():
    def __init__(self, A, Q):
        self.A = A
        self.Q = Q

        (m, _) = Q.shape
        self.zero_mean = np.zeros(m)

    def __call__(self, x):
        new_state = self.A @ x + np.random.multivariate_normal(self.zero_mean, self.Q)
        return new_state


class MeasurementModel():
    def __init__(self, H, R):
        self.H = H
        self.R = R

        (n, _) = R.shape
        self.zero_mean = np.zeros(n)

    def __call__(self, x):
        measurement = self.H @ x + np.random.multivariate_normal(self.zero_mean, self.R)
        return measurement


def create_model_parameters(T=2, s2_x=0.1 ** 2, s2_y=0.1 ** 2, lambda2=0.3 ** 2):
    # Motion model parameters
    F = np.array([[1, T],
                  [0, 1]])
    base_sigma = np.array([[T ** 3 / 3, T ** 2 / 2],
                           [T ** 2 / 2, T]])

    sigma_x = s2_x * base_sigma
    sigma_y = s2_y * base_sigma

    zeros_2 = np.zeros((2, 2))
    # A = F
    A = np.block([[F, zeros_2],
                  [zeros_2, F]])
    #Q = D
    Q = np.block([[sigma_x, zeros_2],
                  [zeros_2, sigma_y]])

    # Measurement model parameters
    H = np.array([[1, 0, 0, 0],
                  [0, 0, 1, 0]])
    R = lambda2 * np.eye(2)

    return A, H, Q, R


def simulate_system(K, x0):
    (A, H, Q, R) = create_model_parameters()

    # Create models
    motion_model = MotionModel(A, Q)
    meas_model = MeasurementModel(H, R)

    (m, _) = Q.shape
    (n, _) = R.shape

    state = np.zeros((K, m))
    meas = np.zeros((K, n))

    # initial state
    x = x0
    for k in range(K):
        x = motion_model(x)
        z = meas_model(x)

        state[k, :] = x
        meas[k, :] = z

    return state, meas


#import numpy as np


class KalmanFilter():
    def __init__(self, A, H, Q, R, x_0, P_0):
        # Model parameters
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R

        # Initial state
        self._x = x_0
        self._P = P_0

    def predict(self):
        self._x = self.A @ self._x
        self._P = self.A @ self._P @ self.A.transpose() + self.Q

    def update(self, z):
        self.S = self.H @ self._P @ self.H.transpose() + self.R
        self.V = z - self.H @ self._x
        self.K = self._P @ self.H.transpose() @ np.linalg.inv(self.S)

        self._x = self._x + self.K @ self.V
        self._P = self._P - self.K @ self.S @ self.K.transpose()

    def get_state(self):
        return self._x, self._P


#import numpy as np
#import matplotlib.pyplot as plt

#from kalman_filter import KalmanFilter
#from simulate_model import simulate_system, create_model_parameters

np.random.seed(21)
(A, H, Q, R) = create_model_parameters()
K = 20
# initial state
x = np.array([0, 0.1, 0, 0.1])
P = 0 * np.eye(4)

(state, meas) = simulate_system(K, x)
kalman_filter = KalmanFilter(A, H, Q, R, x, P)

est_state = np.zeros((K, 4))
est_cov = np.zeros((K, 4, 4))

for k in range(K):
    kalman_filter.predict()
    kalman_filter.update(meas[k, :])
    (x, P) = kalman_filter.get_state()

    est_state[k, :] = x
    est_cov[k, ...] = P

plt.figure(figsize=(7, 5))
plt.plot(state[:, 0], state[:, 2], '-bo')
plt.plot(est_state[:, 0], est_state[:, 2], '-y')
plt.plot(meas[:, 0], meas[:, 1], ':rx')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend(['true state', 'inferred state', 'observed measurement'])
plt.axis('square')
plt.tight_layout(pad=0)
plt.plot()