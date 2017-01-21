# Sensors : GPS, IMU, Wheel Encoder
# GPS --> Absolute Position, mapped to lat/long

import numpy as np
from abc import ABCMeta, abstractmethod
from sensor import GPS, IMU, Encoder
from utility import *

class PoseEKF(object):
    def __init__(self, n, m, x = None, pqr_init = (0.1, 1e-4, 0.1)):

        # Initialization values
        p,q,r = pqr_init 

        if x == None:
            self.x = np.zeros((n,1))
        if P == None:
            # p initialzed to covariance of 9999 ... arbitrary :/
            self.P = np.eye(n) * p

        self.Q = np.eye(n) * q # Process Noise Model -- depends on robot locomotion accuracy
        self.R = np.eye(m) * r # Measurement Noise Model -- depends on sensor precision
 
    def predict(self,u):
        """
        predict(x) : x = state vector
        """

        # Alias names to save typing
        x,P,Q,R = self.x, self.P, self.Q, self.R

        self.x = self.f(x,u) # dot(B,u), but not used here
        self.P = dot(F,P,F.T) + Q #
        return self.x

    def update(self, z):
        """
        update(z) : z = observations
        """

        # Alias names to save typing
        x,P,Q,R = self.x, self.P, self.Q, self.R

        y = z - self.h(x) # Y = Measurement "Error" or Innovation
        S = dot(H,P,H.T) + R # S = Innovation Covariance
        K = dot(P,H.T,np.linalg.pinv(S)) # K = "Optimal" Kalman Gain; pinv for numerical stability

        self.x += dot(K,y) # Now update x
        self.P -= dot(K,H,P) # Now update P
        return self.x

    def f(self,x):
        th,v,w = x[2:5,:]
        return x + colvec(v*cos(th)*dt,v*sin(th)*dt,w*dt,0,0)
    
    def F(self,x):
        # Jacobian of f
        F = np.eye(5)
        th,v = x[2:4,:]
        s,c = sin(th), cos(th)
        F[0,2] = -v*s*dt
        F[0,3] = c*dt
        F[1,2] = v*c*dt
        F[1,3] = s*dt
        F[2,4] = dt
        raise NotImplementedError()    

    def h(self,x):
        # map x --> observations vector
        # obs = [compass, accelerometer, gyroscope, gps, encoder]
        return np.vstack((IMU.h(x), GPS.h(x), Encoder.h(x)))

    def H(self,x):
        # Jacobian of h
        return np.vstack((IMU.H(x), GPS.H(x), Encoder.H(x)))
        raise NotImplementedError()    

class DiffDriveRobot(object):
    def __init__(self):
        # Diff Drive Robot
        # Parametrized by Wheel Distance, etc.

        self.state = colvec(0,0,0,0,0) # x,y,t,v,w
        self.e_state = colvec(0,0,0,0,0) # estimated state
    def imu(self):
        # IMU = 3DoF w/h Accelerometer, Magnetometer(Compass), Gyroscope
        # IMU --> "Absolute Orientation" based on Magnetometer & Gyroscope
        # lin vel, ang acc, orientation 
        pass
    def gps(self):
        origin = 42.2932, -71.2637
        # simulate gps
        pass
    def encoder(self):
        # N_TICKS = 24
        pass
    def sense(self):
        return self.imu() + self.gps() + self.encoder()








