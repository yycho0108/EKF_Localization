# Sensors : GPS, IMU, Wheel Encoder
# GPS --> Absolute Position, mapped to lat/long

import numpy as np
from abc import ABCMeta, abstractmethod
from sensor import GPS, Gyroscope, Magnetometer, Compass, Accelerometer, IMU, Encoder 
from utility import *

W_R = 0.2 # Wheel Rad
W_D = 0.3 # Wheel Dist

gps = GPS()
gyroscope = Gyroscope()
magnetometer = Magnetometer()
compass = Compass()
accelerometer = Accelerometer()
imu = IMU()
encoder = Encoder(W_R,W_D)

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

    def h(self,x,u):
        # map x --> observations vector
        # obs = [gps, gyro, magneto, compass, encoder]
        # Accelerometer not being used
        return np.vstack((gps.h(x), gyroscope.h(x), magnetometer.h(x), compass.h(x), encoder.h(u)))

    def H(self,x):
        # Jacobian of h
        return np.vstack((gps.H(x), gyroscope.H(x), magnetometer.H(x), compass.H(x), encoder.H(x)))
        raise NotImplementedError()    

class DiffDriveRobot(object):
    def __init__(self):
        # Diff Drive Robot
        # Parametrized by Wheel Distance, etc.
        self.state = colvec(0,0,0,0,0) # x,y,t,v,w
        self.e_state = colvec(0,0,0,0,0) # estimated state
    def sense(self):
        return self.imu() + self.gps() + self.encoder()


if __name__ == "__main__":
    ekf = PoseEKF(5,5)
