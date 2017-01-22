# Sensors : GPS, IMU, Wheel Encoder
# GPS --> Absolute Position, mapped to lat/long

import numpy as np
from abc import ABCMeta, abstractmethod

def colvec(*args):
    return np.atleast_2d(args).T

def dot(*args):
    return reduce(np.dot, args)

class EKF(object):
    __metaclass__ = ABCMeta
    def __init__(self, n, m, x = None, pqr_init = (0.1, 1e-4, 0.1)):

        # Initialization values
        p,q,r = pqr_init 

        super(self,EKF).__init__()
        if x == None:
            self.x = np.zeros((n,1))
        if P == None:
            # p initialzed to covariance of 9999 ... arbitrary :/
            self.P = np.eye(n) * p

        self.Q = np.eye(n) * q # Process Noise Model
        self.R = np.eye(m) * r # Measurement Noise Model
 
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

    @abstractmethod
    def f(self,x,u):
        # u = Command :)
        # position = position + dt * velocity-in-dir (+ 0.5 * dt^2 * acceleration-in-dir)
        # velocity = velocity + dt * acceleration-in-dir
        # acceleration = acceleration

        # rotation = rotation + dt * ang_vel
        # angular velocity = ... <<

        raise NotImplementedError()    

    @abstractmethod
    def getF(self):
        raise NotImplementedError()    

    @abstractmethod
    def h(self,x):
        # map x --> observations
        raise NotImplementedError()    

    @abstractmethod
    def getH(self,x):
        raise NotImplementedError()    

class PoseEKF(EKF):
    """
    PoseEKF
    Extended Kalman Filter;
    Implements Odometry-Based Motion Model
    """
    def __init__(self):
        super(self,PoseEKF).__init__(8,3)
        # 8 = size of state
        # 3 = # of measurements

    def f(self,x,u):
        # u = actually the estimated x(t) given cmd and x(t-1)
        x_1,y_1,t_1 = x[:3,0]
        x_2,y_2,t_2 = u[:,0] # unpack
        dx = x_2-x_1
        dy = y_2-y_1

        d_r1 = np.arctan2(dy,dx)
        d_tr = np.sqrt((dx)**2 + (dy)**2)
        d_r2 = t_2 - t_1 - d_r1

        G_t = np.asarray([
                [1,0,-d_tr*np.sin(t_1+d_r1)],
                [0,1,d_tr*np.cos(t_1+d_r1)],
                [0,0,0]
                ])
        V_t = np.asarray([
                [-d_tr*np.sin(t_1+d_r1), np.cos(t_1+d_r1), 0],
                [d_tr*np.cos(t_1+d_r1), np.sin(t_1+d_r1), 0],
                [1,0,1]
                ])


    def getF(self):
        pass
    def h(self,x):
        pass
    def getH(self,x):
        pass


class DiffDriveRobot(object):
    def __init__(self):
        # Diff Drive Robot
        # Parametrized by Wheel Distance, etc.

        self.state = colvec(0,0,0,0,0,0,0,0) # x,y,t, x',y',t', x',y'
        self.e_state = colvec(0,0,0,0,0,0,0,0,0) # estimated state
    def imu(self):
        # IMU = 3DoF w/h Accelerometer, Magnetometer(Compass), Gyroscope
        # IMU --> "Absolute Orientation" based on Magnetometer & Gyroscope
        # lin vel, ang acc, orientation 
        pass
    def gps(self):
        pass
    def encoder(self):
        # N_TICKS = 24
        pass
    def sense(self):
        return self.imu() + self.gps() + self.encoder()
    def predict(self,cmd,dt):
        # diff drive, prediction based on its wheel encoder
        w_l,w_r = cmd
        v_l,v_r = w_l*WHEEL_RADIUS,w_r*WHEEL_RADIUS
        R = (WHEEL_DISTANCE / 2.0) * (v_l+v_r)/(v_r-v_l)
        w = (v_r-v_l) / (WHEEL_DISTANCE)

        x,y,p = self.state[:3,0]
        iccx, iccy = x-R*np.sin(p),y+R*np.cos(p)
        wdt = w*dt

        M = np.asarray([
            [np.cos(wdt),-sin(wdt),0],
            [np.sin(wdt),cos(wdt),0],
            [0,0,1]
            ])
        return dot(M,colvec(x-iccx, y-iccy,p)) + colvec(iccx,iccy,wdt)
        











