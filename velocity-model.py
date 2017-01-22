#!/usr/bin/python

# Sensors : GPS, IMU, Wheel Encoder
# GPS --> Absolute Position, mapped to lat/long

import numpy as np
from abc import ABCMeta, abstractmethod
from sensor import GPS, Gyroscope, Magnetometer, Compass, Accelerometer, IMU, Encoder 
from utility import *
from matplotlib import pyplot as plt
from comet import CometAnimation

W_R = 0.2 # Wheel Rad
W_D = 0.3 # Wheel Dist

# instantiate virtual sensors
gps = GPS()
gyroscope = Gyroscope()
magnetometer = Magnetometer()
compass = Compass()
accelerometer = Accelerometer()
imu = IMU()
encoder = Encoder(W_R,W_D)

dt = 5e-2 # .05 sec

# TODO : possibly 
class PoseEKF(object):
    def __init__(self, n, m):
        # TODO : modify P initialization and (especially) R w.r.t sensor characteristics

        p = 1e-2
        q = 1e-1

        # Initialization values
        self.x = np.zeros((n,1))

        self.P = np.eye(n) * p
        # (Initial) Covariance

        self.Q = np.eye(n) * q
        # Process Noise Model -- depends on robot locomotion accuracy

        self.R = np.diag([gps.s(),gps.s(),gyroscope.s(),magnetometer.s(),magnetometer.s(),compass.s(),encoder.s(),encoder.s()])
        # Measurement Noise Model -- depends on sensor precision

        # gps x2
        # gyro x1
        # magneto x2
        # compass x1
        # encoder x2

    def predict(self,x):
        """
        predict(x) : x = state vector
        """
        self.x = x

        # Alias names to save typing
        x,P,Q,R = self.x, self.P, self.Q, self.R

        self.x = self.f(x) # dot(B,u), but not used here
        F = self.F(x)
        self.P = dot(F,P,F.T) + Q
        return self.x

    def update(self, z, u):
        """
        update(z) : z = observations
        """

        # Alias names to save typing
        x,P,Q,R = self.x, self.P, self.Q, self.R
        H = self.H(x)

        y = z - self.h(x,u) # Y = Measurement "Error" or Innovation
        S = dot(H,P,H.T) + R # S = Innovation Covariance
        K = dot(P,H.T,np.linalg.pinv(S)) # K = "Optimal" Kalman Gain; pinv for numerical stability

        self.x += dot(K,y) # Now update x
        self.P -= dot(K,H,P) # Now update P
        return self.x

    def f(self,x):
        th,v,w = x[2:5,0]
        res = x + colvec(v*cos(th)*dt,v*sin(th)*dt,w*dt,0,0)
        res[2,0] = norm_angle(res[2,0])
        return res
    
    def F(self,x):
        # Jacobian of f
        _F = np.eye(5)
        th,v = x[2:4,:]
        s,c = sin(th), cos(th)
        _F[0,2] = -v*s*dt
        _F[0,3] = c*dt
        _F[1,2] = v*c*dt
        _F[1,3] = s*dt
        _F[2,4] = dt
        return _F

    def h(self,x,u):
        # map x --> observations vector
        # u is only needed here because encoder needs to simulate input :P
        # obs = [gps, gyro, magneto, compass, encoder] #TODO: magneto and compass are redundant; try removing one
        # Accelerometer not being used #TODO: consider augmenting state definition
        return np.vstack((gps.h(x), gyroscope.h(x), magnetometer.h(x), compass.h(x), encoder.h(u)))

    def H(self,x):
        # Jacobian of h
        return np.vstack((gps.H(x), gyroscope.H(x), magnetometer.H(x), compass.H(x), encoder.H(x)))

#class DiffDriveRobot(object):
#    def __init__(self):
#        # Diff Drive Robot
#        # Parametrized by Wheel Distance, etc.
#        self.state = colvec(0,0,0,0,0) # x,y,t,v,w
#        self.e_state = colvec(0,0,0,0,0) # estimated state
#    def sense(self):
#        return self.imu() + self.gps() + self.encoder()

def move(x,u):
    # diff drive, prediction based on its wheel encoder
    x,y,t,v,w = x[:,0]

    # U as Angular Acceleration for Both Motors

    #rml = w*W_D
    #rpl = v*2.0

    #v_r = (rml + rpl) / 2.0
    #v_l = (rpl - rml) / 2.0

    #a_l,a_r = u[:,0]

    #v_l += a_l * dt
    #v_r += a_r * dt

    # U as Angular Velocities for Both Motors
    w_l,w_r = u[:,0]
    v_l,v_r = w_l*W_R,w_r*W_R

    R = (W_D/ 2.0) * (v_l+v_r)/(v_r-v_l)
    w = (v_r-v_l) / (W_D)
    v = (v_r+v_l) / 2.0

    iccx, iccy = x-R*np.sin(t),y+R*np.cos(t)
    wdt = w*dt
    M = np.asarray([
        [np.cos(wdt),-sin(wdt),0],
        [np.sin(wdt),cos(wdt),0],
        [0,0,1]
        ])

    x,y,t = (dot(M,colvec(x-iccx, y-iccy,t)) + colvec(iccx,iccy,wdt))[:,0]
    t = norm_angle(t)
    # todo : add process noise
    return colvec(x,y,t,v,w)

def sense(x,u):
    return np.vstack((gps.get(x),gyroscope.get(x),magnetometer.get(x),compass.get(x),encoder.get(u)))

if __name__ == "__main__":
    ekf = PoseEKF(5,8)
    x = np.random.rand(5,1) # real state
    #x = np.zeros((5,1))
    e_x = x.copy() # estimated state

    real = []
    est = []

    r_vel = []
    e_vel = []
    # TODO : Plot Velocities

    for i in range(1000):
        if i % 100 == 0:
            u = np.random.normal(size=(2,1)) # cmd
        x = move(x,u)

        real.append(x[:,0])
        est.append(e_x[:,0])

        for j in range(10):
            z = sense(x,u) # sensor values after movement
            #print 'z', z
            ekf.update(z,u)
        e_x = ekf.predict(e_x)
        print 'x', x
        print 'e_x', ekf.x

    real = np.array(real)
    est = np.array(est)

    ani = CometAnimation(real,est)
    plt.show()

    save = str(raw_input("SAVE?(y/n)\n")).lower()
    if(save == 'y'):
        ani.save('demo.mp4')
