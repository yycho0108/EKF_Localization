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

dt = 1e-2 # .01 sec

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

        gpv = gps.s()**2
        gyv = gyroscope.s()**2
        mv = magnetometer.s()**2
        cv = compass.s()**2
        ev = encoder.s()**2

        self.R = np.diag([gpv,gpv,gyv,mv,mv,cv,ev,ev])
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

def add_noise(x,s):
    return x + np.random.normal(loc=0,scale=s,size=x.shape)

def rpm2rps(r):
    return r * (2 * pi / 60)

def v2t(V,w):
    # voltage to motor torque
    # T(V,w)
    k = 1. / rpm2rps(1.315 * (500/12))
    R = 2.45
    return V*k/R - k*k*w/R

def t2a(T):
    # M = Mass of Wheel
    M = .1 # kg
    I = (1./2)*M*W_R**2
    return T/I

def u2a(x,u):
    # U as voltage for Both motors
    # convert to v_l, v_r
    x,y,t,v,w = x[:,0]

    rml = w*W_D
    rpl = v*2.0
    
    v_r = (rml + rpl) / 2.0
    v_l = (rpl - rml) / 2.0

    w_l = v_l / W_R
    w_r = v_r / W_R
    
    V_l,V_r = u[:,0]
    T_l,T_r = v2t(V_l,w_l), v2t(V_r,w_r)
    a_l,a_r = t2a(T_l), t2a(T_r)

    a = (a_l + a_r) / 2.0
    return a

def u2v(x,u):
    # U as voltage for Both motors
    # convert to v_l, v_r
    x,y,t,v,w = x[:,0]

    rml = w*W_D
    rpl = v*2.0
    
    v_r = (rml + rpl) / 2.0
    v_l = (rpl - rml) / 2.0

    w_l = v_l / W_R
    w_r = v_r / W_R
    
    V_l,V_r = u[:,0]
    T_l,T_r = v2t(V_l,w_l), v2t(V_r,w_r)
    a_l,a_r = t2a(T_l), t2a(T_r)
    #print a_l, a_r
    w_l += a_l * dt
    w_r += a_r * dt
    v_l = w_l * W_R
    v_r = w_r * W_R
    return v_l, v_r

def move(x,u):
    # diff drive, prediction based on its wheel encoder

    # ===
    # U as Voltage for Both Motors
    v_l,v_r = u2v(x,u)
    # ====
    # U as Angular Velocities for Both Motors
    # w_l,w_r = u[:,0]
    # v_l,v_r = w_l*W_R,w_r*W_R
    # ====
    x,y,t,v,w = x[:,0]

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

    G = colvec(2e-2,2e-2,1e-2,2e-2,2e-2)
    N = np.random.normal(size=(5,1))*G
    return colvec(x,y,t,v,w) + N

def sense(x,u):

    # obtain v_l, v_r from x,u when u is voltage
    v,w = x[3:,0]
    v_l,v_r = u2v(x,u)
    u = colvec(v_l, v_r)

    return np.vstack((gps.get(x),gyroscope.get(x),magnetometer.get(x),compass.get(x),encoder.get(u)))

if __name__ == "__main__":
    ekf = PoseEKF(5,8)
    x = np.random.rand(5,1) # real state
    #x = np.zeros((5,1))
    e_x = x.copy() # estimated state -- same as real
    #e_x = np.random.normal(size=(5,1), scale=10) # -- random!

    real = []
    est = []

    r_vel = []
    e_vel = []
    err = []
    # TODO : Plot Velocities

    u = np.random.normal(size=(2,1), scale=10) # cmd
    b = np.random.normal(size=(2,1), scale=5)
    a = np.zeros((2,1))

    for i in range(1000):
        if i % 10 == 0:
            #u = np.zeros((2,1)) 
            u = np.random.normal(size=(2,1), scale=12)
            #u = np.ones((2,1))

            #a = b + np.random.normal(size=(2,1), scale=10) # cmd

        x = move(x,u)

        real.append(x[:2,0])
        est.append(e_x[:2,0])
        #err.append(np.linalg.norm(x[:2] - e_x[:2])) # error measure based on displacement
        err.append(abs(x[3,0] - e_x[3,0])) # error measure based on velocity

        z = sense(x,u) # sensor values after movement
        #print 'z', z
        ekf.update(z,u)

        e_x = ekf.predict(e_x)

        print 'x', x
        print 'e_x', ekf.x

    real = np.array(real)
    est = np.array(est)

    plt.plot(err)
    plt.show()

    ani = CometAnimation(real,est)
    plt.show()

    save = str(raw_input("SAVE?(y/n)\n")).lower()
    if(save == 'y'):
        ani.save('demo.mp4')
