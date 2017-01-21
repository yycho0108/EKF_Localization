# sigma = stddev
# variance = sigma ** 2
# TODO : Convert Everything to np.mat
# TODO : Implement Accelerometer when State Definition is Modified

from abc import ABCMeta, abstractmethod
from utility import *
import numpy as np
from matplotlib import pyplot as plt


def norm_angle(theta):
    return np.arctan2(np.sin(theta),np.cos(theta))

class Sensor(object):
    """
    Base Class for Producing Fake Sensor
    """
    __metaclass__ = ABCMeta
    def __init__(self,sigma):
        self.sigma = sigma
        pass
    @abstractmethod
    def get(self,x):
        pass
    @abstractmethod
    def h(self,x):
        pass
    @abstractmethod
    def H(self,x):
        pass
    def add_noise(self,x):
        return x + np.random.normal(loc=0,scale=self.sigma,size=x.shape)

# Absolute Position, Lat/Long
class GPS(Sensor):
    def __init__(self):
        lt = 42.2932
        ln = -71.2637 # Origin - Olin's Location
        ER = 6.371e6 # Earth's Radius in M
        Rl = ER * np.abs(cos(d2r(lt))) # Radius at Latitude

        s_m = 1.5 # stddev in Meters
        # variance was around 1.4~3.0, sttdev = about 1.5

        s = s_m * (180./pi) / ER # Variance in Degrees
        super(GPS,self).__init__(s)
        self.lt,self.ln,self.ER,self.Rl = lt,ln,ER,Rl

    def get(self,x):
        return self.add_noise(self.h(x))

    def h(self,x):
        # mapping from state to observation
        lt,ln,ER,Rl = self.lt,self.ln,self.ER,self.Rl # Unpack
        dx,dy = x[:2,0] # change from origin
        dp = dy / (ER * 2 * pi) * 360
        dt = dx / (Rl * 2 * pi) * 360
        return colvec(ln+dt,lt+dp) # Fake GPS Coordinates

    # Invert Transformation...
    def h_inv(self,z):
        # mapping from observation to state
        lt,ln,ER,Rl = self.lt,self.ln,self.ER,self.Rl # Unpack
        t,p = z[:,0]
        dp,dt = p-lt,t-ln
        dy = (dp * pi / 180) * ER
        dx = (dt * pi / 180) * Rl
        return colvec(dx,dy)
    def H(self,x):
        # Jacobian Matrix H
        # effectively d(h(x))/d(x)
        res = np.zeros((2,5))
        res[0,0] = 360./(self.Rl*2*pi)
        res[1,1] = 360./(self.ER*2*pi)
        return res

# Angular Velocity, rad/s
class Gyroscope(Sensor):
    def __init__(self):
        s = d2r(.095)
        super(Gyroscope,self).__init__(s) # .095 deg/s
    def get(self,x):
        return self.add_noise(self.h(x))
    def h(self,x):
        #x,y,t,v,w
        return np.array([x[4]])
    def H(self,x):
        res = np.zeros((1,5))
        res[0,4] = 1.
        return res

class Magnetometer(Sensor):
    def __init__(self):
        self.decl = -14.61 # decl = true_north - mag_north
        s_g = 1.1e-3 # white noise 1.1 mG
        s = s_g / .52
        # assuming 0.52G (Total Field Strength) corresponds to "1"
        # Magnetic Declination ~= -14.61
        super(Magnetometer,self).__init__(s)
    def get(self,x):
        # ignore "z" component, only return x,y w.r.t. true north
        return self.add_noise(self.h(x))
    def h(self,x):
        t = x[2,0]
        return dot(R(-t),R(d2r(-self.decl)),colvec(0,1))
    def H(self,x):
        t = x[2,0]
        res = np.zeros((2,5))
        cd,sd = cos(self.decl), sin(self.decl)
        ct,st = cos(t), sin(t)
        res[0,2] = -sd*st + cd*ct
        res[1,2] = -sd*ct - cd*st
        return res

# Angular Position, rad
class Compass(Sensor):
    def __init__(self):
        s = 1.1e-3 / .52
        super(Compass,self).__init__(s)
    def get(self,x):
        #x,y,t,v,w
        return self.add_noise(self.h(x))
    def h(self,x):
        # w.r.t North ...? TODO: Validate
        return np.array(norm_angle([x[2] + np.pi / 2]))
    def H(self,x):
        res = np.zeros((1,5))
        res[0,2] = 1.
        return res

# Linear Acceleration - uh oh.
class Accelerometer(Sensor):
    def __init__(self):
        s_g = 2.8e-3
        s = s_g * 9.8 # m/s^2
        super(Accelerometer,self).__init__(s)
    def get(self,x):
        return self.add_noise(self.h(x))
    def h(self,x):
        pass
    def H(self,x):
        pass

class IMU(Sensor):
    # Compass + Gyro + Accelerometer
    def __init__(self):
        super(IMU,self).__init__(0.0) # Obviously wrong
        pass
    def get(self,x):
        pass
    def h(self,x):
        pass
    def H(self,x):
        pass

class Encoder(Sensor):
    def __init__(self,r,l):
        s = 1e-2 # Arbitrary, stddev .01m
        super(Encoder,self).__init__(s)
        self.r = r # Wheel radius
        self.l = l # Wheel Distance
    def get(self,u):
        return self.add_noise(self.h(u))
    def h(self,u):
        r,l = self.r, self.l
        wl,wr = u
        vl,vr = r*wl, r*wr
        v = (vr+vl) / 2
        w = 2 * (vr - v) / (l/2)
        return colvec(v,w)
    def H(self,x):
        res = np.zeros((2,5))
        res[0,3] = 1.
        res[1,4] = 1.
        return res

def test_gps(x):
    # Test GPS
    gps = GPS()
    z = gps.get(x)
    print 'x', x
    print 'x->z', z
    print 'z->x', gps.h_inv(z)
    print 'H', gps.H(x)

def test_gyro(x):
    gyro = Gyroscope()
    z = gyro.get(x)
    print 'x', x
    print 'x->z', z
    print 'H', gyro.H(x)

def test_mag(x):
    mag = Magnetometer()
    z = mag.get(x)
    print 'x', x
    print 'x->z', z
    print 'H', mag.H(x)

def test_compass(x):
    compass = Compass()
    z = compass.get(x)
    print 'x', x
    print 'x->z', z
    print 'z->x', z-np.pi/2
    print 'H', compass.H(x)

#def test_accel(x):
#    accel = Accelerometer()
#    z = accel.get(x)
#    print 'x', x
#    print 'x->z', z

def test_enc(x,u):
    r = .1 # m
    l = .25 # m
    enc = Encoder(r,l)
    z = enc.get(u)
    print 'x', x
    print 'x->z', z
    print 'H', enc.H(x)


if __name__ == "__main__":
    x = np.random.rand(5,1) # x,y,t,v,w

    #test_gps(x)
    #test_gyro(x)
    #test_mag(x)
    #test_compass(x)
    #test_accel(x)
    u = (-1,1.5) # cmd, wl-wr, rad/s
    test_enc(x,u)
    # Test 
    pass
