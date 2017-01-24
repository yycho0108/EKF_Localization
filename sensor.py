# sigma = stddev
# variance = sigma ** 2
# TODO : Convert Everything to np.mat
# TODO : Implement Accelerometer when State Definition is Modified

from abc import ABCMeta, abstractmethod
from utility import *
import numpy as np
from matplotlib import pyplot as plt


class Sensor(object):
    """
    Base Class for Producing Fake Sensor
    """
    __metaclass__ = ABCMeta
    def __init__(self,sigma,n):
        self.sigma = sigma
        self.n = n
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
    def s(self):
        return [self.sigma for _ in range(self.n)]

# Absolute Position, Lat/Long
# TODO : simulate losing signal
class GPS(Sensor):
    lt = 42.2932
    ln = -71.2637 # Origin - Olin's Location
    ER = 6.371e6 # Earth's Radius in M
    Rl = ER * np.abs(cos(d2r(lt))) # Radius at Latitude
    def __init__(self):
        s_m = 1.5 # stddev, in Meters-ish

        # variance was around 1.4~3.0, sttdev = about 1.5
        s = s_m * (180./pi) / GPS.ER # stddev in Degrees(lat,long)
        super(GPS,self).__init__(s,2) # added noise in terms of deg.

    def get(self,x):
        #return self.h(x) + np.random.normal(loc=0,scale=self.sigma,size=(2,1))
        z = self.add_noise(self.h(x))
        #print 'x vs. gps_x', x[:2,0], GPS.h_inv(z)
        return z

    def h(self,x):
        # mapping from state to observation
        lt,ln,ER,Rl = self.lt,self.ln,self.ER,self.Rl # Unpack
        dx,dy = x[:2,0] # change from origin
        dph = dy / (ER * 2 * pi) * 360
        dth = dx / (Rl * 2 * pi) * 360
        return colvec(ln+dth,lt+dph) # Fake GPS Coordinates

    # Invert Transformation...
    @staticmethod
    def h_inv(z):
        # mapping from observation to state
        lt,ln,ER,Rl = GPS.lt,GPS.ln,GPS.ER,GPS.Rl # Unpack
        t,p = z[:,0]
        dph,dth = p-lt,t-ln
        dy = (dph * pi / 180) * ER
        dx = (dth * pi / 180) * Rl
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
        super(Gyroscope,self).__init__(s,1) # .095 deg/s
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
        #self.decl = -14.61 # decl = true_north - mag_north
        self.decl = 0.0 # TODO : How do I account for bias?
        s_g = 1.1e-3 # white noise 1.1 mG
        s = s_g / .52
        # assuming 0.52G (Total Field Strength) corresponds to "1"
        # Magnetic Declination ~= -14.61
        super(Magnetometer,self).__init__(s,2)
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
        super(Compass,self).__init__(s,1)
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
        super(Accelerometer,self).__init__(s,1)
        self.v = None
    def get(self,x):
        # cheat a bit and return velocity reading?
        return self.add_noise(self.h(x))
    def h(self,x,a):
        p_x,p_y,t,v,w = x[:,0]
        dv = a * dt
        return colvec(v+dv) # cheat and return velocity ... um
    def H(self,x):
        pass

class IMU(Sensor):
    # Compass + Gyro + Accelerometer
    def __init__(self):
        super(IMU,self).__init__(0.0,0) # Obviously wrong
        pass
    def get(self,x):
        pass
    def h(self,x):
        pass
    def H(self,x):
        pass

# v = w*r

class Encoder(Sensor):
    def __init__(self,r,l):
        # TODO : add resolution constraint (# TICKS)
        s = 1e-2 # Arbitrary, stddev .1m
        super(Encoder,self).__init__(s,2)
        self.r = r # Wheel radius
        self.l = l # Wheel Distance
    def set_u(self,u):
        self.u = u
    def get(self,x):
        u = self.u
        # real!
        r,l = self.r, self.l
        v_l,v_r = u2v(x,u)
        w_l,w_r = v_l / W_R, v_r / W_R
        return self.add_noise(colvec(w_l, w_r))
        #v = (v_r+v_l) / 2
        #w = 2 * (v_r - v) / (l/2)
        #res = colvec(v,w)
        #return self.add_noise(res)
    def h(self,x):
        # w_l, w_r based on v,w
        v,w = x[3:,0]
        # v_r+v_l = v*2
        # v_r-v_l = w*W_D
        p = v*2
        m = w*W_D
        v_l, v_r = (p-m)/2.0, (p+m)/2.0
        w_l, w_r = v_l/W_R, v_r/W_R
        return colvec(w_l, w_r)
    def H(self,x):
        #v,w -> w_l, w_r
        res = np.zeros((2,5))
        res[0,3] = 1./W_R
        res[0,4] = -W_D/(2.*W_R)
        res[1,3] = 1./W_R
        res[1,4] = W_D/(2.*W_R)
        return res
# Alternative Model
#    def get(self,x):
#        u = self.u
#        # real!
#        r,l = self.r, self.l
#        v_l,v_r = u2v(x,u)
#        w_l,w_r = v_l/W_R, v_r/W_R
#        return self.add_noise(colvec(w_l, w_r))
#    def h(self,x):
#        # w_l, w_r based on v,w
#        v,w = x[3:,0]
#        # v_r+v_l = v*2
#        # v_r-v_l = w*W_D
#        p = v*2
#        m = w*W_D
#        v_l, v_r = (p-m)/2., (p+m)/2.
#        w_l, w_r = v_l/W_R, v_r/W_R
#        return colvec(w_l, w_r)
#    def H(self,x):
#        #v,w -> w_l, w_r
#        res = np.zeros((2,5))
#        res[0,3] = 1./W_R
#        res[0,4] = -W_D/(2.*W_R)
#        res[1,3] = 1./W_R
#        res[1,4] = W_D/(2.*W_R)
#        return res

class Beacon(Sensor):
    def __init__(self,x,y):
        self.x = x
        self.y = y
        s = 1e-1 # 1 m deviation
        super(Beacon,self).__init__(s,1)
    def get(self,x):
        return self.add_noise(self.h(x))
    def h(self,x):
        x,y = x[:2,0]
        dx = x - self.x
        dy = y - self.y
        d = np.sqrt(dx**2+dy**2)
        return colvec(d)
    def H(self,x):
        d = self.h(x)[0,0]
        ref = colvec(self.x, self.y)

        res = np.zeros((1,5))
        if d == 0:
            return res
        tmp = (x[:2,:] - ref) / d
        dx,dy = tmp[:,0]
        res[0,0] = dx
        res[0,1] = dy
        return res

def test_gps(x):
    # Test GPS
    gps = GPS()
    z = gps.get(x)
    print 'x', x
    print 'x->z', z
    print 'z->x', GPS.h_inv(z)
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
    enc.set_u(u)
    z = enc.get(x)
    print 'x', x
    print 'x->z', z
    print 'H', enc.H(x)

# instantiate virtual sensors
gps = GPS()
gps_2 = GPS()
gyroscope = Gyroscope()
magnetometer = Magnetometer()
compass = Compass()
accelerometer = Accelerometer()
imu = IMU()
encoder = Encoder(W_R,W_D)
beacon_1 = Beacon(0,0) # beacon at origin
beacon_2 = Beacon(5,5) # beacon
beacon_3 = Beacon(0,5) # beacon
beacon_4 = Beacon(5,0) # beacon

#sensors = [gps]
#sensors = [encoder]
sensors = [gps,encoder]
#sensors = [gps,encoder,gyroscope,magnetometer]
#sensors = [gps,encoder]#,beacon_1,beacon_2]
#sensors = [gps,gyroscope,magnetometer,encoder]#,beacon_1,beacon_2,beacon_3,beacon_4]

def sense(x,u):
    encoder.set_u(u) # indicate u = voltage cmds
    return np.vstack([s.get(x) for s in sensors])

if __name__ == "__main__":
    x = np.random.rand(5,1) # x,y,t,v,w
    test_gps(x)
    #test_gyro(x)
    #test_mag(x)
    #test_compass(x)
    #test_accel(x)
    u = colvec(-1,1.5) # cmd, wl-wr, rad/s
    test_enc(x,u)
    # Test 
    pass
