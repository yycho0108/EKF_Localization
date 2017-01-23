import numpy as np

# unpack
cos = np.cos
sin = np.sin
pi = np.pi
d2r = np.deg2rad
r2d = np.rad2deg

# parameters
W_R = 0.2 # Wheel Rad
W_D = 0.3 # Wheel Dist
dt = 1e-2 # .01 sec

def colvec(*args):
    return np.atleast_2d(args).T

def dot(*args):
    return reduce(np.dot, args)

def R(t):
    c = cos(t)
    s = sin(t)
    return np.array([[c,-s],[s,c]])

def norm_angle(theta):
    return np.arctan2(np.sin(theta),np.cos(theta))


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
