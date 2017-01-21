import numpy as np

# unpack
cos = np.cos
sin = np.sin
pi = np.pi
d2r = np.deg2rad
r2d = np.rad2deg

def colvec(*args):
    return np.atleast_2d(args).T

def dot(*args):
    return reduce(np.dot, args)

def R(t):
    c = cos(t)
    s = sin(t)
    return np.array([[c,-s],[s,c]])
