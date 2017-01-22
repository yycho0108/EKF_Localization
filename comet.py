import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation as animation
TimedAnimation = animation.TimedAnimation

def minmax(a):
    return np.min(a), np.max(a)

class CometAnimation(TimedAnimation):
    def __init__(self, real_val, est_val,fig=None, ax=None):

        if fig == None:
            fig = plt.figure()
            print '::)'
        if ax == None:
            ax = fig.add_subplot(1,1,1)

        ax.set_xlabel('x')
        ax.set_ylabel('y')

        self.l = len(real_val) 

        minrx, maxrx = minmax(real_val[:,0])
        minry, maxry = minmax(real_val[:,1])
        minmx, maxmx = minmax(est_val[:,0])
        minmy, maxmy = minmax(est_val[:,1])

        minx = min(minrx, minmx)
        miny = min(minry, minmy)
        maxx = max(maxrx, maxmx)
        maxy = max(maxry, maxmy)
        
        ax.set_xlim(minx, maxx)
        ax.set_ylim(miny, maxy)

        ax.set_aspect('equal')

        self.real_val = real_val
        self.est_val = est_val

        self.comet1, = plt.plot([],[],label='real')
        self.comet2, = plt.plot([],[],label='estimated')
        self.centers, = plt.plot([],[],'or',label='centers')
        self.origin, = plt.plot([0],[0],"*b")
        plt.legend()

        animation.TimedAnimation.__init__(self, fig, interval=50, blit=True)

    def _draw_frame(self, framedata):
        i = framedata
        self.comet1.set_data(self.real_val[1:i, 0], self.real_val[1:i, 1])
        self.comet2.set_data(self.est_val[1:i, 0], self.est_val[1:i, 1])
        self.centers.set_data(
                [self.real_val[i,0],self.est_val[i,0]],
                [self.real_val[i,1],self.est_val[i,1]],
                )

        self._drawn_artists = [self.comet1, self.comet2, self.centers, self.origin]

    def new_frame_seq(self):
        return iter(range(self.l))

    def _init_draw(self):
        plts = [self.comet1, self.comet2, self.centers]
        for p in plts:
            p.set_data([], [])
