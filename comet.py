import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation as animation
TimedAnimation = animation.TimedAnimation

def minmax(a):
    return np.min(a), np.max(a)

def plot_line(p1,p2,label):
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], label=label)

class CometAnimation(TimedAnimation):
    def __init__(self, real_pos, est_pos, real_vel, est_vel, fig=None, ax=None):

        if fig == None:
            fig = plt.figure()
            print '::)'
        if ax == None:
            ax = fig.add_subplot(1,1,1)

        ax.set_xlabel('x')
        ax.set_ylabel('y')

        self.l = len(real_pos) 

        minrx, maxrx = minmax(real_pos[:,0])
        minry, maxry = minmax(real_pos[:,1])
        minmx, maxmx = minmax(est_pos[:,0])
        minmy, maxmy = minmax(est_pos[:,1])

        minx = min(minrx, minmx)
        miny = min(minry, minmy)
        maxx = max(maxrx, maxmx)
        maxy = max(maxry, maxmy)
        
        ax.set_xlim(minx, maxx)
        ax.set_ylim(miny, maxy)

        ax.set_aspect('equal')

        self.real_pos = real_pos
        self.est_pos = est_pos

        self.real_vel = real_vel
        self.est_vel = est_vel

        self.comet1, = plt.plot([],[],label='real')
        self.comet2, = plt.plot([],[],label='estimated')
        self.vel1, = plt.plot([],[],label='r_vel')
        self.vel2, = plt.plot([],[],label='e_vel')

        self.centers, = plt.plot([],[],'or',label='centers')
        self.origin, = plt.plot([0],[0],"*b")
        plt.legend()

        animation.TimedAnimation.__init__(self, fig, interval=50, blit=True)

    def _draw_frame(self, framedata):
        i = framedata
        self.comet1.set_data(self.real_pos[1:i, 0], self.real_pos[1:i, 1])
        self.comet2.set_data(self.est_pos[1:i, 0], self.est_pos[1:i, 1])
        # TODO:  Implementing here
        #self.vel1.set_data(self.real_vel[i], :

        self.centers.set_data(
                [self.real_pos[i,0],self.est_pos[i,0]],
                [self.real_pos[i,1],self.est_pos[i,1]],
                )

        self._drawn_artists = [self.comet1, self.comet2, self.centers, self.origin]

    def new_frame_seq(self):
        return iter(range(self.l))

    def _init_draw(self):
        plts = [self.comet1, self.comet2, self.centers]
        for p in plts:
            p.set_data([], [])
