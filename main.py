#!/usr/bin/python

import pygame
from pygame.locals import *

from display import Display, RobotObject, RobotEstimateObject, GPSObject, StatusObject 
from velocity_model import PoseEKF, sensors, sense, move
import numpy as np
from utility import *

from sensor import GPS

W,H = 1000,1000

def quit():
    for event in pygame.event.get():
        if event.type == QUIT:
            return True
        if event.type == KEYDOWN and event.key == K_ESCAPE:
            return True
    return False

def get_cmd():
    mx,my = pygame.mouse.get_pos()
    cx,cy = W*3./4, H*3./4
    dx,dy = mx-cx,-(my-cy)
    speed = np.sqrt(dx**2 + dy**2)
    S_MAX = W/4.
    if(speed < S_MAX):
        # 125 == 12V
        v = dy * (12. / S_MAX) 
        w = dx * (12. / S_MAX)
        v_l = v + w/2
        v_r = v - w/2
        return colvec(v_l, v_r)
    else:
        return colvec(0,0)

def status_string(x_r, x_e):
    return str(x_r) + str(x_e)

def main():
    # == STATE ==
    x_r = np.random.rand(5,1) # real state
    x_e = np.random.rand(5,1) # real state
    #x_e = x_r.copy() # estimated state
    ekf = PoseEKF(5,sensors)

    # == DISPLAY ==
    disp = Display(W,H)
    r_r = RobotObject((255,128,128)) # real robot
    r_e = RobotEstimateObject((128,255,255)) # Estimated Robot

    s_r = StatusObject((0,0),'State')
    s_e = StatusObject((0,200),'Estimated State')

    #g = GPSObject()

    # == LOOP ==
    while True:
        if quit():
            pygame.quit()
            break
        u = get_cmd()

        # print ekf.P[3,3]
        # print 'e,r', x_e[3], x_r[3],
        x_e = ekf.predict(x_e)
        #print ekf.P[0,0], ekf.P[1,1]
        x_r = move(x_r, u)
        z = sense(x_r, u) # observe based on REAL states
        ekf.update(z,u)

        # display-related
        r_r.update(x_r)
        r_e.update(x_e,ekf.P[:2,:2])
        s_r.update(x_r)
        s_e.update(x_e)

        # update this when changing around the order of the sensors
        # g.update(GPS.h_inv(z[:2,:]))

        disp.draw([r_e, r_r, s_r, s_e])
        disp.update()
        pygame.time.wait(30)

if __name__ == "__main__":
    main()
