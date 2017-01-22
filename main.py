import pygame
from pygame.locals import *

from display import Display, RobotObject
from velocity_model import PoseEKF, sense, move
import numpy as np
from utility import *

def quit():
    for event in pygame.event.get():
        if event.type == QUIT:
            return True
    return False

def get_cmd():
    mx,my = pygame.mouse.get_pos()
    cx,cy = 500*3/4, 500*3/4
    dx,dy = mx-cx,-(my-cy)
    speed = np.sqrt(dx**2 + dy**2)
    if(speed < 125.):
        # 125 == 12V
        v = dy * (12. / 125) 
        w = dx * (12. / 125)
        v_l = v + w/2
        v_r = v - w/2
        return colvec(v_l, v_r)
    else:
        return colvec(0,0)


def main():
    # == STATE ==
    x_r = np.random.rand(5,1) # real state
    x_e = x_r.copy() # estimated state
    ekf = PoseEKF(5,8)

    # == DISPLAY ==
    disp = Display()
    r_r = RobotObject((255,128,128)) # real robot
    r_e = RobotObject((128,255,255)) # Estimated Robot

    # == LOOP ==
    while True:
        if quit():
            break
        u = get_cmd()

        x_r = move(x_r, u)
        z = sense(x_r, u)
        ekf.update(z,u)
        x_e = ekf.predict(x_e)

        r_r.update(x_r)
        r_e.update(x_e)
        disp.draw([r_e, r_r])
        disp.update()
        pygame.time.wait(100)

if __name__ == "__main__":
    main()
