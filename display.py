import pygame
import numpy as np
from pygame.locals import *

def m2p(m):
    return m * 100 # 1m = 100px
CX,CY = 250,250
r2d = np.rad2deg
WHITE = (255,255,255)
BLACK = (0,0,0)
GREEN = (0,255,0)

class RobotObject(pygame.sprite.Sprite):
    def __init__(self,color):
        # 20cm x 30cm robot dims.
        w,h = m2p(.2), m2p(.3)
        l = np.sqrt(w**2 + h**2)

        self.img = pygame.Surface((l,l))
        self.color = color
        self.t_col = (color[0]/2, color[1]/2, color[2]/2)

        top = (l-w)/2
        bot = (l+w)/2
        left = (l-h)/2
        right = (l+h)/2
        
        poly = [(left,top),(left,bot),(l/2,bot),(right,l/2),(l/2,top)]

        self.img.fill(WHITE)
        self.img.set_colorkey(WHITE)
        pygame.draw.polygon(self.img,self.color,poly)

        self.w,self.h = w,h
        self.trajectory = []

    def update(self,x):
        self.x = x
        x,y = self.x[:2,0]
        x,y = CX + m2p(x), CY - m2p(y)
        self.trajectory.append((x,y))

    def draw(self,screen):
        x,y,t,v,_ = self.x[:,0]
        x,y,v = CX + m2p(x), CY - m2p(y), m2p(v)
        r_img = pygame.transform.rotate(self.img, r2d(t))

        vx = v * np.cos(t) * .25 #.25 = arbitrary scale factor for better visualization
        vy = - v * np.sin(t) * .25 

        pygame.draw.line(screen, self.color,(x,y),(x+vx,y+vy))

        rect = r_img.get_rect()
        rect.center = (x,y)

        if len(self.trajectory) > 1:
            pygame.draw.lines(screen, self.t_col, False,self.trajectory)
        screen.blit(r_img, rect)

class RobotEstimateObject(RobotObject):
    def __init__(self,color):
        super(RobotEstimateObject,self).__init__(color)
        self.P = None
        self.margin = pygame.Surface((200,200))
        self.margin.set_colorkey(WHITE)
        self.margin.set_alpha(128)
    def update(self,x,P):
        super(RobotEstimateObject,self).update(x)
        self.P = P
        self.margin.fill(WHITE)

    def draw(self,screen):
        x,y,t,v,_ = self.x[:,0]
        x,y,v = CX + m2p(x), CY - m2p(y), m2p(v) # center of ellipse

        px,py = self.P[0,0], self.P[1,1]

        dx,dy = 2*m2p(np.sqrt(np.abs(px))), 2*m2p(np.sqrt(np.abs(py))) # 2 * stddev

        e_rect = (100-dx,100-dy, 2*dx, 2*dy) #left,top,width,height
        pygame.draw.ellipse(self.margin,self.t_col,e_rect,0) 

        rect = self.margin.get_rect()
        rect.center = (x,y)
        screen.blit(self.margin, rect)
        super(RobotEstimateObject,self).draw(screen)

class GPSObject(pygame.sprite.Sprite):
    def __init__(self):
        self.x = self.y = 0
        pass
    def update(self,x):
        x,y = x[:2,0]
        self.x,self.y = CX + m2p(x), CY - m2p(y)
    def draw(self,screen):
        pygame.draw.circle(screen, BLACK, (int(self.x),int(self.y)), 5)
        

class Display(object):
    def __init__(self, w,h):
        pygame.init()
        self.w,self.h = w,h
        self.screen = pygame.display.set_mode((self.w,self.h))
        pygame.display.set_caption('EKF Localization')
    def draw(self,objects):
        self.screen.fill(WHITE)
        for o in objects:
            o.draw(self.screen)

        # control panel
        pygame.draw.ellipse(self.screen, GREEN, (self.w/2, self.h/2, self.w/2, self.h/2),2)
        pygame.draw.ellipse(self.screen, GREEN, (self.w * 3/4 - 5, self.h * 3/4 - 5, 10, 10),2)

        pygame.display.flip()
    def update(self):
        pygame.display.update()

def quit():
    for event in pygame.event.get():
        if event.type == QUIT:
            return True
    return False

if __name__ == "__main__":
    disp = Display()
    x = np.random.random((5,1))
    r = RobotObject(x, (255,128,128))

    while True:
        if quit():
            break
        x[2,0] += (np.pi / 64)
        r.update(x)
        disp.draw([r])
        disp.update()
        pygame.time.wait(100)
