import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.image as img
import numpy as np

angle = np.linspace(0,2*np.pi,100)
CM_PER_STEP = 13/1000

class Epuck:

    rradius = 3.65
    wheel_space = 5.4
    cam_angle = np.pi/4

    def __init__(self,xpos=0,ypos=0,phi=0):
        self.x = xpos
        self.y = ypos
        self.phi = phi
        self.trail = []
        
    def set(self,xpos,ypos,phi):
        self.x = xpos
        self.y = ypos
        self.phi = phi
        self.trail.clear()
    
    def check_spd(self,spd):
        if spd > 1000 :
            return 0
        elif spd < -1000 :
            return 0 
        else :
            return 1


    def draw(self,col):
        print(" drawn at {:.2f} {:.2f}".format(self.x,self.y))
        px = self.rradius*np.cos(angle)
        py = self.rradius*np.sin(angle)
        px += self.x
        py += self.y
        plt.plot(px,py,color=col) #draw outer circle

        plt.plot([self.x,self.x+self.rradius*np.sin(self.phi)],
                 [self.y,self.y+self.rradius*np.cos(self.phi)],color=col) #draw direction

        plt.plot([self.x+self.rradius*np.sin(self.phi), 
                 (self.x+self.rradius*2*np.sin(self.phi))],
                 [self.y+self.rradius*np.cos(self.phi),
                 (self.y+self.rradius*2*np.cos(self.phi))],
                 color=col)

        plt.plot([self.x+self.rradius*np.sin(self.phi), 
                 (self.x+self.rradius*2*np.sin(self.phi))+((self.rradius*np.tan(self.cam_angle/2))*np.cos(self.phi))],
                 [self.y+self.rradius*np.cos(self.phi),
                 (self.y+self.rradius*2*np.cos(self.phi))+((self.rradius*np.tan(self.cam_angle/2))*-1*np.sin(self.phi))],
                 color=col)

        plt.plot([self.x+self.rradius*np.sin(self.phi), 
                 (self.x+self.rradius*2*np.sin(self.phi))+((self.rradius*np.tan(self.cam_angle/2))*-1*np.cos(self.phi))],
                 [self.y+self.rradius*np.cos(self.phi),
                 (self.y+self.rradius*2*np.cos(self.phi))+((self.rradius*np.tan(self.cam_angle/2))*np.sin(self.phi))],
                 color=col)

    def move(self,stepl,stepr,ms):
        r = (CM_PER_STEP)/1000 #correction from steps/s to cm/ms
        spdl = stepl*r
        spdr = stepr*r
        if spdr == 0 or spdl == 0: 
            ratio = 2
        else :
            ratio = spdl/spdr
        
        if (ratio == 1):        # goes straight
            for i in range(ms):
                self.trail.append([self.x,self.y,self.phi])
                self.x += (spdl)*np.sin(self.phi)
                self.y += (spdl)*np.cos(self.phi)

        else:                   # goes around 
            radi = ((self.wheel_space)/(ratio-1))+(5.5/2)
            alpha = (spdl*ms)/(radi+ (self.wheel_space/2))
            dphi = alpha/ms
            dm = np.sin(dphi)*radi
            
            for i in range(ms):
                self.trail.append([self.x,self.y,self.phi])
                dx = dm*np.sin(self.phi+(dphi/2))
                dy = dm*np.cos(self.phi+(dphi/2))
                self.phi += dphi
                self.x += dx
                self.y += dy
        #self.draw("#dddddd")
    
    def show_trail(self,col):
        cx = []
        cy = []
        for elem in self.trail:
            #self.draw(col)
            cx.append(elem[0])
            cy.append(elem[1])
        plt.plot(cx,cy,color=col)
        self.trail_rst()

    def trail_rst(self):
        self.trail.clear()

    def read_command_file(self,filename):
        f = open(filename, 'r')
        while 1:
            command = f.readline()
            if not command : 
                break

            if command[0:4]=="MOVE":
                size = int(int(f.readline())/3)
                if size == 1:
                    size = 0
                for i in range(size):
                    a,b,c = f.readline().split(" ")
                    spdl,spdr,tms = int(a),int(b),int(c)
                    
                    if (self.check_spd(spdl) and self.check_spd(spdr)):
                        print("moving...",spdl,spdr,tms)
                        self.move(spdl,spdr,tms)
                        self.show_trail("#ff0000")
                    else : 
                        print("Invalid speed (>1000):",spdl,spdr)
                        exit()
            elif command[0:3]=="END":
                str,b,c = command.split(" ")
                supx,supy = float(b),float(c)
                print("err X: %2f Y: %2f "%(self.x - supx,self.y - supy))
                break
            else:
                print("No commands found")
                exit()
                
        f.close()
        print(filename,"done.")

    def simulate(self,instrfile):
        self.read_command_file(instrfile)
        self.draw("#000000")

        plt.show()