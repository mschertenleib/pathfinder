from multiprocessing.connection import wait
import matplotlib.pyplot as plt
import numpy as np
import sys
import serial
import time
import struct
import os
import EPuck2

SPCM = 1000/13      #steps per cm 
RAD = 2*10000       #unit correction
ESP_D = 5.5         #distance between wheel contact points
APRPP = 3           #nb approx per line for bezier

size = 50           #size of the map in cm

original_stdout = sys.stdout

instrfile = sys.argv[1][2:]
APRPP = int(sys.argv[2])
timecom = int(sys.argv[3])*1000
print(timecom)

################################# Internal Functions ################################# 

def recursive_bezier(points,t):
        if(len(points) != 1):
            new_points = []
            for i in range(len(points)-1) :
                px = points[i][0] + t*(points[i+1][0]-points[i][0])
                py = points[i][1] + t*(points[i+1][1]-points[i][1])
                new_points.append([px,py])
            return recursive_bezier(new_points,t)
        else:
            return points[0]

def angle_points(a0,a1,b0,b1):
    v1 = [a1[0]-a0[0],a1[1]-a0[1]]
    v2 = [b1[0]-b0[0],b1[1]-b0[1]]
    nv1 = [v1[1],-v1[0]]
    if np.dot(nv1,v2) >= 0:
        return np.arccos((v1[0]*v2[0] + v1[1]*v2[1])/(norm(v1) * norm(v2)))
    else :
        return -np.arccos((v1[0]*v2[0] + v1[1]*v2[1])/(norm(v1) * norm(v2)))
def norm(vect):
    return np.sqrt(vect[0]**2 + vect[1]**2)

def dist(a,b):
    return np.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)

def circle_center_from_3_points(p1, p2, p3):
    z1 = p1[0] + p1[1]*1j
    z2 = p2[0] + p2[1]*1j
    z3 = p3[0] + p3[1]*1j

    if (z1 == z2) or (z2 == z3) or (z3 == z1):
        raise ValueError(f'Duplicate points: {p1},{p2},{p3}')
        
    w = (z3 - z1)/(z2 - z1)
    
    if w.imag == 0:
        return p1 #if colinear just sends first point
        
    c = (z2 - z1)*(w - abs(w)**2)/(2j*w.imag) + z1;  # Simplified denominator
    
    return [c.real,c.imag]

################################# Class def ################################# 

class Move:
    def __init__(self):
        self.data = [[0,0]]
        self.path = []
        self.command = []
        self.wspace = ESP_D
        self.color = "#000000"

    def clear(self):
        self.data.clear()
        self.data.append([0,0])
        self.path.clear()
        self.command.clear()

    def show_path(self):
        x = []
        y = []
        for point in self.path:
            x.append(point[0])
            y.append(point[1])
        plt.plot(x,y,c="#ff0000")
        plt.show()

    def gen_bezier_path(self,steps):
        steps *= (len(self.data)-1)
        self.path.clear()
        for i in range(steps+1):
            t = i/steps
            self.path.append(recursive_bezier(self.data,t))

    def gen_stg_command(self,time):
        self.path.clear()
        nbcom = (len(self.data)-1)*2
        tps = int(time/nbcom)
        print(time,len(self.data),nbcom,tps)
        self.command.clear()

        for i in range(len(self.data)-1) : 
            #turning phase
            if i == 0 :
                alpha = angle_points([0,0],[0,1],self.data[0],self.data[1])
            else:
                alpha = angle_points(self.data[i-1],self.data[i],self.data[i],self.data[i+1])
            #print("angl %d : %2f"%(i,alpha*(180/np.pi)))
            l = int((((ESP_D/2)*alpha)*SPCM*1000)/tps)
            r = int((-((ESP_D/2)*alpha)*SPCM*1000)/tps)
            self.command.append([l,r,tps])
            #forward phase
            spd = int((dist(self.data[i],self.data[i+1])*SPCM*1000)/tps)
            self.command.append([spd,spd,tps])

    def smooth_line(self):
        print("to be done")

    def pathlen(self):
        acc = 0
        for i in range(len(self.path) -1):
            acc += dist(self.path[i],self.path[i+1])
        return acc

    def genfile(self,filename):
        f = open(filename, 'w')
        sys.stdout = f # Change the standard output to the file we created.
        print('MOVE')
        nb_word = len(self.command)*3
        print(nb_word)

        for step in self.command:
            cmdline = str(step)[1:len(str(step))-1]
            lspd,rspd,stime = cmdline.split(", ")
            print(lspd,rspd,stime)
        
        print("END %.2f %.2f"%(self.data[-1][0],self.data[-1][1]))
        sys.stdout = original_stdout # Reset the standard output to its original value

    def gen_bez_command(self,time):
        self.command.clear()
        timestep = (int)(time/((len(self.path)/2 -1)))
        
        for i in range(0,len(self.path)-2,2):
            center = circle_center_from_3_points(self.path[i],self.path[i+1],self.path[i+2])
            if(center == self.path[i]): #if goes straight
                Rtrav = dist(self.path[i],self.path[i+2])
                Ltrav = int(Rtrav*SPCM*(1000/timestep))
                self.command.append([Ltrav,Ltrav,timestep])

            else:
                rad = dist(self.path[i],center)
                angle = 2*np.arcsin((dist(self.path[i],self.path[i+2]))/(2*rad))
                #print("angle = {:.2f}deg".format(angle*(180/np.pi)))
                Pvect = [(self.path[i+1][1]-self.path[i][1]),-(self.path[i+1][0]-self.path[i][0])]  #vect perp au deux premier point dir droite
                Rvect = [center[0]-self.path[i][0],center[1]-self.path[i][1]]             #vect point i in center

                if np.dot(Pvect,Rvect) > 0 :                                    #determine si centre a droite
                    Rtrav = (rad - (self.wspace/2))*angle
                    Ltrav = (rad + (self.wspace/2))*angle
                    a = (int)(Ltrav*SPCM*(1000/timestep))
                    b = (int)(Rtrav*SPCM*(1000/timestep))
                    c = timestep
                    self.command.append([a,b,c])
            
                else :                                                          # ou gauche
                    Rtrav = (rad + (self.wspace/2))*angle
                    Ltrav = (rad - (self.wspace/2))*angle
                    a = (int)(Ltrav*SPCM*(1000/timestep))
                    b = (int)(Rtrav*SPCM*(1000/timestep))
                    c = timestep
                    self.command.append([a,b,c])



################################# communication ################################# 

def send_instr(file,port):
    ser = serial.Serial(port, 57600, timeout=5)
    if(not ser.is_open):
        ser.open()

    file = open(file,"r")

    command = []

    instr = file.readline().rstrip("\n")
    len = int(file.readline().rstrip("\n"))
    for i in range(len):
        for inf in file.readline().split():
            if(inf == "END"):
                print("finished reading",instrfile)
                break
            command.append(int(inf))

    for c in instr :                    #send instr like "MOVE"
        ser.write(c.encode('utf-8'))

    time.sleep(1)                       #send size of data
    ser.write(struct.pack(">h",len))


    for inf in command:                 #sends data
        ser.write(struct.pack(">h",inf))

    for c in "END":
        ser.write(c.encode('utf-8'))

    print("sent:",instr,len,command)
    confirm = ser.read(1).decode("ascii")
    print(confirm)
    if(confirm == 'c'):
        print(" Confirmed !")
    else:
        print(" epuck didn't listen. again. ")
    ser.close()


def listen_ser(port,time):
    print("Opening ",port," and reading :")
    ser = serial.Serial(port, 57600, timeout=1)
    if(not ser.is_open):
        ser.open()
    i = 0
    while i < time:
        bytesToRead = ser.inWaiting()
        serstr = ser.read(bytesToRead)
        if serstr != b'':
            i= 0
            ret = serstr.decode("ascii")
            print("->",ret)
            if ret.find("DONE") != -1:
                break
        else:
            i += 1
    print("stopped listening.",i)
    ser.close()

################################# Events Functions #################################

def onclick(event):
    plt.clf()
    global ix, iy
    #ix, iy = int(event.xdata), int(event.ydata)
    ix, iy = event.xdata, event.ydata
    print ('x = %.2f, y = %.2f'%(ix, iy))
    current_move.data.append([ix,iy])
    x = []
    y = []
    for point in current_move.data:
        x.append(point[0])
        y.append(point[1])
    plt.plot(x,y,c="#00ff55")
    plt.xlim([-50,50])
    plt.ylim([-50,50])
    plt.show()

def on_press(event):
    #print('press', event.key)
    sys.stdout.flush()
    if event.key == 'x':
        send_instr(instrfile,"COM6")
        listen_ser("COM6",1e6)
    elif event.key == 'l':
        listen_ser("COM6",1e6)
    elif event.key == 'r':
        robot.set(0,0,0)
        robot.simulate(instrfile)
    elif event.key == 'c':
        print("Clearing move and robot")
        current_move.clear()
        robot.set(0,0,0)
    elif event.key == 'b':
        print("generating bezier command")
        current_move.gen_bezier_path(APRPP)
        current_move.gen_bez_command(timecom)
        current_move.show_path()
        #print(current_move.command)
        current_move.genfile(instrfile)
    elif event.key == 'n':
        print("generating stop turn go command")
        current_move.gen_stg_command(timecom)
        #print(current_move.command)
        current_move.genfile(instrfile)


################################# Main #################################

robot = EPuck2.Epuck(0,0,0)
current_move = Move()

fig = plt.figure(figsize=(5,5),edgecolor="#000000",facecolor="#ffffff")

ax = fig.add_subplot(111)
ax.set_aspect('equal','box')
plt.xlim([-size,size])
plt.ylim([-size,size])

fig.canvas.mpl_connect('button_press_event', onclick)
fig.canvas.mpl_connect('key_press_event', on_press)

plt.show()