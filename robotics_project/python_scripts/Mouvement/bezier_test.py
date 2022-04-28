
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import matplotlib.animation as anim
import numpy as np
import sys
import serial
import time
import struct
import os
import graphics

SPCM = 1000/13

original_stdout = sys.stdout

instrfile = "SpdInstr.txt"

points = [[0,0]]
nbApprox = 9
timecom = 10000

RAD = 2*10000
ESP_D = 5.5

def send_instr(file,port):
    ser = serial.Serial(port, 19200, timeout=5)
    if(not ser.is_open):
        ser.open()

    file = open(file,"r")

    command = []

    instr = file.readline().rstrip("\n")
    len = int(file.readline().rstrip("\n"))
    for i in range(len):
        for inf in file.readline().split():
            command.append(int(inf))

    print("sending:",instr,len,command)

    for c in instr :                    #send instr like "MOVE"
        ser.write(c.encode('utf-8'))

    time.sleep(1)                       #send size of data
    ser.write(struct.pack(">H",len))


    for inf in command:                 #sends data
        ser.write(struct.pack(">H",inf))

    confirm = ser.read(1)
    print(confirm)
    if(confirm == 'o'):
        print(" Confirmed !")
    else:
        print(" epuck didn't listen. again. ")
    

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

def generate_path(points,steps):
    bezier = []
    steps -= 1
    for i in range(steps+1):
        t = i/steps
        #print("i = ",i,", nb = ",steps,", t = ",t)
        bezier.append(recursive_bezier(points,t))
    return bezier

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

def gen_command(path,rbt_width,time,filename):
    f = open(filename, 'w')
    sys.stdout = f # Change the standard output to the file we created.
    print('MOVE')
    nb_word = (int)((len(path)-1)*3/2)
    print(nb_word)

    command = []
    timestep = (int)(time/((len(path) -1)/2))
    
    for i in range(0,len(path)-2,2):
        center = circle_center_from_3_points(path[i],path[i+1],path[i+2])
        if(center == path[i]): #if goes straight
            Rtrav = dist(path[i],path[i+2])
            Ltrav = int(Rtrav*SPCM*(1000/timestep))
            command.append([Ltrav,Ltrav])
            command.append(timestep)
            print(Ltrav,Ltrav,timestep)
            
        else:
            rad = dist(path[i],center)
            angle = 2*np.arcsin((dist(path[i],path[i+2]))/(2*rad))
            #print("angle = {:.2f}deg".format(angle*(180/np.pi)))
            Pvect = [(path[i+1][1]-path[i][1]),-(path[i+1][0]-path[i][0])]  #vect perp au deux premier point dir droite
            Rvect = [center[0]-path[i][0],center[1]-path[i][1]]                                       #vect point i au centre
            circles.append([center,rad])

            if np.dot(Pvect,Rvect) > 0 :                                    #determine si centre a droite
                Rtrav = (rad - (rbt_width/2))*angle
                Ltrav = (rad + (rbt_width/2))*angle
                command.append([(int)(((RAD*2*np.pi)*Ltrav)/timestep),(int)(((RAD*2*np.pi)*Rtrav)/timestep)])
                command.append((int)(timestep))
                a = (int)(Ltrav*SPCM*(1000/timestep))
                b = (int)(Rtrav*SPCM*(1000/timestep))
                c = (int)(timestep)
                print(a,b,c)
        
            else :                                                          # ou gauche
                Rtrav = (rad + (rbt_width/2))*angle
                Ltrav = (rad - (rbt_width/2))*angle
                command.append([(int)(((RAD*2*np.pi)*Ltrav)/timestep),(int)(((RAD*2*np.pi)*Rtrav)/timestep)])
                command.append((int)(timestep))
                a = (int)(Ltrav*SPCM*(1000/timestep))
                b = (int)(Rtrav*SPCM*(1000/timestep))
                c = (int)(timestep)
                print(a,b,c)
    print("END",points[-1][0],points[-1][1])
    sys.stdout = original_stdout # Reset the standard output to its original value
    return command

def pathlen(path):
    acc = 0
    for i in range(len(path) -1):
        acc += dist(path[i],path[i+1])
    return acc

def onclick(event):
    plt.clf()
    global ix, iy
    ix, iy = event.xdata, event.ydata
    print ('x = %2f, y = %2f'%(ix, iy))
    points.append([ix,iy])
    path = generate_path(points,nbApprox)
    x = []
    y = []
    for point in path:
        x.append(point[0])
        y.append(point[1])
    plt.plot(x,y,c="#00ff55")
    plt.xlim([-50,50])
    plt.ylim([-50,50])
    plt.show()
    gen_command(path,ESP_D,timecom,instrfile)
    strt = [points[0][0],
            points[0][1],
            np.arcsin((points[1][0]-points[0][0])/dist(points[0],points[1]))]
    if points[1][1] < 0 :
        strt[2] = np.pi - strt[2]

    print(strt)
    graphics.simulate(instrfile,strt)
    
fig = plt.figure(figsize=(5,5),edgecolor="#000000",facecolor="#ffffff")

ax = fig.add_subplot(111)
ax.set_aspect('equal','box')
plt.xlim([-50,50])
plt.ylim([-50,50])

circles = []

angle = np.linspace(0,2*np.pi,100)
for circ in circles:
    px = circ[1]*np.cos(angle)
    py = circ[1]*np.sin(angle)
    px += circ[0][0]
    py += circ[0][1]
    #plt.plot(px,py,color="#aaaaaa")

cid = fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()
send_instr("SpdInstr.txt","COM8")