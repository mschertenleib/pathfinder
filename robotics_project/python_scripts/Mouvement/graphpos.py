import matplotlib.pyplot as plt
import numpy as np

k = 0
file = open("pos.txt","r")
size = 200
posx = []
posy = []
phi = []

#for inf in file:
#    if inf == "\n": continue
#    if inf.rstrip("\n") == "DONE": break
#    inf.rstrip("\n")
#    a,b,c = inf.split()
#    posx.append(float(a[1:]))
#    posy.append(float(b[1:]))
#    phi.append(float(c[1:]))
#    k = k+1

for inf in file:
    if inf == "\n": continue
    if inf.rstrip("\n") == "SCAN": continue
    if inf[0] == "X": continue
    if inf.rstrip("\n") == "MAP": break
    inf.rstrip("\n")
    a,d = inf.split()
    angle = float(a[1:])
    dist = float(d[1:])
    posx.append(dist*np.sin(angle))
    posy.append(dist*np.cos(angle))
    print(k,angle,dist)
    k = k+1
    

i = np.linspace(0,200,k)
fig = plt.figure(figsize=(5,5),edgecolor="#000000",facecolor="#ffffff")

ax = fig.add_subplot(111)
ax.set_aspect('equal','box')

#plt.plot(i,posx,c="#ff0000")
#plt.plot(i,posy,c="#0000ff")
#plt.plot(i,phi,c="#00ff00")
plt.plot(posx,posy)

plt.show()