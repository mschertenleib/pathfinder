import cv2
from matplotlib.backend_bases import MouseButton
import environment_map as envmap
import EPuck2
import os
import struct
import time
import serial
import sys
import numpy as np
from matplotlib.widgets import Button
import matplotlib.pyplot as plt
import matplotlib.units as units
from multiprocessing.connection import wait
import matplotlib
matplotlib.use('TkAgg')

SPCM = 1000/13  # steps per cm
RAD = 2*10000  # unit correction
ESP_D = 5.5  # distance between wheel contact points
APRPP = 3  # nb approx per line for bezier

size = 50  # size of the map in cm

original_stdout = sys.stdout

instrfile = 'instructions.txt'  # sys.argv[1][2:]
#APRPP = int(sys.argv[2])
timecom = 1000  # int(sys.argv[3])*1000
print(timecom)

################################# Internal Functions #################################


def recursive_bezier(points, t):
    if(len(points) != 1):
        new_points = []
        for i in range(len(points)-1):
            px = points[i][0] + t*(points[i+1][0]-points[i][0])
            py = points[i][1] + t*(points[i+1][1]-points[i][1])
            new_points.append([px, py])
        return recursive_bezier(new_points, t)
    else:
        return points[0]


def angle_points(a0, a1, b0, b1):
    v1 = [a1[0]-a0[0], a1[1]-a0[1]]
    v2 = [b1[0]-b0[0], b1[1]-b0[1]]
    nv1 = [v1[1], -v1[0]]
    if np.dot(nv1, v2) >= 0:
        return np.arccos((v1[0]*v2[0] + v1[1]*v2[1])/(norm(v1) * norm(v2)))
    else:
        return -np.arccos((v1[0]*v2[0] + v1[1]*v2[1])/(norm(v1) * norm(v2)))


def norm(vect):
    return np.sqrt(vect[0]**2 + vect[1]**2)


def dist(a, b):
    return np.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)


def circle_center_from_3_points(p1, p2, p3):
    z1 = p1[0] + p1[1]*1j
    z2 = p2[0] + p2[1]*1j
    z3 = p3[0] + p3[1]*1j

    if (z1 == z2) or (z2 == z3) or (z3 == z1):
        raise ValueError(f'Duplicate points: {p1},{p2},{p3}')

    w = (z3 - z1)/(z2 - z1)

    if w.imag == 0:
        return p1  # if colinear just sends first point

    c = (z2 - z1)*(w - abs(w)**2)/(2j*w.imag) + z1  # Simplified denominator

    return [c.real, c.imag]

################################# Class def #################################


class Move:
    def __init__(self):
        self.data = [[0, 0]]
        self.path = []
        self.command = []
        self.wspace = ESP_D
        self.color = "#000000"

    def clear(self):
        self.data.clear()
        self.data.append([0, 0])
        self.path.clear()
        self.command.clear()

    def show_path(self):
        x = []
        y = []
        for point in self.path:
            x.append(point[0])
            y.append(point[1])
        plt.plot(x, y, c="#ff0000")
        plt.show()

    def gen_bezier_path(self, steps):
        steps *= (len(self.data)-1)
        self.path.clear()
        for i in range(steps+1):
            t = i/steps
            self.path.append(recursive_bezier(self.data, t))

    def gen_stg_command(self, time):
        self.path.clear()
        nbcom = (len(self.data)-1)*2
        tps = int(time/nbcom)
        print(time, len(self.data), nbcom, tps)
        self.command.clear()

        for i in range(len(self.data)-1):
            # turning phase
            if i == 0:
                alpha = angle_points(
                    [0, 0], [0, 1], self.data[0], self.data[1])
            else:
                alpha = angle_points(
                    self.data[i-1], self.data[i], self.data[i], self.data[i+1])
            #print("angl %d : %2f"%(i,alpha*(180/np.pi)))
            l = int((((ESP_D/2)*alpha)*SPCM*1000)/tps)
            r = int((-((ESP_D/2)*alpha)*SPCM*1000)/tps)
            self.command.append([l, r, tps])
            # forward phase
            spd = int((dist(self.data[i], self.data[i+1])*SPCM*1000)/tps)
            self.command.append([spd, spd, tps])

    def smooth_line(self):
        print("to be done")

    def pathlen(self):
        acc = 0
        for i in range(len(self.path) - 1):
            acc += dist(self.path[i], self.path[i+1])
        return acc

    def genfile(self, filename):
        f = open(filename, 'w')
        sys.stdout = f  # Change the standard output to the file we created.
        print('MOVE')
        nb_word = len(self.command)*3
        print(nb_word)

        for step in self.command:
            cmdline = str(step)[1:len(str(step))-1]
            lspd, rspd, stime = cmdline.split(", ")
            print(lspd, rspd, stime)

        print("END %.2f %.2f" % (self.data[-1][0], self.data[-1][1]))
        sys.stdout = original_stdout  # Reset the standard output to its original value

    def gen_bez_command(self, time):
        self.command.clear()
        timestep = (int)(time/((len(self.path)/2 - 1)))

        for i in range(0, len(self.path)-2, 2):
            center = circle_center_from_3_points(
                self.path[i], self.path[i+1], self.path[i+2])
            if(center == self.path[i]):  # if goes straight
                Rtrav = dist(self.path[i], self.path[i+2])
                Ltrav = int(Rtrav*SPCM*(1000/timestep))
                self.command.append([Ltrav, Ltrav, timestep])

            else:
                rad = dist(self.path[i], center)
                angle = 2 * \
                    np.arcsin((dist(self.path[i], self.path[i+2]))/(2*rad))
                #print("angle = {:.2f}deg".format(angle*(180/np.pi)))
                # vect perp au deux premier point dir droite
                Pvect = [(self.path[i+1][1]-self.path[i][1]), -
                         (self.path[i+1][0]-self.path[i][0])]
                Rvect = [center[0]-self.path[i][0], center[1] -
                         self.path[i][1]]  # vect point i in center

                if np.dot(Pvect, Rvect) > 0:  # determine si centre a droite
                    Rtrav = (rad - (self.wspace/2))*angle
                    Ltrav = (rad + (self.wspace/2))*angle
                    a = (int)(Ltrav*SPCM*(1000/timestep))
                    b = (int)(Rtrav*SPCM*(1000/timestep))
                    c = timestep
                    self.command.append([a, b, c])

                else:                                                          # ou gauche
                    Rtrav = (rad + (self.wspace/2))*angle
                    Ltrav = (rad - (self.wspace/2))*angle
                    a = (int)(Ltrav*SPCM*(1000/timestep))
                    b = (int)(Rtrav*SPCM*(1000/timestep))
                    c = timestep
                    self.command.append([a, b, c])


################################# communication #################################

def send_instr(file, port):
    ser = serial.Serial(port, 57600, timeout=5)
    if(not ser.is_open):
        ser.open()

    file = open(file, "r")

    command = []

    instr = file.readline().rstrip("\n")
    len = int(file.readline().rstrip("\n"))
    for i in range(len):
        for inf in file.readline().split():
            if(inf == "END"):
                print("finished reading", instrfile)
                break
            command.append(int(inf))

    for c in instr:  # send instr like "MOVE"
        ser.write(c.encode('utf-8'))

    time.sleep(1)  # send size of data
    ser.write(struct.pack(">h", len))

    for inf in command:  # sends data
        ser.write(struct.pack(">h", inf))

    for c in "END":
        ser.write(c.encode('utf-8'))

    print("sent:", instr, len, command)
    confirm = ser.read(1).decode("ascii")
    print(confirm)
    if(confirm == 'c'):
        print(" Confirmed !")
    else:
        print(" epuck didn't listen. again. ")
    ser.close()


def listen_ser(port, time):
    print("Opening ", port, " and reading :")
    ser = serial.Serial(port, 57600, timeout=1)
    if(not ser.is_open):
        ser.open()
    i = 0
    while i < time:
        bytesToRead = ser.inWaiting()
        serstr = ser.read(bytesToRead)
        if serstr != b'':
            i = 0
            ret = serstr.decode("ascii")
            print("->", ret)
            if ret.find("DONE") != -1:
                break
        else:
            i += 1
    print("stopped listening.", i)
    ser.close()

################################# Events Functions #################################


def on_mouse_button_press(event):
    if not event.inaxes:
        return

    if event.inaxes is map_ax:
        if event.button is MouseButton.LEFT:
            x_mm, y_mm = event.xdata, event.ydata
            constructed_map.set_occupied(x_mm, y_mm)

        elif event.button is MouseButton.RIGHT:
            global ix, iy
            ix, iy = event.xdata, event.ydata
            print('x = %.2f, y = %.2f' % (ix, iy))
            current_move.data.append([ix, iy])
            x = []
            y = []
            for point in current_move.data:
                x.append(point[0])
                y.append(point[1])
            map_ax.plot(x, y, c="#00ff55")


def action_send_instruction():
    send_instr(instrfile, "COM6")
    listen_ser("COM6", 1e6)


def action_listen():
    listen_ser("COM6", 1e6)


def action_restart():
    robot.set(0, 0, 0)
    robot.simulate(instrfile)


def action_clear():
    print("Clearing move and robot")
    current_move.clear()
    robot.set(0, 0, 0)


def action_bezier():
    print("Generating Bezier command")
    current_move.gen_bezier_path(APRPP)
    current_move.gen_bez_command(timecom)
    current_move.show_path()
    # print(current_move.command)
    current_move.genfile(instrfile)


def action_stg():
    print("Generating stop-turn-go command")
    current_move.gen_stg_command(timecom)
    # print(current_move.command)
    current_move.genfile(instrfile)


def on_key_press(event):
    print('Pressed', event.key)
    sys.stdout.flush()
    if event.key == 'x':
        action_send_instruction()
    elif event.key == 'l':
        action_listen()
    elif event.key == 'r':
        action_restart()
    elif event.key == 'c':
        action_clear()
    elif event.key == 'b':
        action_bezier()
    elif event.key == 'n':
        action_stg()


def construct_map():
    robot_x_mm, robot_y_mm, robot_angle_rad, distance_mm = next(data_generator)
    robot.set(robot_x_mm, robot_y_mm, robot_angle_rad)
    constructed_map.construct((robot_x_mm, robot_y_mm), robot_angle_rad, EPuck2.Epuck.RADIUS_MM,
                              distance_mm, EPuck2.Epuck.TOF_SENSOR_OFFSET_MM, EPuck2.Epuck.TOF_MAX_DISTANCE_MM)


def update():    
    map_image = constructed_map.to_image(height=height_px, width=width_px)

    img = cv2.merge([map_image] * 3)

    goal_mm = (width_mm//4, height_mm*5//6)
    cv2.rectangle(img, goal_mm, np.add(
        goal_mm, (cell_size_mm, cell_size_mm)), (1, 0, 0))
    path = constructed_map.find_path(
        (robot.x_mm, robot.y_mm), goal_mm, WALKABLE_MIN_RADIUS)
    if len(path) >= 2:
        for i in range(len(path)-1):
            start = (int(path[i][0]), int(path[i][1]))
            end = (int(path[i+1][0]), int(path[i+1][1]))
            cv2.line(img, start, end, (0, 0, 1))
    
    map_ax.clear()
    map_ax.imshow(img)
    robot.draw(map_ax, color="#00ff00")
    map_ax.invert_yaxis()
    
    walkable_ax.clear()
    walkable_ax.imshow(constructed_map.walkable(WALKABLE_MIN_RADIUS, height_px, width_px), cmap='gray')
    walkable_ax.invert_yaxis()
    
    fig.canvas.draw()
    fig.canvas.flush_events()


def on_update_button_clicked(event):
    update()


def on_send_instruction_button_clicked(event):
    action_send_instruction()


def on_listen_button_clicked(event):
    action_listen()


def on_restart_button_clicked(event):
    action_restart()


def on_clear_button_clicked(event):
    action_clear()


def on_bezier_button_clicked(event):
    action_bezier()


def on_stg_button_clicked(event):
    action_stg()


if __name__ == '__main__':

    width_mm = 640
    height_mm = 480
    cell_size_mm = 10
    width_px = width_mm
    height_px = height_mm
    
    WALKABLE_MIN_RADIUS = EPuck2.Epuck.RADIUS_MM * 1.5

    robot = EPuck2.Epuck(x_mm=width_mm/2, y_mm=height_mm/2)
    current_move = Move()

    true_map = envmap.Environment_map(width_mm, height_mm, cell_size_mm=1)
    true_map.set_all_free()
    true_map.set_occupied_line_normalized((0.2, 0.3), (0.8, 0.1))
    true_map.set_occupied_line_normalized((0.5, 0.7), (0.8, 0.7))
    true_map.set_occupied_line_normalized((0.1, 0.1), (0.1, 0.9))
    true_map.set_occupied_line_normalized((0.7, 0.3), (0.7, 0.7))
    true_map.set_occupied_rectangle_normalized((0.38, 0.65), (0.4, 1))

    constructed_map = envmap.Environment_map(
        width_mm=width_mm, height_mm=height_mm, cell_size_mm=cell_size_mm)

    data_generator = true_map.robot_data_generator(
        EPuck2.Epuck.TOF_SENSOR_OFFSET_MM, EPuck2.Epuck.TOF_MAX_DISTANCE_MM, True)

    # Create figures and subplots
    fig = plt.figure()
    map_ax = fig.add_subplot(1, 2, 1)
    map_ax.set_title('Map')
    walkable_ax = fig.add_subplot(1, 2, 2)
    walkable_ax.set_title('Walkable')
    
    for ax in (map_ax, walkable_ax):
        ax.set_aspect('equal', 'box')
        ax.set_xlim([0, width_mm])
        ax.set_ylim([0, height_mm])
        ax.set_xlabel('millimeters')
        ax.set_ylabel('millimeters')

    # Create figures and subplots
    #fig = plt.figure()
    #map_ax = fig.add_subplot()
    #map_ax.set_title('Map')
    #map_ax.set_aspect('equal', 'box')
    #map_ax.set_xlim([0, width_mm])
    #map_ax.set_ylim([0, height_mm])
    #map_ax.set_xlabel('millimeters')
    #map_ax.set_ylabel('millimeters')
    #plt.subplots_adjust(bottom=0.2)

    # Create buttons
    button_color = 'lightgoldenrodyellow'
    button_hovercolor = '0.975'

    BUTTON_Y = 0.025
    BUTTON_WIDTH = 0.1
    BUTTON_HEIGHT = 0.04
    BUTTON_MARGIN = 0.02
    NUM_BUTTONS = 7
    BUTTON_SPACING = BUTTON_WIDTH + BUTTON_MARGIN

    update_button_ax = plt.axes(
        [BUTTON_MARGIN, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    send_instruction_button_ax = plt.axes(
        [BUTTON_MARGIN + BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    listen_button_ax = plt.axes(
        [BUTTON_MARGIN + 2 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    restart_button_ax = plt.axes(
        [BUTTON_MARGIN + 3 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    clear_button_ax = plt.axes(
        [BUTTON_MARGIN + 4 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    bezier_button_ax = plt.axes(
        [BUTTON_MARGIN + 5 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
    stg_button_ax = plt.axes(
        [BUTTON_MARGIN + 6 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])

    update_button = Button(update_button_ax, 'Update',
                           color=button_color, hovercolor=button_hovercolor)
    send_instruction_button = Button(
        send_instruction_button_ax, 'Send', color=button_color, hovercolor=button_hovercolor)
    listen_button = Button(listen_button_ax, 'Listen',
                           color=button_color, hovercolor=button_hovercolor)
    restart_button = Button(restart_button_ax, 'Restart',
                            color=button_color, hovercolor=button_hovercolor)
    clear_button = Button(clear_button_ax, 'Clear',
                          color=button_color, hovercolor=button_hovercolor)
    bezier_button = Button(bezier_button_ax, 'Bezier',
                           color=button_color, hovercolor=button_hovercolor)
    stg_button = Button(stg_button_ax, 'STG',
                        color=button_color, hovercolor=button_hovercolor)

    update_button.on_clicked(on_update_button_clicked)
    send_instruction_button.on_clicked(on_send_instruction_button_clicked)
    listen_button.on_clicked(on_listen_button_clicked)
    restart_button.on_clicked(on_restart_button_clicked)
    clear_button.on_clicked(on_clear_button_clicked)
    bezier_button.on_clicked(on_bezier_button_clicked)
    stg_button.on_clicked(on_stg_button_clicked)

    # Connect events
    fig.canvas.mpl_connect('button_press_event', on_mouse_button_press)
    fig.canvas.mpl_connect('key_press_event', on_key_press)
    
    timer = fig.canvas.new_timer(interval=10)
    timer.add_callback(construct_map)
    timer.start()
    
    timer = fig.canvas.new_timer(interval=200)
    timer.add_callback(update)
    timer.start()

    # Start the matplotlib main
    plt.show()
