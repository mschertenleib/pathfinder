import EPuck2
import sys
import numpy as np


def recursive_bezier(points, t):
    if len(points) != 1:
        new_points = []
        for i in range(len(points)-1):
            px = points[i][0] + t*(points[i+1][0]-points[i][0])
            py = points[i][1] + t*(points[i+1][1]-points[i][1])
            new_points.append([px, py])
        return recursive_bezier(new_points, t)
    else:
        return points[0]


def angle_points(a0, a1, b0, b1):
    v1 = np.subtract(a1, a0)
    v2 = np.subtract(b1, b0)
    if np.dot(v1, (v2[1], -v2[0])) >= 0:
        return np.arccos(np.dot(v1, v2) / (norm(v1) * norm(v2)))
    else:
        return -np.arccos(np.dot(v1, v2) / (norm(v1) * norm(v2)))


def norm(vect):
    return np.sqrt(vect[0]**2 + vect[1]**2)


def dist(a, b):
    return norm(np.subtract(b, a))


def circle_center_from_3_points(p1, p2, p3):
    z1 = p1[0] + p1[1] * 1j
    z2 = p2[0] + p2[1] * 1j
    z3 = p3[0] + p3[1] * 1j

    if (z1 == z2) or (z2 == z3) or (z3 == z1):
        raise ValueError(f'Duplicate points: {p1}, {p2}, {p3}')

    w = (z3 - z1) / (z2 - z1)

    if w.imag == 0:
        return p1  # if colinear just sends first point

    c = (z2 - z1) * (w - abs(w)**2) / (2j * w.imag) + z1  # Simplified denominator

    return (c.real, c.imag)


class Move:
    def __init__(self, x_mm, y_mm):
        self.data = [(x_mm, y_mm)]
        self.path = []
        self.command = []

    def reset_data(self, x_mm, y_mm):
        self.data = [(x_mm, y_mm)]

    def draw_path(self, ax, color):
        xs_mm = []
        ys_mm = []
        for x_mm, y_mm in self.data:
            xs_mm.append(x_mm)
            ys_mm.append(y_mm)
        ax.plot(xs_mm, ys_mm, color=color)

    def gen_stg_command(self, time_ms, current_robot_angle_rad):

        number_comms = (len(self.data) - 1) * 2
        ms_per_move_step = int(time_ms / number_comms)
        print('STG over', time_ms, 'ms,', len(self.data), 'points,',
              number_comms, 'comms,', ms_per_move_step, 'ms per move step')

        self.command.clear()

        for i in range(len(self.data) - 1):

            # Turning phase
            if i == 0:
                angle_rad = angle_points(
                    (0, 0), (np.cos(current_robot_angle_rad), np.sin(current_robot_angle_rad)), self.data[0], self.data[1])
            else:
                angle_rad = angle_points(
                    self.data[i-1], self.data[i], self.data[i], self.data[i+1])

            left_mm = -EPuck2.EPuck2.WHEEL_SPACING_MM / 2 * angle_rad
            right_mm = EPuck2.EPuck2.WHEEL_SPACING_MM / 2 * angle_rad
            left_steps = left_mm / EPuck2.EPuck2.MM_PER_STEP
            right_steps = right_mm / EPuck2.EPuck2.MM_PER_STEP
            left_steps_per_second = int(left_steps * 1000 / ms_per_move_step)
            right_steps_per_second = int(right_steps * 1000 / ms_per_move_step)
            self.command.append(
                (left_steps_per_second, right_steps_per_second, ms_per_move_step))

            # Forward phase
            distance_mm = dist(self.data[i], self.data[i+1])
            distance_steps = distance_mm / EPuck2.EPuck2.MM_PER_STEP
            steps_per_second = int(distance_steps * 1000 / ms_per_move_step)
            self.command.append(
                (steps_per_second, steps_per_second, ms_per_move_step))

    def gen_bezier_path(self, steps):
        steps *= (len(self.data) - 1)
        self.path.clear()
        if steps == 0:
            return
        for i in range(steps + 1):
            t = i / steps
            self.path.append(recursive_bezier(self.data, t))

    def pathlen(self):
        acc = 0
        for i in range(len(self.path) - 1):
            acc += dist(self.path[i], self.path[i+1])
        return acc

    def genfile(self, filename):
        f = open(filename, 'w')
        original_stdout = sys.stdout
        sys.stdout = f  # Change the standard output to the file we created.
        print('!MOVE')
        nb_word = len(self.command)*3
        print(nb_word)

        for step in self.command:
            cmdline = str(step)[1:len(str(step))-1]
            lspd, rspd, stime = cmdline.split(', ')
            print(lspd, rspd, stime)

        print('END %.2f %.2f' % (self.data[-1][0], self.data[-1][1]))
        sys.stdout = original_stdout  # Reset the standard output to its original value

    def gen_bez_command(self, time):
        self.command.clear()
        timestep = (int)(time/((len(self.path)/2 - 1)))
        
        SPCM = 10 / EPuck2.EPuck2.MM_PER_STEP
        WHEEL_SPACE_CM = EPuck2.EPuck2.WHEEL_SPACING_MM / 10

        for i in range(0, len(self.path)-2, 2):
            center = circle_center_from_3_points(
                self.path[i], self.path[i+1], self.path[i+2])
            if(center == self.path[i]):  # if goes straight
                Rtrav = dist(self.path[i], self.path[i+2])
                Ltrav = int(Rtrav*SPCM*(1000/timestep))
                self.command.append((Ltrav, Ltrav, timestep))

            else:
                rad = dist(self.path[i], center)
                angle = 2 * \
                    np.arcsin((dist(self.path[i], self.path[i+2]))/(2*rad))
                #print("angle = {:.2f}deg".format(angle*(180/np.pi)))
                # vect perp au deux premier point dir droite
                Pvect = ((self.path[i+1][1]-self.path[i][1]), -
                         (self.path[i+1][0]-self.path[i][0]))
                Rvect = (center[0]-self.path[i][0], center[1] -
                         self.path[i][1])  # vect point i in center

                if np.dot(Pvect, Rvect) > 0:  # determine si centre a droite
                    Rtrav = (rad - (WHEEL_SPACE_CM/2))*angle
                    Ltrav = (rad + (WHEEL_SPACE_CM/2))*angle
                    a = (int)(Ltrav*SPCM*(1000/timestep))
                    b = (int)(Rtrav*SPCM*(1000/timestep))
                    c = timestep
                    self.command.append((a, b, c))

                else:                                                          # ou gauche
                    Rtrav = (rad + (WHEEL_SPACE_CM/2))*angle
                    Ltrav = (rad - (WHEEL_SPACE_CM/2))*angle
                    a = (int)(Ltrav*SPCM*(1000/timestep))
                    b = (int)(Rtrav*SPCM*(1000/timestep))
                    c = timestep
                    self.command.append((a, b, c))
