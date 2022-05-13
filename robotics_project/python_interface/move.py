import EPuck2
import sys
import numpy as np


def recursive_bezier(points, t):
    if len(points) != 1:
        new_points = []
        for i in range(len(points) - 1):
            px = points[i][0] + t * (points[i + 1][0] - points[i][0])
            py = points[i][1] + t * (points[i + 1][1] - points[i][1])
            new_points.append((px, py))
        return recursive_bezier(new_points, t)
    else:
        return points[0]


def angle_vecs(v1, v2):
    if np.dot(v1, (v2[1], -v2[0])) >= 0:
        return np.arccos(np.dot(v1, v2) / (norm(v1) * norm(v2)))
    else:
        return -np.arccos(np.dot(v1, v2) / (norm(v1) * norm(v2)))


def angle_points(a0, a1, b0, b1):
    v1 = np.subtract(a1, a0)
    v2 = np.subtract(b1, b0)
    return angle_vecs(v1, v2)


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

    c = (z2 - z1) * (w - abs(w)**2) / \
        (2j * w.imag) + z1  # Simplified denominator

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

    def gen_straight_command(self, distance_mm, steps_per_second):
        distance_steps = distance_mm / EPuck2.EPuck2.MM_PER_STEP
        duration_ms = int(distance_steps / steps_per_second * 1000)
        self.command.append(
            (int(steps_per_second), int(steps_per_second), duration_ms))

    def gen_in_place_turn_command(self, angle_rad, steps_per_second):
        travel_mm = EPuck2.EPuck2.WHEEL_SPACING_MM / 2 * abs(angle_rad)
        travel_steps = travel_mm / EPuck2.EPuck2.MM_PER_STEP
        duration_ms = int(travel_steps / steps_per_second * 1000)
        steps_per_second_left = - \
            int(steps_per_second) if angle_rad > 0 else int(steps_per_second)
        steps_per_second_right = int(
            steps_per_second) if angle_rad > 0 else -int(steps_per_second)
        self.command.append(
            (steps_per_second_left, steps_per_second_right, duration_ms))

    def gen_turn_command(self, radius_mm, angle_rad, steps_per_second):
        # NOTE: A positive radius means the turn is to the right
        
        distance_center_mm = radius_mm * angle_rad
        
        if radius_mm >= 0:
            distance_left_mm = (radius_mm + (EPuck2.EPuck2.WHEEL_SPACING_MM / 2)) * angle_rad
            distance_right_mm = (radius_mm - (EPuck2.EPuck2.WHEEL_SPACING_MM / 2)) * angle_rad
        else:
            distance_left_mm = (radius_mm - (EPuck2.EPuck2.WHEEL_SPACING_MM / 2)) * angle_rad
            distance_right_mm = (radius_mm + (EPuck2.EPuck2.WHEEL_SPACING_MM / 2)) * angle_rad

        distance_center_steps = distance_center_mm / EPuck2.EPuck2.MM_PER_STEP
        distance_left_steps = distance_left_mm / EPuck2.EPuck2.MM_PER_STEP
        distance_right_steps = distance_right_mm / EPuck2.EPuck2.MM_PER_STEP
        duration_s = abs(distance_center_steps / steps_per_second)
        steps_per_second_left = int(distance_left_steps / duration_s)
        steps_per_second_right = int(distance_right_steps / duration_s)
        duration_ms = int(duration_s * 1000)
        self.command.append((steps_per_second_left, steps_per_second_right, duration_ms))


    def gen_stg_command(self, steps_per_second, current_robot_angle_rad):

        self.command.clear()

        for i in range(len(self.data) - 1):

            # Turning phase
            if i == 0:
                angle_rad = angle_points((0, 0), (np.cos(current_robot_angle_rad), np.sin(
                    current_robot_angle_rad)), self.data[0], self.data[1])
            else:
                angle_rad = angle_points(
                    self.data[i - 1], self.data[i], self.data[i], self.data[i + 1])

            self.gen_in_place_turn_command(angle_rad, steps_per_second)

            # Forward phase
            self.gen_straight_command(
                dist(self.data[i], self.data[i + 1]), steps_per_second)

    def gen_smooth_turn_command(self, steps_per_second, current_robot_angle_rad):

        self.command.clear()

        # First align with the direction of the first segment
        angle_rad = angle_points((0, 0), (np.cos(current_robot_angle_rad), np.sin(
            current_robot_angle_rad)), self.data[0], self.data[1])
        self.gen_in_place_turn_command(angle_rad, steps_per_second)

        for i in range(1, len(self.data) - 1):
            vec_segment_1 = np.subtract(self.data[i], self.data[i - 1])
            vec_segment_2 = np.subtract(self.data[i + 1], self.data[i])
            length_segment_1_mm = norm(vec_segment_1)
            length_segment_2_mm = norm(vec_segment_2)
            max_tangent_length_mm = min(
                length_segment_1_mm / 2, length_segment_2_mm / 2)
            angle_rad = angle_vecs(vec_segment_1, vec_segment_2)
            turn_radius_mm = max_tangent_length_mm * np.tan(angle_rad / 2)
            self.gen_turn_command(turn_radius_mm, angle_rad, steps_per_second)

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

    def gen_file(self, filename):
        f = open(filename, 'w')
        original_stdout = sys.stdout
        sys.stdout = f  # Change the standard output to the file we created.
        print('!MOVE')
        nb_word = len(self.command)
        print(nb_word)

        for step in self.command:
            cmdline = str(step)[1:len(str(step))-1]
            lspd, rspd, stime = cmdline.split(', ')
            print(lspd, rspd, stime)

        print('END %.2f %.2f' % (self.data[-1][0], self.data[-1][1]))
        sys.stdout = original_stdout  # Reset the standard output to its original value

    def gen_bez_command(self, steps_per_second, current_robot_angle_rad):
        self.command.clear()

        if len(self.path) <= 2:
            return

        # First align with the direction of the first segment
        angle_rad = angle_points((0, 0), (np.cos(current_robot_angle_rad), np.sin(
            current_robot_angle_rad)), self.path[0], self.path[1])
        self.gen_in_place_turn_command(angle_rad, steps_per_second)

        for i in range(0, len(self.path) - 2, 2):
            center = circle_center_from_3_points(
                self.path[i], self.path[i + 1], self.path[i + 2])

            if center == self.path[i]:  # If goes straight
                self.gen_straight_command(
                    dist(self.path[i], self.path[i + 2]), steps_per_second)

            else:
                rad = dist(self.path[i], center)
                angle = 2 * \
                    np.arcsin((dist(self.path[i], self.path[i+2])) / (2 * rad))
                # Vector perpendicular to the first two points, pointing to the right
                Pvect = ((self.path[i + 1][1] - self.path[i][1]
                          ), -(self.path[i + 1][0] - self.path[i][0]))
                # Vector from point i to center
                Rvect = (center[0] - self.path[i][0],
                         center[1] - self.path[i][1])

                if np.dot(Pvect, Rvect) > 0:  # Center to the right
                    Rtrav_mm = (
                        rad - (EPuck2.EPuck2.WHEEL_SPACING_MM / 2)) * angle
                    Ltrav_mm = (
                        rad + (EPuck2.EPuck2.WHEEL_SPACING_MM / 2)) * angle

                else:  # Center to the left
                    Rtrav_mm = (
                        rad + (EPuck2.EPuck2.WHEEL_SPACING_MM / 2)) * angle
                    Ltrav_mm = (
                        rad - (EPuck2.EPuck2.WHEEL_SPACING_MM / 2)) * angle

                duration_ms = int(30000 / len(self.path)) * 2
                steps_per_second_left = int(
                    Ltrav_mm / EPuck2.EPuck2.MM_PER_STEP * 1000 / duration_ms)
                steps_per_second_right = int(
                    Rtrav_mm / EPuck2.EPuck2.MM_PER_STEP * 1000 / duration_ms)
                self.command.append(
                    (steps_per_second_left, steps_per_second_right, duration_ms))
