import EPuck2


import sys
import numpy as np


def recursive_bezier(points, t):
    """
    Used to generate a Bezier curve recursively
    """

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
    """
    Get the angle formed by two vectors
    """

    norm_v1 = norm(v1)
    norm_v2 = norm(v2)
    if norm_v1 == 0 or norm_v2 == 0:
        return 0

    if np.dot(v1, (v2[1], -v2[0])) >= 0:
        return np.arccos(np.dot(v1, v2) / (norm_v1 * norm_v2))
    else:
        return -np.arccos(np.dot(v1, v2) / (norm_v1 * norm_v2))


def angle_points(a0, a1, b0, b1):
    """
    Get the angle formed by two segments a0a1 and b0b1
    """

    v1 = np.subtract(a1, a0)
    v2 = np.subtract(b1, b0)
    return angle_vecs(v1, v2)


def norm(vect):
    """
    Get the norm of a 2D vector
    """

    return np.sqrt(vect[0]**2 + vect[1]**2)


def dist(a, b):
    """
    Get the distance between two points:
    """

    return norm(np.subtract(b, a))


def circle_center_from_3_points(p1, p2, p3):
    """
    Get the center of a circle passing through three points
    """

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
    """
    Represents a set of moves. Stores control points, from which a set of commands for the robot can be created

    Attributes
    ---
    points:
        Control points for the commands
    commands:
        Set of commands (speed_left_steps_per_second, speed_right_steps_per_second, duration_ms)
    path:
        Only used for Bezier curves, path of the robot
    """

    def __init__(self, x_mm, y_mm):
        self.points = [(x_mm, y_mm)]
        self.commands = []
        self.path = []

    def reset_points(self, x_mm, y_mm):
        """
        Resets the control points and sets the first one to (x_mm, y_mm)

        Parameters
        ---
        x_mm:
            The x coordinate to start with in millimeters
        y_mm:
            The y coordinate to start with in millimeters
        """

        self.points = [(x_mm, y_mm)]

    def draw_path(self, ax, color):
        """
        Draws the control points of the move set

        Parameters
        ---
        ax:
            The matplotlib Axes instance in which to draw
        color:
            The color to use
        """

        xs_mm = []
        ys_mm = []
        for x_mm, y_mm in self.points:
            xs_mm.append(x_mm)
            ys_mm.append(y_mm)
        ax.plot(xs_mm, ys_mm, color=color)

    def remove_adjacent_duplicate_points(self):
        """
        Removes duplicate control points that are adjacent
        """

        for i in range(len(self.points) - 1, 0, -1):
            if self.points[i] == self.points[i - 1]:
                del self.points[i]

    def gen_straight_command(self, distance_mm, steps_per_second):
        """
        Generates a straight line command with the given radius

        Parameters
        ---
        distance_mm:
            The distance to travel in millimeters
        steps_per_second:
            The speed of the wheels
        """

        if distance_mm == 0 or steps_per_second == 0:
            return

        distance_steps = distance_mm / EPuck2.EPuck2.MM_PER_STEP
        duration_ms = int(distance_steps / steps_per_second * 1000)
        self.commands.append(
            (int(steps_per_second), int(steps_per_second), duration_ms))

    def gen_in_place_turn_command(self, angle_rad, steps_per_second):
        """
        Generates an in-place turn command (wheels turn with opposite speeds)

        Parameters
        ---
        angle_rad:
            The angle of the turn in radians
        steps_per_second:
            The speed of the wheels
        """

        if angle_rad == 0 or steps_per_second == 0:
            return

        travel_mm = EPuck2.EPuck2.WHEEL_SPACING_MM / 2 * abs(angle_rad)
        travel_steps = travel_mm / EPuck2.EPuck2.MM_PER_STEP
        duration_ms = int(travel_steps / steps_per_second * 1000)
        steps_per_second_left = - \
            int(steps_per_second) if angle_rad > 0 else int(steps_per_second)
        steps_per_second_right = int(
            steps_per_second) if angle_rad > 0 else -int(steps_per_second)
        self.commands.append(
            (steps_per_second_left, steps_per_second_right, duration_ms))

    def gen_turn_command(self, radius_mm, angle_rad, steps_per_second):
        """
        Generates a turn command with the given radius

        Parameters
        ---
        radius_mm:
            The radius of the turn in millimeters
        angle_rad:
            The angle of the turn in radians
        steps_per_second:
            The speed of the fastest wheel
        """

        if angle_rad == 0 or steps_per_second == 0:
            return

        if radius_mm == 0:
            self.gen_in_place_turn_command(angle_rad, steps_per_second)
            return

        distance_left_mm = (
            radius_mm - (EPuck2.EPuck2.WHEEL_SPACING_MM / 2)) * angle_rad
        distance_right_mm = (
            radius_mm + (EPuck2.EPuck2.WHEEL_SPACING_MM / 2)) * angle_rad
        distance_left_steps = distance_left_mm / EPuck2.EPuck2.MM_PER_STEP
        distance_right_steps = distance_right_mm / EPuck2.EPuck2.MM_PER_STEP

        if abs(distance_left_steps) > abs(distance_right_steps):
            steps_per_second_left = int(steps_per_second)
            steps_per_second_right = int(
                steps_per_second / distance_left_mm * distance_right_mm)
            duration_ms = int(
                abs(distance_left_steps / steps_per_second * 1000))
            self.commands.append(
                (steps_per_second_left, steps_per_second_right, duration_ms))
        elif abs(distance_right_steps) > abs(distance_left_steps):
            steps_per_second_right = int(steps_per_second)
            steps_per_second_left = int(
                steps_per_second / distance_right_steps * distance_left_steps)
            duration_ms = int(
                abs(distance_right_steps / steps_per_second * 1000))
            self.commands.append(
                (steps_per_second_left, steps_per_second_right, duration_ms))

    def gen_stg_command(self, steps_per_second, current_robot_angle_rad):
        """
        Generates a Stop-Turn-Go command set. For each control point, the robot turns in place to align with the next one then drives straight until it reaches it

        Parameters
        ---
        steps_per_second:
            The speed of the wheels
        current_robot_angle_rad:
            The current angle of the robot in radians
        """

        self.commands.clear()

        self.remove_adjacent_duplicate_points()

        for i in range(len(self.points) - 1):

            # Turning phase
            if i == 0:
                angle_rad = angle_points((0, 0), (np.cos(current_robot_angle_rad), np.sin(
                    current_robot_angle_rad)), self.points[0], self.points[1])
            else:
                angle_rad = angle_points(
                    self.points[i - 1], self.points[i], self.points[i], self.points[i + 1])

            if abs(angle_rad) > 0:
                self.gen_in_place_turn_command(angle_rad, steps_per_second)

            # Forward phase
            self.gen_straight_command(
                dist(self.points[i], self.points[i + 1]), steps_per_second)

    def gen_smooth_turn_command(self, steps_per_second, current_robot_angle_rad):
        """
        Generates a smooth turn command set. Each corner in the control points is cut following a circular arc

        Parameters
        ---
        steps_per_second:
            The speed of the wheels
        current_robot_angle_rad:
            The current angle of the robot in radians
        """

        self.commands.clear()

        self.remove_adjacent_duplicate_points()

        if len(self.points) < 2:
            return

        # First align with the direction of the first segment
        angle_rad = angle_points((0, 0), (np.cos(current_robot_angle_rad), np.sin(
            current_robot_angle_rad)), self.points[0], self.points[1])
        self.gen_in_place_turn_command(angle_rad, steps_per_second)

        max_tangent_length_mm = 0

        for i in range(1, len(self.points) - 1):  # For each corner
            vec_segment_1 = np.subtract(self.points[i], self.points[i - 1])
            vec_segment_2 = np.subtract(self.points[i + 1], self.points[i])
            length_segment_1_mm = norm(vec_segment_1)
            length_segment_2_mm = norm(vec_segment_2)
            straight_distance_mm = length_segment_1_mm - max_tangent_length_mm
            max_tangent_length_mm = min(
                length_segment_1_mm / 2, length_segment_2_mm / 2)
            straight_distance_mm -= max_tangent_length_mm
            angle_rad = angle_vecs(vec_segment_1, vec_segment_2)
            self.gen_straight_command(straight_distance_mm, steps_per_second)
            if angle_rad != 0:
                turn_radius_mm = max_tangent_length_mm / np.tan(angle_rad / 2)
                self.gen_turn_command(
                    turn_radius_mm, angle_rad, steps_per_second)

        # For the last segment, drive to the end of it
        vec_last_segment = np.subtract(self.points[-1], self.points[-2])
        length_last_segment_mm = norm(vec_last_segment)
        straight_distance_mm = length_last_segment_mm - max_tangent_length_mm
        self.gen_straight_command(straight_distance_mm, steps_per_second)

    def gen_bezier_path(self, steps):
        """
        Generates a Bezier path

        Parameters
        ---
        steps:
            The number of discretization steps in the Bezier curve generation
        """

        steps *= (len(self.points) - 1)
        self.path.clear()
        if steps == 0:
            return
        for i in range(steps + 1):
            t = i / steps
            self.path.append(recursive_bezier(self.points, t))

    def gen_file(self, filename):
        """
        Generates an instruction file from the command data

        Parameters
        ---
        filename:
            The name of the file to write to
        """

        f = open(filename, 'w')
        original_stdout = sys.stdout
        sys.stdout = f  # Change the standard output to the file we created.
        print('!MOVE')
        nb_commands = len(self.commands)
        print(nb_commands)

        for command in self.commands:
            cmdline = str(command)[1:len(str(command))-1]
            lspd, rspd, stime = cmdline.split(', ')
            print(lspd, rspd, stime)

        print('END %.2f %.2f' % (self.points[-1][0], self.points[-1][1]))
        sys.stdout = original_stdout  # Reset the standard output to its original value

    def gen_bez_command(self, steps_per_second, current_robot_angle_rad):
        """
        Generates a Bezier command set. The control points are used to generate a Bezier curve

        Parameters
        ---
        steps_per_second:
            The speed of the wheels
        current_robot_angle_rad:
            The current angle of the robot in radians
        """

        self.commands.clear()
        self.remove_adjacent_duplicate_points()

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
                self.commands.append(
                    (steps_per_second_left, steps_per_second_right, duration_ms))
