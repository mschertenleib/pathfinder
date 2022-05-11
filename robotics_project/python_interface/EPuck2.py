import matplotlib.pyplot as plt
import numpy as np


class EPuck2:
    """
    Class representing the e-puck2 robot, its physical dimensions, and offering a set of methods used to simulate the robot
    """

    RADIUS_MM = 36.5
    WHEEL_SPACING_MM = 54
    WHEEL_DIAMETER_MM = 41
    WHEEL_CIRCUMFERENCE_MM = np.pi * WHEEL_DIAMETER_MM
    STEPS_PER_TURN = 1000
    MM_PER_STEP = WHEEL_CIRCUMFERENCE_MM / STEPS_PER_TURN
    MAX_STEPS_PER_SECOND = 1000
    CAM_FOV_RAD = np.pi / 4
    TOF_SENSOR_OFFSET_MM = RADIUS_MM  # Distance from robot center to TOF sensor
    TOF_MAX_DISTANCE_MM = 2000

    def __init__(self, x_mm=0, y_mm=0, angle_rad=0):
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.angle_rad = angle_rad
        self.trail = [(self.x_mm, self.y_mm, self.angle_rad)]

    def is_speed_valid(self, steps_per_second):
        return abs(steps_per_second) <= self.MAX_STEPS_PER_SECOND

    def draw(self, ax, color):
        """
        Draws the robot
        
        Parameter
        ---
        ax:
            The matplotlib Axes instance on which to draw
        color:
            The color to use when drawing
        """

        # Draw outer circle
        points_per_circle = 100
        angles_rad = np.linspace(0, 2 * np.pi, points_per_circle)
        px_mm = self.x_mm + self.RADIUS_MM * np.cos(angles_rad)
        py_mm = self.y_mm + self.RADIUS_MM * np.sin(angles_rad)
        ax.plot(px_mm, py_mm, color=color)

        cos_angle = np.cos(self.angle_rad)
        sin_angle = np.sin(self.angle_rad)

        # Draw left wheel
        left_wheel_center_x_mm = self.x_mm - self.WHEEL_SPACING_MM / 2 * sin_angle
        left_wheel_center_y_mm = self.y_mm + self.WHEEL_SPACING_MM / 2 * cos_angle
        left_wheel_start_x_mm = left_wheel_center_x_mm - \
            self.WHEEL_DIAMETER_MM / 2 * cos_angle
        left_wheel_start_y_mm = left_wheel_center_y_mm - \
            self.WHEEL_DIAMETER_MM / 2 * sin_angle
        left_wheel_end_x_mm = left_wheel_center_x_mm + \
            self.WHEEL_DIAMETER_MM / 2 * cos_angle
        left_wheel_end_y_mm = left_wheel_center_y_mm + \
            self.WHEEL_DIAMETER_MM / 2 * sin_angle
        ax.plot([left_wheel_start_x_mm, left_wheel_end_x_mm], [
                left_wheel_start_y_mm, left_wheel_end_y_mm], color=color)

        # Draw right wheel
        right_wheel_center_x_mm = self.x_mm + self.WHEEL_SPACING_MM / 2 * sin_angle
        right_wheel_center_y_mm = self.y_mm - self.WHEEL_SPACING_MM / 2 * cos_angle
        right_wheel_start_x_mm = right_wheel_center_x_mm - \
            self.WHEEL_DIAMETER_MM / 2 * cos_angle
        right_wheel_start_y_mm = right_wheel_center_y_mm - \
            self.WHEEL_DIAMETER_MM / 2 * sin_angle
        right_wheel_end_x_mm = right_wheel_center_x_mm + \
            self.WHEEL_DIAMETER_MM / 2 * cos_angle
        right_wheel_end_y_mm = right_wheel_center_y_mm + \
            self.WHEEL_DIAMETER_MM / 2 * sin_angle
        ax.plot([right_wheel_start_x_mm, right_wheel_end_x_mm], [
                right_wheel_start_y_mm, right_wheel_end_y_mm], color=color)

        front_x_mm = self.x_mm + self.RADIUS_MM * cos_angle
        front_y_mm = self.y_mm + self.RADIUS_MM * sin_angle

        # Draw direction
        #ax.plot([self.x_mm, front_x_mm], [self.y_mm, front_y_mm], color=color)

        cone_end_x = front_x_mm + self.RADIUS_MM * cos_angle
        cone_end_y = front_y_mm + self.RADIUS_MM * sin_angle

        # Draw central line
        #ax.plot([front_x_mm, cone_end_x], [front_y_mm, cone_end_y], color=color)

        # Draw left camera line
        ax.plot([front_x_mm, cone_end_x - (self.RADIUS_MM * np.tan(self.CAM_FOV_RAD / 2)) * sin_angle],
                [front_y_mm, cone_end_y +
                    (self.RADIUS_MM * np.tan(self.CAM_FOV_RAD / 2)) * cos_angle],
                color=color)

        # Draw right camera line
        ax.plot([front_x_mm, cone_end_x + (self.RADIUS_MM * np.tan(self.CAM_FOV_RAD / 2)) * sin_angle],
                [front_y_mm, cone_end_y -
                    (self.RADIUS_MM * np.tan(self.CAM_FOV_RAD / 2)) * cos_angle],
                color=color)

    def draw_trail(self, ax, color):
        """
        Draws the trail of the robot, i.e. the path it followed
        
        Parameter
        ---
        ax:
            The matplotlib Axes instance on which to draw
        color:
            The color to use when drawing
        """
        
        xs_mm = []
        ys_mm = []
        for x_mm, y_mm, angle_rad in self.trail:
            xs_mm.append(x_mm)
            ys_mm.append(y_mm)
        ax.plot(xs_mm, ys_mm, color=color)

    def move_steps(self, steps_left, steps_right):
        """
        Moves the simulated robot by the given number of steps
        
        Parameters
        ---
        steps_left:
            Number of steps to turn the left motor
        steps_right:
            Number of steps to turn the right motor
        """

        distance_left_mm = steps_left * self.MM_PER_STEP
        distance_right_mm = steps_right * self.MM_PER_STEP

        if distance_left_mm == distance_right_mm:
            self.x_mm += distance_left_mm * np.cos(self.angle_rad)
            self.y_mm += distance_left_mm * np.sin(self.angle_rad)
            self.trail.append((self.x_mm, self.y_mm, self.angle_rad))

        else:
            turn_radius_mm = 0.5 * (distance_left_mm + distance_right_mm) / (
                distance_right_mm - distance_left_mm) * self.WHEEL_SPACING_MM
            turn_angle_rad = (distance_right_mm -
                              distance_left_mm) / self.WHEEL_SPACING_MM
            start_angle = self.angle_rad
            end_angle = start_angle + turn_angle_rad
            turn_center_x_mm = self.x_mm - \
                turn_radius_mm * np.sin(self.angle_rad)
            turn_center_y_mm = self.y_mm + \
                turn_radius_mm * np.cos(self.angle_rad)
            points_per_circle = 100
            num_points = int(
                abs(turn_angle_rad / (2 * np.pi) * points_per_circle))
            if num_points == 0:
                num_points = 1
            angles_rad = np.linspace(start_angle, end_angle, num_points)
            pxs_mm = turn_center_x_mm + turn_radius_mm * np.sin(angles_rad)
            pys_mm = turn_center_y_mm - turn_radius_mm * np.cos(angles_rad)
            for i in range(num_points):
                self.trail.append((pxs_mm[i], pys_mm[i], angles_rad[i]))
            self.x_mm = pxs_mm[num_points - 1]
            self.y_mm = pys_mm[num_points - 1]
            self.angle_rad = end_angle

    def move_speed(self, steps_per_second_left, steps_per_second_right, duration_ms):
        """
        Moves the simulated robot with the given speed during the specified time
        
        Parameters
        ---
        steps_per_second_left:
            Speed of the left motor
        steps_per_second_right:
            Speed of the right motor
        duration_ms:
            Duration of the move in milliseconds
        """
        
        steps_left = steps_per_second_left * duration_ms / 1000
        steps_right = steps_per_second_right * duration_ms / 1000
        self.move_steps(steps_left, steps_right)

    def read_command_file(self, filename):
        """
        Read the specified file and executes the moves
        
        Parameters
        ---
        filename:
            The name of the instruction file to read
        """
        
        f = open(filename, 'r')
        while True:
            command = f.readline()
            if not command:
                break

            if command[0:5] == '!MOVE':
                size = int(int(f.readline()) / 3)
                if size == 1:
                    size = 0
                for i in range(size):
                    steps_per_second_left_str, steps_per_second_right_str, time_ms_str = f.readline().split(' ')
                    steps_per_second_left = int(steps_per_second_left_str)
                    steps_per_second_right = int(steps_per_second_right_str)
                    time_ms = int(time_ms_str)

                    if not self.is_speed_valid(steps_per_second_left) or not self.is_speed_valid(steps_per_second_right):
                        print('Invalid speed (>', self.MAX_STEPS_PER_SECOND, '):',
                              steps_per_second_left, steps_per_second_right)
                        return

                    print('Moving', steps_per_second_left,
                          steps_per_second_right, time_ms)
                    self.move_speed(steps_per_second_left,
                                    steps_per_second_right, time_ms)

            elif command[0:3] == 'END':
                end_str, supposed_x_str, supposed_y_str = command.split(' ')
                supposed_x_mm, supposed_y_mm = float(supposed_x_str), float(supposed_y_str)
                print('Error X:', self.x_mm - supposed_x_mm,
                      'mm   Y:', self.y_mm - supposed_y_mm, 'mm')
                break
            else:
                print('No commands found')
                return

        f.close()
        print(filename, 'Done.')
