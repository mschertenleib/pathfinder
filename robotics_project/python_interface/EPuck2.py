import matplotlib.pyplot as plt
import matplotlib.image as img
import numpy as np


MM_PER_STEP = 130 / 1000


class Epuck:

    RADIUS_MM = 36.5
    WHEEL_SPACING_MM = 54
    CAM_ANGLE_RAD = np.pi / 4
    TOF_SENSOR_OFFSET_MM = RADIUS_MM
    TOF_MAX_DISTANCE_MM = 2000

    def __init__(self, x_mm=0, y_mm=0, angle_rad=0):
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.angle_rad = angle_rad
        self.trail = []

    def set(self, x_mm, y_mm, angle_rad):
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.angle_rad = angle_rad
        self.trail.clear()

    def check_speed(self, steps_per_second):
        return abs(steps_per_second) <= 1000

    def draw(self, ax, color):
        #print('Drawn at {:.2f} {:.2f}'.format(self.x_mm, self.y_mm))

        # Draw outer circle
        angles_rad = np.linspace(0, 2*np.pi, 100)
        px_mm = self.x_mm + self.RADIUS_MM*np.cos(angles_rad)
        py_mm = self.y_mm + self.RADIUS_MM*np.sin(angles_rad)
        ax.plot(px_mm, py_mm, color=color)

        
        front_x = self.x_mm + self.RADIUS_MM * np.cos(self.angle_rad)
        front_y = self.y_mm + self.RADIUS_MM * np.sin(self.angle_rad)
        
        # Draw direction
        ax.plot([self.x_mm, front_x], [self.y_mm, front_y], color=color)
        
        cone_end_x = front_x + self.RADIUS_MM * np.cos(self.angle_rad)
        cone_end_y = front_y + self.RADIUS_MM * np.sin(self.angle_rad)

        # Draw central line
        ax.plot([front_x, cone_end_x], [front_y, cone_end_y], color=color)

        # Draw left line
        ax.plot([front_x, cone_end_x - (self.RADIUS_MM * np.tan(self.CAM_ANGLE_RAD / 2)) * np.sin(self.angle_rad)],
                [front_y, cone_end_y + (self.RADIUS_MM * np.tan(self.CAM_ANGLE_RAD / 2)) * np.cos(self.angle_rad)],
                color=color)

        # Draw right line
        ax.plot([front_x, cone_end_x + (self.RADIUS_MM * np.tan(self.CAM_ANGLE_RAD / 2)) * np.sin(self.angle_rad)],
                [front_y, cone_end_y - (self.RADIUS_MM * np.tan(self.CAM_ANGLE_RAD / 2)) * np.cos(self.angle_rad)],
                color=color)

    def move(self, steps_per_second_left, steps_per_second_right, ms):
        SCALE_FACTOR = MM_PER_STEP / 1000  # correction from steps/s to mm/ms
        mm_per_ms_left = steps_per_second_left*SCALE_FACTOR
        mm_per_ms_right = steps_per_second_right*SCALE_FACTOR

        if mm_per_ms_right == 0 or mm_per_ms_left == 0:
            ratio = 2
        else:
            ratio = mm_per_ms_left/mm_per_ms_right

        if (ratio == 1):        # goes straight
            for i in range(ms):
                self.trail.append([self.x_mm, self.y_mm, self.angle_rad])
                self.x_mm += (mm_per_ms_left)*np.sin(self.angle_rad)
                self.y_mm += (mm_per_ms_left)*np.cos(self.angle_rad)

        else:                   # goes around
            radi = ((self.WHEEL_SPACING_MM)/(ratio-1))+(5.5/2)
            alpha = (mm_per_ms_left*ms)/(radi + (self.WHEEL_SPACING_MM/2))
            dphi = alpha/ms
            dm = np.sin(dphi)*radi

            for i in range(ms):
                self.trail.append([self.x_mm, self.y_mm, self.angle_rad])
                dx = dm*np.sin(self.angle_rad+(dphi/2))
                dy = dm*np.cos(self.angle_rad+(dphi/2))
                self.angle_rad += dphi
                self.x_mm += dx
                self.y_mm += dy
        # self.draw("#dddddd")

    def show_trail(ax, self, col):
        cx = []
        cy = []
        for elem_x, elem_y in self.trail:
            # self.draw(col)
            cx.append(elem_x)
            cy.append(elem_y)
        ax.plot(cx, cy, color=col)
        self.trail_reset()

    def trail_reset(self):
        self.trail.clear()

    def read_command_file(self, filename):
        f = open(filename, 'r')
        while 1:
            command = f.readline()
            if not command:
                break

            if command[0:4] == "MOVE":
                size = int(int(f.readline())/3)
                if size == 1:
                    size = 0
                for i in range(size):
                    a, b, c = f.readline().split(" ")
                    steps_per_second_left, steps_per_second_right, time_ms = int(
                        a), int(b), int(c)

                    if self.check_speed(steps_per_second_left) and self.check_speed(steps_per_second_right):
                        print("Moving...", steps_per_second_left,
                              steps_per_second_right, time_ms)
                        self.move(steps_per_second_left,
                                  steps_per_second_right, time_ms)
                        self.show_trail("#ff0000")
                    else:
                        print("Invalid speed (>1000):",
                              steps_per_second_left, steps_per_second_right)
                        exit()
            elif command[0:3] == "END":
                str, b, c = command.split(" ")
                supx, supy = float(b), float(c)
                print("err X: %2f Y: %2f " %
                      (self.x_mm - supx, self.y_mm - supy))
                break
            else:
                print("No commands found")
                exit()

        f.close()
        print(filename, "done.")

    def simulate(self, instrfile):
        self.read_command_file(instrfile)
        self.draw("#000000")

        plt.show()
