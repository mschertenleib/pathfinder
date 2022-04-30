import numpy as np
import cv2


class Environment_map:

    def __init__(self, width_mm, height_mm, cell_size_mm):
        self.cell_size_mm = cell_size_mm
        self.width_mm = width_mm
        self.height_mm = height_mm
        self.width = self.__mm_to_index(self.width_mm)
        self.height = self.__mm_to_index(self.height_mm)
        self.walls = np.zeros((self.height, self.width), dtype=np.bool8)

    def __mm_to_index(self, mm):
        return int(mm / self.cell_size_mm)

    def get_at_index(self, x_index, y_index):
        return self.walls[y_index][x_index]

    def set_at_index(self, x_index, y_index, value):
        self.walls[y_index][x_index] = value

    def get_at_mm(self, x_mm, y_mm):
        x_index = self.__mm_to_index(x_mm)
        y_index = self.__mm_to_index(y_mm)
        return self.get_at_index(x_index=x_index, y_index=y_index)

    def set_at_mm(self, x_mm, y_mm, value):
        x_index = self.__mm_to_index(x_mm)
        y_index = self.__mm_to_index(y_mm)
        self.set_at_index(x_index=x_index, y_index=y_index, value=value)

    # normalized coordinates in range [0, 1[
    def generate_horizontal_wall(self, x1, x2, y):
        y_index = int(y * self.height)
        for x_index in range(int(x1 * self.width), int(x2 * self.width)):
            self.set_at_index(x_index, y_index, True)

    # normalized coordinates in range [0, 1[
    def generate_vertical_wall(self, x, y1, y2):
        x_index = int(x * self.width)
        for y_index in range(int(y1 * self.height), int(y2 * self.height)):
            self.set_at_index(x_index, y_index, True)

    def raycast(self, start_x_mm, start_y_mm, angle_rad):
        x_mm, y_mm = start_x_mm, start_y_mm
        delta_mm = self.cell_size_mm / 10
        while x_mm > 0 and x_mm < self.width_mm and y_mm > 0 and y_mm < self.height_mm:
            if self.get_at_mm(x_mm, y_mm):
                return np.sqrt((x_mm - start_x_mm)**2 + (y_mm - start_y_mm)**2)
            x_mm += delta_mm * np.cos(angle_rad)
            y_mm += delta_mm * np.sin(angle_rad)

        return np.Infinity

    def to_image(self, img, color):
        img_height_px = img.shape[0]
        #img_width_px = img.shape[1]
        cell_size_px = img_height_px / self.height
        for y_index in range(self.height):
            for x_index in range(self.width):
                if self.get_at_index(x_index, y_index):
                    p1_px = (int(x_index * cell_size_px),
                             int(y_index * cell_size_px))
                    p2_px = (int((x_index + 1) * cell_size_px),
                             int((y_index + 1) * cell_size_px))
                    cv2.rectangle(img, p1_px, p2_px, color, -1)
