import numpy as np


class Environment_map:

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = np.zeros((height, width), dtype=np.bool8)

    def get_value_at(self, x, y):
        return self.walls[y][x]

    def set_value_at(self, x, y, value):
        self.walls[y][x] = value

    # coordinates in range [0, 1[
    def generate_horizontal_wall(self, x1, x2, y):
        for x in range(int(x1 * self.width), int(x2 * self.width)):
            self.set_value_at(x, int(y * self.height), True)

    # coordinates in range [0, 1[
    def generate_vertical_wall(self, x, y1, y2):
        for y in range(int(y1 * self.height), int(y2 * self.height)):
            self.set_value_at(int(x * self.width), y, True)
