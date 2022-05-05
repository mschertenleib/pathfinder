import numpy as np
import cv2
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder


class Environment_map:

    OCCUPIED_THRESHOLD = 0.25
    FREE_THRESHOLD = 0.75

    def __init__(self, width_mm, height_mm, cell_size_mm):
        self.cell_size_mm = cell_size_mm
        self.width_mm = width_mm
        self.height_mm = height_mm
        self.width = self.__val_mm_to_index(self.width_mm)  # width in cells
        self.height = self.__val_mm_to_index(self.height_mm)  # height in cells
        self.free_samples = np.zeros((self.height, self.width), dtype=int)
        self.total_samples = np.zeros((self.height, self.width), dtype=int)
        self.cells = np.ones((self.height, self.width)) * 0.5

    def set_all_free(self):
        self.cells = np.ones((self.height, self.width))
    
    def occupied_line(self, pt1_mm, pt2_mm, thickness=1):
        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.line(self.walls, pt1_index, pt2_index, 0, thickness)
    
    def occupied_line_normalized(self, pt1, pt2, thickness=1):
        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.line(self.cells, pt1_index, pt2_index, 0, thickness)
    
    def occupied_rectangle(self, pt1_mm, pt2_mm, thickness=1):
        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.rectangle(self.walls, pt1_index, pt2_index, 0, thickness)
    
    def occupied_rectangle_normalized(self, pt1, pt2, thickness=1):
        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.rectangle(self.cells, pt1_index, pt2_index, 0, thickness)

    def occupied_circle(self, center_mm, radius_mm, thickness=1):
        center = self.__point_mm_to_index(center_mm)
        radius = self.__val_mm_to_index(radius_mm)
        cv2.circle(self.cells, center, radius, 0, thickness)

    def free_line(self, pt1_mm, pt2_mm, thickness=1):
        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.line(self.walls, pt1_index, pt2_index, 1, thickness)
    
    def free_line_normalized(self, pt1, pt2, thickness=1):
        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.line(self.cells, pt1_index, pt2_index, 1, thickness)
    
    def free_rectangle(self, pt1_mm, pt2_mm, thickness=1):
        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.rectangle(self.walls, pt1_index, pt2_index, 1, thickness)
    
    def free_rectangle_normalized(self, pt1, pt2, thickness=1):
        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.rectangle(self.cells, pt1_index, pt2_index, 1, thickness)

    def free_circle(self, center_mm, radius_mm, thickness=1):
        center = self.__point_mm_to_index(center_mm)
        radius = self.__val_mm_to_index(radius_mm)
        cv2.circle(self.cells, center, radius, 1, thickness)
    
    def raycast(self, start_pos_mm, angle_rad, default_distance_mm):
        """
        @brief                      Returns the distance from start_pos_mm to the closest obstacle in the angle_rad direction, or default_distance_mm if no obstacle intersection was found
        @param start_pos_mm         The starting position in millimeters
        @param angle_rad            The angle in radians
        @param default_distance_mm  The default distance to return if no obstacle was hit
        @return                     The computed distance
        """

        start_x_mm, start_y_mm = start_pos_mm
        x_mm, y_mm = start_pos_mm
        delta_mm = self.cell_size_mm / 16
        while self.__point_mm_is_in_grid((x_mm, y_mm)):
            x_index = self.__val_mm_to_index(x_mm)
            y_index = self.__val_mm_to_index(y_mm)
            if self.cells[y_index][x_index] == 0:
                return np.sqrt((x_mm - start_x_mm)**2 + (y_mm - start_y_mm)**2)
            x_mm += delta_mm * np.cos(angle_rad)
            y_mm += delta_mm * np.sin(angle_rad)

        return default_distance_mm


    #FIXME: bug where values go from almost 0 to almost 1 instantly (or the oppposite)
    
    def construct(self, robot_pos_mm, robot_angle_rad, robot_radius_mm, tof_distance_mm, tof_sensor_offset_mm, tof_max_distance_mm):

        # Free disk where the robot is located
        self.free_circle(center_mm=robot_pos_mm, radius_mm=robot_radius_mm, thickness=-1)

        cos_angle = np.cos(robot_angle_rad)
        sin_angle = np.sin(robot_angle_rad)
        tof_x_mm = robot_pos_mm[0] + tof_sensor_offset_mm * cos_angle
        tof_y_mm = robot_pos_mm[1] + tof_sensor_offset_mm * sin_angle

        if tof_distance_mm < tof_max_distance_mm: # We detected an obstacle

            obstacle_x_mm = tof_x_mm + tof_distance_mm * cos_angle
            obstacle_y_mm = tof_y_mm + tof_distance_mm * sin_angle

            # Free line of sight
            self.__sample_free_line((tof_x_mm, tof_y_mm), (obstacle_x_mm, obstacle_y_mm))

            if self.__point_mm_is_in_grid((obstacle_x_mm, obstacle_y_mm)):
                wall_x_index = self.__val_mm_to_index(obstacle_x_mm)
                wall_y_index = self.__val_mm_to_index(obstacle_y_mm)
                # We already counted a free sample in the obstacle cell by calling __sample_free_line
                self.free_samples[wall_y_index][wall_x_index] -= 1

        else:
            line_end_x_mm = tof_x_mm + tof_max_distance_mm * cos_angle
            line_end_y_mm = tof_y_mm + tof_max_distance_mm * sin_angle
            # Free line of sight
            self.__sample_free_line((tof_x_mm, tof_y_mm), (line_end_x_mm, line_end_y_mm))

        self.__update_cells()

    def to_image(self, height, width):
        return cv2.resize(src=self.cells, dsize=(width, height), interpolation=cv2.INTER_NEAREST)

    def walkable(self, radius_mm):
        kernel = disk_kernel(self.__val_mm_to_index(radius_mm))
        # An obstacle is a positive value, a free cell is a 0
        obstacles = np.ones((self.height, self.width))
        obstacles[self.cells > self.FREE_THRESHOLD] = 0
        cv2.rectangle(obstacles, (0, 0), (self.width-1, self.height-1), 1)
        filtered = cv2.filter2D(src=obstacles, ddepth=-1, kernel=kernel)
        walkable_grid = 1 - np.clip(filtered, 0, 1)
        return walkable_grid
    
    def walkable_resized(self, radius_mm, height, width):
        walkable = self.walkable(radius_mm)
        return cv2.resize(src=walkable, dsize=(width, height), interpolation=cv2.INTER_NEAREST)
    
    def find_path(self, start_mm, goal_mm, robot_radius_mm):
        start = self.__point_mm_to_index(start_mm)
        goal = self.__point_mm_to_index(goal_mm)
        path = self.__find_path(start, goal, robot_radius_mm)
        path = [self.__index_to_point_mm(x, y) for x, y in path]
        return path

    def __x_mm_is_in_grid(self, x_mm):
        return x_mm > 0 and x_mm < self.width_mm

    def __y_mm_is_in_grid(self, y_mm):
        return y_mm > 0 and y_mm < self.height_mm

    def __point_mm_is_in_grid(self, point_mm):
        x_mm, y_mm = point_mm
        return self.__x_mm_is_in_grid(x_mm) and self.__y_mm_is_in_grid(y_mm)

    def __val_mm_to_index(self, val_mm):
        return int(val_mm / self.cell_size_mm)

    def __point_mm_to_index(self, point_mm):
        return self.__val_mm_to_index(point_mm[0]), self.__val_mm_to_index(point_mm[1])
    
    def __index_to_val_mm(self, index):
        return (index + 0.5) * self.cell_size_mm

    def __index_to_point_mm(self, index_x, index_y):
        return self.__index_to_val_mm(index_x), self.__index_to_val_mm(index_y)

    def __sample_free_line(self, pt1_mm, pt2_mm):
        pt1 = self.__point_mm_to_index(pt1_mm)
        pt2 = self.__point_mm_to_index(pt2_mm)

        line_to_add = np.zeros((self.height, self.width), dtype=int)
        cv2.line(img=line_to_add, pt1=pt1, pt2=pt2, color=1)

        cv2.add(self.free_samples, line_to_add, self.free_samples)
        cv2.add(self.total_samples, line_to_add, self.total_samples)

    def __update_cells(self):
        # Required for cv2.divide to be defined when dividing by 0
        assert self.total_samples.dtype == int

        means = cv2.divide(self.free_samples, self.total_samples)
        free_probabilities = 0.5 + (means - 0.5) * (1 - np.power(0.5, self.total_samples))
        cells_to_update = np.logical_and(self.cells > 0, self.cells < 1, self.total_samples > 0)
        self.cells = np.where(cells_to_update, free_probabilities, self.cells)
    
    def __find_path(self, start_coords, goal_coords, robot_radius_mm):
        walkable_cells = self.walkable(robot_radius_mm)
        grid = Grid(matrix=walkable_cells)
        start = grid.node(start_coords[0], start_coords[1])
        end = grid.node(goal_coords[0], goal_coords[1])
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)
        return path

def disk_kernel(radius):
    """
    @brief          Returns a kernel (a 2D numpy array) consisting of a disk of 1's
    @param radius   The radius of the disk (the kernel has a size of 2 * radius + 1)
    @return         The constructed kernel
    """

    kernel_size = 2 * radius + 1
    kernel = np.zeros((kernel_size, kernel_size))
    for x in range(0, kernel_size):
        delta_y = int(np.sqrt(radius * radius - (x - radius) * (x - radius)))
        for y in range(radius - delta_y, radius + delta_y + 1):
            kernel[y][x] = 1
    return kernel
