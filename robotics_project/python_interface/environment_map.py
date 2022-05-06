import numpy as np
import cv2
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder


class Environment_map:

    __OCCUPIED_THRESHOLD = 0.25
    __FREE_THRESHOLD = 0.75

    def __init__(self, width_mm, height_mm, cell_size_mm):
        self.cell_size_mm = cell_size_mm
        self.width_mm = width_mm
        self.height_mm = height_mm
        self.width = self.__val_mm_to_index(self.width_mm)  # width in cells
        self.height = self.__val_mm_to_index(self.height_mm)  # height in cells
        self.free_samples = np.zeros((self.height, self.width))
        self.total_samples = np.zeros((self.height, self.width))
        self.cells = np.ones((self.height, self.width)) * 0.5

    def set_all_free(self):
        """
        @brief      Set all cells on the map to be free
        """

        self.cells = np.ones((self.height, self.width))

    def occupied_line(self, pt1_mm, pt2_mm):
        """
        @brief              Generate an occupied line on the map
        @param pt1_mm       The first position in mm
        @param pt2_mm       The second position in mm
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.line(self.walls, pt1_index, pt2_index, 0)

    def occupied_line_normalized(self, pt1, pt2):
        """
        @brief              Generate an occupied line on the map
        @param pt1          The first position, fraction [0, 1[ of the width of the map
        @param pt2          The second position, fraction [0, 1[ of the height of the map
        """

        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.line(self.cells, pt1_index, pt2_index, 0)

    def occupied_rectangle(self, pt1_mm, pt2_mm):
        """
        @brief              Generate an occupied rectangle on the map
        @param pt1_mm       The first corner position in mm
        @param pt2_mm       The second corner position in mm
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.rectangle(self.walls, pt1_index, pt2_index, 0)

    def occupied_filled_rectangle(self, pt1_mm, pt2_mm):
        """
        @brief              Generate an occupied filled rectangle on the map
        @param pt1_mm       The first corner position in mm
        @param pt2_mm       The second corner position in mm
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.rectangle(self.walls, pt1_index, pt2_index, 0, -1)

    def occupied_rectangle_normalized(self, pt1, pt2):
        """
        @brief              Generate an occupied rectangle on the map
        @param pt1          The first corner position, fraction [0, 1[ of the width of the map
        @param pt2          The second corner position, fraction [0, 1[ of the height of the map
        """

        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.rectangle(self.cells, pt1_index, pt2_index, 0)

    def occupied_filled_rectangle_normalized(self, pt1, pt2):
        """
        @brief              Generate an occupied filled rectangle on the map
        @param pt1          The first corner position, fraction [0, 1[ of the width of the map
        @param pt2          The second corner position, fraction [0, 1[ of the height of the map
        """

        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.rectangle(self.cells, pt1_index, pt2_index, 0, -1)

    def occupied_circle(self, center_mm, radius_mm):
        """
        @brief              Generate an occupied circle on the map
        @param center_mm    The center position in millimeters
        @param radius_mm    The radius in millimeters
        """

        center = self.__point_mm_to_index(center_mm)
        radius = self.__val_mm_to_index(radius_mm)
        cv2.circle(self.cells, center, radius, 0)

    def occupied_filled_circle(self, center_mm, radius_mm):
        """
        @brief              Generate an occupied circle on the map
        @param center_mm    The center position in millimeters
        @param radius_mm    The radius in millimeters
        """

        center = self.__point_mm_to_index(center_mm)
        radius = self.__val_mm_to_index(radius_mm)
        cv2.circle(self.cells, center, radius, 0, -1)

    def free_line(self, pt1_mm, pt2_mm):
        """
        @brief              Generate a free line on the map
        @param pt1_mm       The first position in mm
        @param pt2_mm       The second position in mm
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.line(self.walls, pt1_index, pt2_index, 1)

    def free_line_normalized(self, pt1, pt2):
        """
        @brief              Generate a free line on the map
        @param pt1          The first position, fraction [0, 1[ of the width of the map
        @param pt2          The second position, fraction [0, 1[ of the height of the map
        """

        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.line(self.cells, pt1_index, pt2_index, 1)

    def free_rectangle(self, pt1_mm, pt2_mm):
        """
        @brief              Generate a free rectangle on the map
        @param pt1_mm       The first corner position in mm
        @param pt2_mm       The second corner position in mm
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.rectangle(self.walls, pt1_index, pt2_index, 1)

    def free_filled_rectangle(self, pt1_mm, pt2_mm):
        """
        @brief              Generate a free rectangle on the map
        @param pt1_mm       The first corner position in mm
        @param pt2_mm       The second corner position in mm
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.rectangle(self.walls, pt1_index, pt2_index, 1, -1)

    def free_rectangle_normalized(self, pt1, pt2):
        """
        @brief              Generate a free rectangle on the map
        @param pt1          The first corner position, fraction [0, 1[ of the width of the map
        @param pt2          The second corner position, fraction [0, 1[ of the height of the map
        """

        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.rectangle(self.cells, pt1_index, pt2_index, 1)

    def free_filled_rectangle_normalized(self, pt1, pt2):
        """
        @brief              Generate a free rectangle on the map
        @param pt1          The first corner position, fraction [0, 1[ of the width of the map
        @param pt2          The second corner position, fraction [0, 1[ of the height of the map
        """

        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.rectangle(self.cells, pt1_index, pt2_index, 1, -1)

    def free_circle(self, center_mm, radius_mm):
        """
        @brief              Generate a free circle on the map
        @param center_mm    The center position in millimeters
        @param radius_mm    The radius in millimeters
        """

        center = self.__point_mm_to_index(center_mm)
        radius = self.__val_mm_to_index(radius_mm)
        cv2.circle(self.cells, center, radius, 1)

    def free_filled_circle(self, center_mm, radius_mm):
        """
        @brief              Generate a free circle on the map
        @param center_mm    The center position in millimeters
        @param radius_mm    The radius in millimeters
        """

        center = self.__point_mm_to_index(center_mm)
        radius = self.__val_mm_to_index(radius_mm)
        cv2.circle(self.cells, center, radius, 1, -1)

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

    def construct(self, robot_pos_mm, robot_angle_rad, robot_radius_mm, tof_distance_mm, tof_sensor_offset_mm, tof_max_distance_mm):
        """
        @brief                          Construct a part of the map
        @param robot_pos_mm             The robot position in millimeters
        @param robot_angle_rad          The robot orientation in millimeters
        @param robot_radius_mm          The robot radius in millimeters
        @param tof_distance_mm          The measured TOF distance in millimeters
        @param tof_sensor_offset_mm     The distance from the center of the robot to the TOF in millimeters
        @param tof_max_distance_mm      The maximum distance measurable by the TOF sensor in millimeters
        """

        # Free disk where the robot is located
        self.free_circle(center_mm=robot_pos_mm,
                         radius_mm=robot_radius_mm, thickness=-1)

        cos_angle = np.cos(robot_angle_rad)
        sin_angle = np.sin(robot_angle_rad)
        tof_x_mm = robot_pos_mm[0] + tof_sensor_offset_mm * cos_angle
        tof_y_mm = robot_pos_mm[1] + tof_sensor_offset_mm * sin_angle

        if tof_distance_mm < tof_max_distance_mm:  # We detected an obstacle

            obstacle_x_mm = tof_x_mm + tof_distance_mm * cos_angle
            obstacle_y_mm = tof_y_mm + tof_distance_mm * sin_angle

            # Free line of sight
            self.__sample_free_line(
                (tof_x_mm, tof_y_mm), (obstacle_x_mm, obstacle_y_mm))

            if self.__point_mm_is_in_grid((obstacle_x_mm, obstacle_y_mm)):
                wall_x_index = self.__val_mm_to_index(obstacle_x_mm)
                wall_y_index = self.__val_mm_to_index(obstacle_y_mm)
                # We already counted a free sample in the obstacle cell by calling __sample_free_line
                self.free_samples[wall_y_index][wall_x_index] -= 1

        else:
            line_end_x_mm = tof_x_mm + tof_max_distance_mm * cos_angle
            line_end_y_mm = tof_y_mm + tof_max_distance_mm * sin_angle
            # Free line of sight
            self.__sample_free_line(
                (tof_x_mm, tof_y_mm), (line_end_x_mm, line_end_y_mm))

        self.__update_cells()

    def to_image(self, height, width):
        """
        @brief              Returns a (height, width) grayscale numpy image of the map
        @param height       The height of the image in pixels
        @param width        The width of the image in pixels
        @return             The image
        """

        return cv2.resize(src=self.cells, dsize=(width, height), interpolation=cv2.INTER_NEAREST)

    def walkable(self, radius_mm, height, width):
        """
        @brief              Returns a (height, width) numpy image of the cells of the map that are at least radius_mm millimeters from any obstacle. A 1 (white) represents a valid cell, a 0 (black) represents an invalid cell
        @param radius_mm    The required minimum distance from any obstacle in millimeters
        @param height       The height of the image in pixels
        @param width        The width of the image in pixels
        @return             The image
        """

        walkable = self.__walkable(radius_mm)
        return cv2.resize(src=walkable, dsize=(width, height), interpolation=cv2.INTER_NEAREST)

    def find_path(self, start_mm, goal_mm, radius_mm):
        """
        @brief              Finds the shortest path between start_mm and goal_mm, always keeping at least a radius_mm distance to the nearest obstacle
        @param start_mm     The starting (x, y) point in millimeters
        @param goal_mm      The goal (x, y) point in millimeters
        @param radius_mm    The minimum distance in millimeters to always keep to any obstacle
        @return             The list of points (in millimeter coordinates) forming the path, including start and goal, or an empty list if no path was found
        """

        start = self.__point_mm_to_index(start_mm)
        goal = self.__point_mm_to_index(goal_mm)
        path = self.__find_path(start, goal, radius_mm)
        path = [self.__index_to_point_mm(x, y) for x, y in path]
        return path

    def __x_mm_is_in_grid(self, x_mm):
        """
        @brief              Checks whether the given x coordinate in millimeters lies in the grid
        @param x_mm         The x coordinate in millimeters to check
        @return             True if the coordinate is in the grid, else False
        """

        return x_mm >= 0 and x_mm < self.width_mm

    def __y_mm_is_in_grid(self, y_mm):
        """
        @brief              Checks whether the given y coordinate in millimeters lies in the grid
        @param y_mm         The y coordinate in millimeters to check
        @return             True if the coordinate is in the grid, else False
        """

        return y_mm >= 0 and y_mm < self.height_mm

    def __point_mm_is_in_grid(self, point_mm):
        """
        @brief              Checks whether the given (x, y) point in millimeters lies in the grid
        @param point_mm     The (x, y) point in millimeters to check
        @return             True if the point is in the grid, else False
        """

        x_mm, y_mm = point_mm
        return self.__x_mm_is_in_grid(x_mm) and self.__y_mm_is_in_grid(y_mm)

    def __val_mm_to_index(self, val_mm):
        """
        @brief          Converts a coordinate in millimeters to an index
        @param index    The coordinate in millimeters to convert
        @return         The grid index
        """

        return int(val_mm / self.cell_size_mm)

    def __point_mm_to_index(self, point_mm):
        """
        @brief              Converts an (x, y) point in millimeters to an (x, y) index pair
        @param point_mm     The (x, y) coordinates in millimeters
        @return             The (x, y) index of the point in the grid
        """

        return self.__val_mm_to_index(point_mm[0]), self.__val_mm_to_index(point_mm[1])

    def __index_to_val_mm(self, index):
        """
        @brief          Converts an index to a coordinate in millimeters
        @param index    The grid index to convert
        @return         The coordinate in millimeters
        """

        return (index + 0.5) * self.cell_size_mm

    def __index_to_point_mm(self, index_x, index_y):
        """
        @brief              Converts an (x, y) index pair to an (x, y) point in millimeters
        @param index_x      The horizontal coordinate of the point in the grid
        @param index_y      The vertical coordinate of the point in the grid
        @return             The (x, y) point in millimeters
        """

        return self.__index_to_val_mm(index_x), self.__index_to_val_mm(index_y)

    def __sample_free_line(self, pt1_mm, pt2_mm):
        """
        @brief          Updates the samples with a free line from pt1_mm to pt2_mm
        @param pt1_mm   The starting (x, y) point in millimeters
        @param pt2_mm   The ending (x, y) point in millimeters
        """

        pt1 = self.__point_mm_to_index(pt1_mm)
        pt2 = self.__point_mm_to_index(pt2_mm)

        line_to_add = np.zeros((self.height, self.width))
        cv2.line(img=line_to_add, pt1=pt1, pt2=pt2, color=1)

        self.free_samples = cv2.add(self.free_samples, line_to_add)
        self.total_samples = cv2.add(self.total_samples, line_to_add)

    def __update_cells(self):
        """
        @brief      Updates all the cells of the map depending on their previous value and the samples
        """

        means = cv2.divide(self.free_samples, self.total_samples)
        means[self.total_samples == 0] = 0.5
        free_probabilities = 0.5 + (means - 0.5) * \
            (1 - np.power(0.5, self.total_samples))
        cells_to_update = np.logical_and(
            self.cells > 0, self.cells < 1, self.total_samples > 0)
        self.cells = np.where(cells_to_update, free_probabilities, self.cells)

    def __walkable(self, radius_mm):
        """
        @brief              Returns a numpy grid of the cells of the map that are at least radius_mm millimeters from any obstacle. A 1 (white) represents a valid cell, a 0 (black) represents an invalid cell
        @param radius_mm    The required minimum distance from any obstacle in millimeters
        @return             The grid
        """

        kernel = disk_kernel(self.__val_mm_to_index(radius_mm))
        # An obstacle is a positive value, a free cell is a 0
        obstacles = np.ones((self.height, self.width))
        obstacles[self.cells > self.__FREE_THRESHOLD] = 0
        cv2.rectangle(obstacles, (0, 0), (self.width - 1, self.height - 1), 1)
        filtered = cv2.filter2D(src=obstacles, ddepth=-1, kernel=kernel)
        walkable_grid = 1 - np.clip(filtered, 0, 1)
        return walkable_grid

    def __find_path(self, start_coords, goal_coords, radius_mm):
        """
        @brief                  Finds the shortest path between start_coords and goal_coords, always keeping at least a radius_mm distance to the nearest obstacle
        @param start_coords     The starting (x, y) coordinates
        @param goal_coords      The goal (x, y) coordinates
        @param radius_mm        The minimum distance in millimeters to always keep to any obstacle
        @return                 The list of points forming the path, including start and goal, or an empty list if no path was found
        """

        walkable_cells = self.__walkable(radius_mm)
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
