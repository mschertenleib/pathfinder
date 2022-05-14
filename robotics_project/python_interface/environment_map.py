import numpy as np
import cv2
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder


class Environment_map:
    """
    Represents the 2D map of an environment with obstacles. Cell states can be constructed from samples, or set to be free or occupied directly.

    Attributes
    ---
    cell_size_mm:
        The size of a single map cell in millimeters
    width_mm:
        The width of the map in millimeters
    height_mm:
        The height of the map in millimeters
    width:
        The width of the map in cells
    height:
        The height of the map in cells
    free_samples:
        The number of free samples per cell
    total_samples:
        The number of total samples per cell
    cells:
        The cells of the map
    """

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
        Set all cells on the map to be free
        """

        self.cells = np.ones((self.height, self.width))

    def set_free(self, x_mm, y_mm):
        """
        Sets a free cell on the map

        Parameters
        ---
        x_mm:
            The x coordinate in millimeters
        y_mm:
            The y coordinate in millimeters
        """

        self.__set_cell(x_mm, y_mm, 1)

    def set_occupied(self, x_mm, y_mm):
        """
        Sets an occupied cell on the map

        Parameters
        ---
        x_mm:
            The x coordinate in millimeters
        y_mm:
            The y coordinate in millimeters
        """

        self.__set_cell(x_mm, y_mm, 0)

    def set_occupied_line(self, pt1_mm, pt2_mm):
        """
        Generate an occupied line on the map

        Parameters
        ---
        pt1_mm:
            The first position in mm
        pt2_mm:
            The second position in mm
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.line(self.cells, pt1_index, pt2_index, 0)

    def set_occupied_line_normalized(self, pt1, pt2):
        """
        Generate an occupied line on the map

        Parameters
        ---
        pt1:
            The first position, fraction [0, 1[ of the width of the map
        pt2:
            The second position, fraction [0, 1[ of the height of the map
        """

        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.line(self.cells, pt1_index, pt2_index, 0)

    def set_occupied_rectangle(self, pt1_mm, pt2_mm):
        """
        Generate an occupied rectangle on the map

        Parameters:
        ---
        pt1_mm:
            The first corner position in mm
        pt2_mm:
            The second corner position in mm
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.rectangle(self.cells, pt1_index, pt2_index, 0)

    def set_occupied_filled_rectangle(self, pt1_mm, pt2_mm):
        """
        Generate an occupied filled rectangle on the map

        Parameters:
        ---
        pt1_mm:
            The first corner position in mm
        pt2_mm:
            The second corner position in mm
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.rectangle(self.cells, pt1_index, pt2_index, 0, -1)

    def set_occupied_rectangle_normalized(self, pt1, pt2):
        """
        Generate an occupied rectangle on the map

        Parameters:
        ---
        pt1:
            The first corner position, fraction [0, 1[ of the width of the map
        pt2:
            The second corner position, fraction [0, 1[ of the height of the map
        """

        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.rectangle(self.cells, pt1_index, pt2_index, 0)

    def set_occupied_filled_rectangle_normalized(self, pt1, pt2):
        """
        Generate an occupied filled rectangle on the map

        Parameters:
        ---
        pt1:
            The first corner position, fraction [0, 1[ of the width of the map
        pt2:
            The second corner position, fraction [0, 1[ of the height of the map
        """

        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.rectangle(self.cells, pt1_index, pt2_index, 0, -1)

    def set_occupied_circle(self, center_mm, radius_mm):
        """
        Generate an occupied circle on the map

        Parameters:
        ---
        center_mm:
            The center position in millimeters
        radius_mm:
            The radius in millimeters
        """

        center = self.__point_mm_to_index(center_mm)
        radius = self.__val_mm_to_index(radius_mm)
        cv2.circle(self.cells, center, radius, 0)

    def set_occupied_filled_circle(self, center_mm, radius_mm):
        """
        Generate an occupied circle on the map

        Parameters:
        ---
        center_mm:
            The center position in millimeters
        radius_mm:
            The radius in millimeters
        """

        center = self.__point_mm_to_index(center_mm)
        radius = self.__val_mm_to_index(radius_mm)
        cv2.circle(self.cells, center, radius, 0, -1)

    def set_free_line(self, pt1_mm, pt2_mm):
        """
        Generate a free line on the map

        Parameters:
        ---
        pt1_mm:
            The first position in mm
        pt2_mm:
            The second position in mm
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.line(self.cells, pt1_index, pt2_index, 1)

    def set_free_line_normalized(self, pt1, pt2):
        """
        Generate a free line on the map

        Parameters:
        ---
        pt1:
            The first position, fraction [0, 1[ of the width of the map
        pt2:
            The second position, fraction [0, 1[ of the height of the map
        """

        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.line(self.cells, pt1_index, pt2_index, 1)

    def set_free_rectangle(self, pt1_mm, pt2_mm):
        """
        Generate a free rectangle on the map

        Parameters:
        ---
        pt1_mm:
            The first corner position in mm
        pt2_mm:
            The second corner position in mm
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.rectangle(self.cells, pt1_index, pt2_index, 1)

    def set_free_filled_rectangle(self, pt1_mm, pt2_mm):
        """
        Generate a free rectangle on the map

        Parameters:
        ---
        pt1_mm:
            The first corner position in mm
        pt2_mm:
            The second corner position in mm
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.rectangle(self.cells, pt1_index, pt2_index, 1, -1)

    def set_free_rectangle_normalized(self, pt1, pt2):
        """
        Generate a free rectangle on the map

        Parameters:
        ---
        pt1:
            The first corner position, fraction [0, 1[ of the width of the map
        pt2:
            The second corner position, fraction [0, 1[ of the height of the map
        """

        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.rectangle(self.cells, pt1_index, pt2_index, 1)

    def set_free_filled_rectangle_normalized(self, pt1, pt2):
        """
        Generate a free rectangle on the map

        Parameters:
        ---
        pt1:
            The first corner position, fraction [0, 1[ of the width of the map
        pt2:
            The second corner position, fraction [0, 1[ of the height of the map
        """

        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.rectangle(self.cells, pt1_index, pt2_index, 1, -1)

    def set_free_circle(self, center_mm, radius_mm):
        """
        Generate a free circle on the map

        Parameters:
        ---
        center_mm:
            The center position in millimeters
        radius_mm:
            The radius in millimeters
        """

        center = self.__point_mm_to_index(center_mm)
        radius = self.__val_mm_to_index(radius_mm)
        cv2.circle(self.cells, center, radius, 1)

    def set_free_filled_circle(self, center_mm, radius_mm):
        """
        Generate a free circle on the map

        Parameters:
        ---
        center_mm:
            The center position in millimeters
        radius_mm:
            The radius in millimeters
        """

        center = self.__point_mm_to_index(center_mm)
        radius = self.__val_mm_to_index(radius_mm)
        cv2.circle(self.cells, center, radius, 1, -1)

    def raycast(self, start_pos_mm, angle_rad, max_distance_mm):
        """
        Returns the distance from start_pos_mm to the closest obstacle in the angle_rad direction, or max_distance_mm if no obstacle intersection was found

        Parameters:
        ---
        start_pos_mm:
            The starting position in millimeters
        angle_rad:
            The angle in radians
        max_distance_mm:
            The maximum valid distance in millimeters measured by the TOF

        Returns
        ---
        The computed distance
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

        return max_distance_mm

    def construct(self, robot_pos_mm, robot_angle_rad, robot_radius_mm, tof_distance_mm, tof_sensor_offset_mm, tof_max_distance_mm, line_thickness=1):
        """
        Construct a part of the map

        Parameters:
        ---
        robot_pos_mm:
            The robot position in millimeters
        robot_angle_rad:
            The robot orientation in millimeters
        robot_radius_mm:
            The robot radius in millimeters
        tof_distance_mm:
            The measured TOF distance in millimeters
        tof_sensor_offset_mm:
            The distance from the center of the robot to the TOF in millimeters
        tof_max_distance_mm:
            The maximum distance measurable by the TOF sensor in millimeters
        line_thickness:
            The thickness of the sampling line, in cells
        """

        # Free disk where the robot is located
        self.set_free_filled_circle(
            center_mm=robot_pos_mm, radius_mm=robot_radius_mm*5/4)

        cos_angle = np.cos(robot_angle_rad)
        sin_angle = np.sin(robot_angle_rad)
        tof_x_mm = robot_pos_mm[0] + tof_sensor_offset_mm * cos_angle
        tof_y_mm = robot_pos_mm[1] + tof_sensor_offset_mm * sin_angle

        if tof_distance_mm < tof_max_distance_mm:  # We detected an obstacle
            obstacle_x_mm = tof_x_mm + tof_distance_mm * cos_angle
            obstacle_y_mm = tof_y_mm + tof_distance_mm * sin_angle
            self.__sample_free_line_and_obstacle(
                (tof_x_mm, tof_y_mm), (obstacle_x_mm, obstacle_y_mm), line_thickness)

        else:
            line_end_x_mm = tof_x_mm + tof_max_distance_mm * cos_angle
            line_end_y_mm = tof_y_mm + tof_max_distance_mm * sin_angle
            self.__sample_free_line(
                (tof_x_mm, tof_y_mm), (line_end_x_mm, line_end_y_mm), line_thickness)

        self.__update_cells()

    def as_image(self):
        """
        Get a grayscale image of the map, as a numpy uint8 array

        Returns
        ---
        The image
        """

        return (self.cells * 255).astype(np.uint8)

    def as_image_with_walkable(self, walkable_mm, walkable_color):
        """
        Get a color image of the map with the walkable cells, as a numpy uint8 array

        Parameters
        ---
        walkable_mm:
            The minimum distance in millimeters to keep to any obstacle when computing the walkable cells
        walkable_color:
            A uint8 3-tuple representing the color of the walkable cells

        Returns
        ---
        The image
        """

        map_bgr = cv2.merge([self.as_image()] * 3)
        walkable_mask = cv2.merge([self.__walkable(walkable_mm)] * 3)
        return np.where(walkable_mask > 0, np.array(walkable_color).astype(np.uint8), map_bgr)

    def find_path(self, start_mm, goal_mm, radius_mm):
        """
        Finds the shortest path between start_mm and goal_mm, always keeping at least a radius_mm distance to the nearest obstacle

        Parameters:
        ---
        start_mm:
            The starting (x, y) point in millimeters
        goal_mm:
            The goal (x, y) point in millimeters
        radius_mm:
            The minimum distance in millimeters to always keep to any obstacle

        Returns
        ---
        The list of points (in millimeter coordinates) forming the path, including start and goal, or an empty list if no path was found
        """

        start = self.__point_mm_to_index(start_mm)
        goal = self.__point_mm_to_index(goal_mm)
        path = self.__find_path(start, goal, radius_mm)
        return [self.__index_to_point_mm(x, y) for x, y in path]

    def robot_data_generator(self, tof_sensor_offset_mm, tof_max_distance_mm, add_uncertainty=False):
        """
        Generates simulated testing data for the robot position, orientation and measured distance

        Parameters:
        ---
        tof_sensor_offset_mm:
            The distance from the center of the robot to the TOF sensor in millimeters
        tof_max_distance_mm:
            The maximum valid distance in millimeters measured by the TOF
        add_uncertainty:
            Whether to add a simulated gaussian error to each of the returned values

        Returns
        ---
        A 4-tuple of robot x position in millimeters, robot y position in millimeters, robot angle in radians, and measured distance in millimeters
        """

        i = 0
        while(True):

            robot_angle_rad = i * 1 / 360 * 2 * np.pi
            robot_x_mm = self.width_mm * \
                (0.4 + 0.2 * np.sin(robot_angle_rad*0.7))
            robot_y_mm = self.height_mm / 2

            start_x_mm = robot_x_mm + \
                tof_sensor_offset_mm * np.cos(robot_angle_rad)
            start_y_mm = robot_y_mm + \
                tof_sensor_offset_mm * np.sin(robot_angle_rad)
            distance_mm = self.raycast(
                (start_x_mm, start_y_mm), robot_angle_rad, tof_max_distance_mm)

            if add_uncertainty:
                robot_angle_rad = np.random.normal(
                    robot_angle_rad, np.radians(4))
                robot_x_mm = np.random.normal(robot_x_mm, 5)
                robot_y_mm = np.random.normal(robot_y_mm, 5)
                distance_mm = np.random.normal(distance_mm, distance_mm / 20)

            yield (robot_x_mm, robot_y_mm, robot_angle_rad, distance_mm)

            i += 1

    def __set_cell(self, x_mm, y_mm, value):
        """
        Sets the value of a cell on the map

        Parameters
        ---
        x_mm:
            The x coordinate in millimeters
        y_mm:
            The y coordinate in millimeters
        value:
            The value to set
        """

        x_index = self.__val_mm_to_index(x_mm)
        y_index = self.__val_mm_to_index(y_mm)
        self.cells[y_index][x_index] = value

    def __x_mm_is_in_grid(self, x_mm):
        """
        Checks whether the given x coordinate in millimeters lies in the grid

        Parameters:
        ---
        x_mm:
            The x coordinate in millimeters to check

        Returns
        ---
        True if the coordinate is in the grid, else False
        """

        return x_mm >= 0 and x_mm < self.width_mm

    def __y_mm_is_in_grid(self, y_mm):
        """
        Checks whether the given y coordinate in millimeters lies in the grid

        Parameters:
        ---
        y_mm:
            The y coordinate in millimeters to check

        Returns
        ---
        True if the coordinate is in the grid, else False
        """

        return y_mm >= 0 and y_mm < self.height_mm

    def __point_mm_is_in_grid(self, point_mm):
        """
        Checks whether the given (x, y) point in millimeters lies in the grid

        Parameters:
        ---
        point_mm:
            The (x, y) point in millimeters to check

        Returns
        ---
        True if the point is in the grid, else False
        """

        x_mm, y_mm = point_mm
        return self.__x_mm_is_in_grid(x_mm) and self.__y_mm_is_in_grid(y_mm)

    def __val_mm_to_index(self, val_mm):
        """
        Converts a coordinate in millimeters to an index

        Parameters:
        ---
        index:
            The coordinate in millimeters to convert

        Returns
        ---
        The grid index
        """

        return int(val_mm / self.cell_size_mm)

    def __point_mm_to_index(self, point_mm):
        """
        Converts an (x, y) point in millimeters to an (x, y) index pair

        Parameters:
        ---
        point_mm:
            The (x, y) coordinates in millimeters

        Returns
        ---
        The (x, y) index of the point in the grid
        """

        return self.__val_mm_to_index(point_mm[0]), self.__val_mm_to_index(point_mm[1])

    def __index_to_val_mm(self, index):
        """
        Converts an index to a coordinate in millimeters

        Parameters:
        ---
        index    The grid index to convert

        Returns
        ---
        The coordinate in millimeters
        """

        return (index + 0.5) * self.cell_size_mm

    def __index_to_point_mm(self, index_x, index_y):
        """
        Converts an (x, y) index pair to an (x, y) point in millimeters

        Parameters:
        ---
        index_x:
            The horizontal coordinate of the point in the grid
        index_y:
            The vertical coordinate of the point in the grid

        Returns
        ---
        The (x, y) point in millimeters
        """

        return self.__index_to_val_mm(index_x), self.__index_to_val_mm(index_y)

    def __sample_free_line(self, pt1_mm, pt2_mm, thickness=1):
        """
        Updates the samples with a free line from pt1_mm to pt2_mm

        Parameters:
        ---
        pt1_mm:
            The starting (x, y) point in millimeters
        pt2_mm:
            The ending (x, y) point in millimeters
        thickness:
            The thickness of the line in cells
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)

        # Line image as np.uint8 for anti-aliasing to work
        line_to_add = np.zeros((self.height, self.width), dtype=np.uint8)
        cv2.line(img=line_to_add, pt1=pt1_index, pt2=pt2_index,
                 color=255, thickness=thickness, lineType=cv2.LINE_AA)
        line_to_add_float = line_to_add.astype(float) / 255

        self.free_samples = cv2.add(self.free_samples, line_to_add_float)
        self.total_samples = cv2.add(self.total_samples, line_to_add_float)

    def __sample_free_line_and_obstacle(self, pt1_mm, pt2_mm, thickness=1):
        """
        Updates the samples with an obstacle on pt2_mm and a free line from pt1_mm

        Parameters:
        ---
        pt1_mm:
            The starting (x, y) point in millimeters
        pt2_mm:
            The ending (x, y) point in millimeters
        thickness:
            The thickness of the line in cells
        """

        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)

        # Add a full line to total_samples
        line_to_add = np.zeros((self.height, self.width), dtype=np.uint8)
        cv2.line(img=line_to_add, pt1=pt1_index, pt2=pt2_index,
                 color=255, thickness=thickness, lineType=cv2.LINE_AA)
        self.total_samples = cv2.add(
            self.total_samples, line_to_add.astype(float) / 255)

        # Remove the tip of the line before adding to free_samples
        #line_vector = np.subtract(pt2_index, pt1_index)
        #delta = line_vector / np.linalg.norm(line_vector) * thickness
        #line_mask_center = np.add(pt2_index, (int(delta[0]), int(delta[1])))
        #line_mask_pt1 = np.add(line_mask_center, (line_vector[1], -line_vector[0]))
        #line_mask_pt2 = np.subtract(line_mask_center, (line_vector[1], -line_vector[0]))
        #cv2.line(img=line_to_add, pt1=line_mask_pt1, pt2=line_mask_pt2, color=0, thickness=thickness)
        cv2.circle(img=line_to_add, center=pt2_index,
                   radius=thickness, color=0, thickness=-1)
        self.free_samples = cv2.add(
            self.free_samples, line_to_add.astype(float) / 255)

    def __update_cells(self):
        """
        Updates all the cells of the map depending on their previous value and the samples
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
        Returns an integer numpy grid of the cells of the map that are at least radius_mm millimeters from any obstacle. A 1 represents a valid cell, a 0 represents an invalid cell

        Parameters:
        ---
        radius_mm:
            The required minimum distance from any obstacle in millimeters

        Returns
        ---
            The grid
        """

        # An obstacle is a positive value, a free cell is a 0
        obstacles = np.ones((self.height, self.width), np.uint8)
        obstacles[self.cells > self.__FREE_THRESHOLD] = 0
        cv2.rectangle(obstacles, (0, 0), (self.width - 1, self.height - 1), 1)

        kernel = disk_kernel(self.__val_mm_to_index(radius_mm))
        filtered = cv2.filter2D(src=obstacles, ddepth=-1, kernel=kernel)

        walkable_grid = 1 - np.clip(filtered, 0, 1)
        return walkable_grid.astype(np.uint8)

    def __find_path(self, start_coords, goal_coords, radius_mm):
        """
        Finds the shortest path between start_coords and goal_coords, always keeping at least a radius_mm distance to the nearest obstacle

        Parameters:
        ---
        start_coords:
            The starting (x, y) coordinates
        goal_coords:
            The goal (x, y) coordinates
        radius_mm:
            The minimum distance in millimeters to always keep to any obstacle

        Returns
        ---
        The list of points forming the path, including start and goal, or an empty list if no path was found
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
    Returns a kernel (a 2D numpy array) consisting of a disk of 1's

    Parameters:
    ---
    radius:
        The radius of the disk (the kernel has a size of 2 * radius + 1)

    Returns
    ---
    The constructed kernel
    """

    kernel_size = 2 * radius + 1
    kernel = np.zeros((kernel_size, kernel_size), np.uint8)
    for x in range(0, kernel_size):
        delta_y = int(np.sqrt(radius * radius - (x - radius) * (x - radius)))
        for y in range(radius - delta_y, radius + delta_y + 1):
            kernel[y][x] = 1
    return kernel
