import numpy as np
import cv2


# Considerations about cell value representation:
# - The disk filtering needs free cells to be represented by a 0
# - The pathfinding library expects a 0 to represent an obstacle and a 1 a free cell
# - When showing the image, it would make sense for a wall to be 0 (black)


class Grid_map:

    FREE = 0
    WALL = 1

    def __init__(self, width_mm, height_mm, cell_size_mm):
        self.cell_size_mm = cell_size_mm
        self.width_mm = width_mm
        self.height_mm = height_mm
        self.width = self.__val_mm_to_index(self.width_mm)  # width in cells
        self.height = self.__val_mm_to_index(self.height_mm)  # height in cells
        self.walls = np.zeros((self.height, self.width))

    def point(self, x_mm, y_mm):
        x_index = self.__val_mm_to_index(x_mm)
        y_index = self.__val_mm_to_index(y_mm)
        self.walls[y_index][x_index] = self.WALL
    
    def point_normalized(self, pt):
        x_index = int(pt[0] * self.width)
        y_index = int(pt[1] * self.height)
        self.walls[y_index][x_index] = self.WALL

    def line(self, pt1_mm, pt2_mm):
        pt1_index = self.__point_mm_to_index(pt1_mm)
        pt2_index = self.__point_mm_to_index(pt2_mm)
        cv2.line(self.walls, pt1_index, pt2_index, self.WALL)
    
    def line_normalized(self, pt1, pt2):
        pt1_index = (int(pt1[0] * self.width_mm), int(pt1[1] * self.height_mm))
        pt2_index = (int(pt2[0] * self.width_mm), int(pt2[1] * self.height_mm))
        cv2.line(self.walls, pt1_index, pt2_index, self.WALL)

    def disk(self, center_mm, radius_mm):
        center = self.__point_mm_to_index(center_mm)
        radius = self.__val_mm_to_index(radius_mm)
        cv2.circle(img=self.walls, center=center,
                   radius=radius, color=self.WALL_VALUE, thickness=-1)

    def to_image(self, height, width):
        return cv2.resize(src=self.walls, dsize=(width, height), interpolation=cv2.INTER_NEAREST)

    def walkable(self, radius_mm):
        kernel = disk_kernel(self.__val_mm_to_index(radius_mm))
        filtered = cv2.filter2D(src=self.walls, ddepth=-1, kernel=kernel)
        return filtered

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
            if self.walls[y_index][x_index]:
                return np.sqrt((x_mm - start_x_mm)**2 + (y_mm - start_y_mm)**2)
            x_mm += delta_mm * np.cos(angle_rad)
            y_mm += delta_mm * np.sin(angle_rad)

        return default_distance_mm

    def __x_mm_is_in_grid(self, x_mm):
        return x_mm > 0 and x_mm < self.width_mm

    def __y_mm_is_in_grid(self, y_mm):
        return y_mm > 0 and y_mm < self.height_mm

    def __point_mm_is_in_grid(self, point_mm):
        x_mm, y_mm = point_mm
        return self.__x_mm_is_in_grid(x_mm) and self.__y_mm_is_in_grid(y_mm)

    def __val_mm_to_index(self, val):
        return int(val / self.cell_size_mm)

    def __point_mm_to_index(self, point):
        return self.__val_mm_to_index(point[0]), self.__val_mm_to_index(point[1])


class Constructed_map:

    FREE_VALUE = 0
    OCCUPIED_VALUE = 1
    FREE_THRESHOLD = 0.3
    OCCUPIED_THRESHOLD = 0.7

    def __init__(self, width_mm, height_mm, cell_size_mm):
        self.cell_size_mm = cell_size_mm
        self.width_mm = width_mm
        self.height_mm = height_mm
        self.width = self.__val_mm_to_index(self.width_mm)  # width in cells
        self.height = self.__val_mm_to_index(self.height_mm)  # height in cells
        self.sums = np.ones((self.height, self.width)) * \
            (self.FREE_VALUE + self.OCCUPIED_VALUE) / 2
        self.samples = np.zeros((self.height, self.width))

    def construct(self, robot_pos_mm, robot_angle_rad, robot_radius_mm, tof_distance_mm, tof_sensor_offset_mm, tof_max_distance_mm):

        # Free disk where the robot is located
        self.__free_disk(center_mm=robot_pos_mm, radius_mm=robot_radius_mm)

        cos_angle = np.cos(robot_angle_rad)
        sin_angle = np.sin(robot_angle_rad)
        tof_x_mm = robot_pos_mm[0] + tof_sensor_offset_mm * cos_angle
        tof_y_mm = robot_pos_mm[1] + tof_sensor_offset_mm * sin_angle

        if tof_distance_mm < tof_max_distance_mm: # We detected an obstacle

            obstacle_x_mm = tof_x_mm + tof_distance_mm * cos_angle
            obstacle_y_mm = tof_y_mm + tof_distance_mm * sin_angle

            # Free line of sight
            self.__free_line((tof_x_mm, tof_y_mm),
                             (obstacle_x_mm, obstacle_y_mm))

            if self.__point_mm_is_in_grid((obstacle_x_mm, obstacle_y_mm)):
                wall_x_index = self.__val_mm_to_index(obstacle_x_mm)
                wall_y_index = self.__val_mm_to_index(obstacle_y_mm)
                # We already counted a sample by setting a free line of sight, and the obstacle cell was wrongly counted as free
                self.sums[wall_y_index][wall_x_index] -= self.FREE_VALUE
                self.sums[wall_y_index][wall_x_index] += self.OCCUPIED_VALUE

        else:
            line_end_x_mm = tof_x_mm + tof_max_distance_mm * cos_angle
            line_end_y_mm = tof_y_mm + tof_max_distance_mm * sin_angle
            # Free line of sight
            self.__free_line((tof_x_mm, tof_y_mm), (line_end_x_mm, line_end_y_mm))

    def to_image(self, height, width):
        samples = self.samples.copy()
        samples[self.samples == 0] = 1
        means = cv2.divide(self.sums, samples)
        return cv2.resize(src=means, dsize=(width, height), interpolation=cv2.INTER_NEAREST)

    def samples_image(self, height, width):
        normalized_samples = self.samples / self.samples.max()
        return cv2.resize(src=normalized_samples, dsize=(width, height), interpolation=cv2.INTER_NEAREST)

    def walkable(self, radius_mm):
        # This only works if free cells are 0
        assert self.FREE_VALUE == 0 and self.FREE_VALUE < self.OCCUPIED_VALUE

        # FIXME
        kernel = disk_kernel(self.__val_mm_to_index(radius_mm))
        filtered = cv2.filter2D(src=self.sums, ddepth=-1, kernel=kernel)
        return filtered
    
    def walkable_resized(self, radius_mm, height, width):
        walkable = self.walkable(radius_mm=radius_mm)
        return cv2.resize(src=walkable, dsize=(width, height), interpolation=cv2.INTER_NEAREST)

    def __x_mm_is_in_grid(self, x_mm):
        return x_mm > 0 and x_mm < self.width_mm

    def __y_mm_is_in_grid(self, y_mm):
        return y_mm > 0 and y_mm < self.height_mm

    def __point_mm_is_in_grid(self, point_mm):
        x_mm, y_mm = point_mm
        return self.__x_mm_is_in_grid(x_mm) and self.__y_mm_is_in_grid(y_mm)

    def __val_mm_to_index(self, val):
        return int(val / self.cell_size_mm)

    def __point_mm_to_index(self, point):
        return self.__val_mm_to_index(point[0]), self.__val_mm_to_index(point[1])

    def __is_free(self, value):
        if self.FREE_THRESHOLD < self.OCCUPIED_THRESHOLD:
            return value <= self.FREE_THRESHOLD
        else:
            return value >= self.FREE_THRESHOLD

    def __is_occupied(self, value):
        if self.FREE_THRESHOLD < self.OCCUPIED_THRESHOLD:
            return value >= self.OCCUPIED_THRESHOLD
        else:
            return value <= self.OCCUPIED_THRESHOLD

    def __is_unknown(self, value):
        return not self.__is_free(value) and not self.__is_occupied(value)

    def __free_line(self, pt1_mm, pt2_mm):
        pt1 = self.__point_mm_to_index(pt1_mm)
        pt2 = self.__point_mm_to_index(pt2_mm)

        # Add samples
        to_add = np.zeros((self.height, self.width))
        cv2.line(img=to_add, pt1=pt1, pt2=pt2, color=1)
        cv2.add(self.samples, to_add, self.samples)

        # Add values
        to_add = np.zeros((self.height, self.width))
        cv2.line(img=to_add, pt1=pt1, pt2=pt2, color=self.FREE_VALUE)
        cv2.add(self.sums, to_add, self.sums)

    def __free_disk(self, center_mm, radius_mm):
        center = self.__point_mm_to_index(center_mm)
        radius = self.__val_mm_to_index(radius_mm)

        # Add samples
        to_add = np.zeros((self.height, self.width))
        cv2.circle(img=to_add, center=center,
                   radius=radius, color=1, thickness=-1)
        cv2.add(self.samples, to_add, self.samples)

        # Add values
        to_add = np.zeros((self.height, self.width))
        cv2.circle(img=to_add, center=center, radius=radius,
                   color=self.FREE_VALUE, thickness=-1)
        cv2.add(self.sums, to_add, self.sums)


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
