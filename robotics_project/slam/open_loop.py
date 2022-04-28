import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv


def generate_horizontal_wall(walls, x1, x2, y):
    height = walls.shape[0]
    width = walls.shape[1]
    for x in range(int(x1 * width), int(x2 * width)):
        walls[int(y * height)][x] = True


def generate_vertical_wall(walls, x, y1, y2):
    height = walls.shape[0]
    width = walls.shape[1]
    for y in range(int(y1 * height), int(y2 * height)):
        walls[y][int(x * width)] = True


def generate_walls(height, width):
    walls = np.zeros((height, width), dtype=np.bool8)
    generate_horizontal_wall(walls, 0.1, 0.9, 0.2)
    generate_horizontal_wall(walls, 0.5, 0.8, 0.7)
    generate_vertical_wall(walls, 0.1, 0.1, 0.9)
    generate_vertical_wall(walls, 0.7, 0.3, 0.7)
    generate_vertical_wall(walls, 0.4, 0.6, 0.9)
    return walls


def create_lidar_data():
    arr = np.zeros((360, 2))
    for i in range(arr.shape[0]):
        arr[i] = (i, 30 + np.random.normal() * 5)
    return arr


def raycast(walls, start_x, start_y, angle):
    px, py = start_x, start_y
    delta = 0.1
    while int(py) > 0 and int(py) < walls.shape[0] and int(px) > 0 and int(px) < walls.shape[1]:
        if walls[int(py)][int(px)]:
            return np.sqrt((px - start_x)**2 + (py - start_y)**2)
        px += delta * np.cos(angle)
        py += delta * np.sin(angle)

    return np.Infinity


def get_lidar_data(walls, robot_x, robot_y, tof_sensor_offset):
    arr = np.zeros((360, 2))
    for i in range(arr.shape[0]):
        angle = np.radians(i)
        start_x = robot_x + tof_sensor_offset * np.cos(angle)
        start_y = robot_y + tof_sensor_offset * np.sin(angle)
        exact_distance = raycast(walls, start_x, start_y, angle)
        distance = exact_distance * (1 + (np.random.rand() * 2 - 1) * 0.1)
        arr[i] = (i, distance)
    return arr


def draw_robot(img, robot_x, robot_y, robot_angle, robot_radius):
    color = (0, 0, 255)

    cv.circle(img, (int(robot_x), int(robot_y)), robot_radius, color)

    cv.line(img, (int(robot_x), int(robot_y)), (int(robot_x + 1.5 * robot_radius * np.cos(np.radians(robot_angle))),
            int(robot_y + 1.5 * robot_radius * np.sin(np.radians(robot_angle)))), color)


if __name__ == '__main__':

    width = 2*160
    height = 2*120
    robot_x = width / 2
    robot_y = height / 2
    robot_radius = 2*5
    robot_delta = 2*robot_radius
    tof_sensor_offset = robot_radius

    walls = generate_walls(height, width)
    constructed_map = np.zeros((height, width), dtype=np.bool8)

    cv.namedWindow('image', cv.WINDOW_NORMAL)

    while True:

        lidar_data = get_lidar_data(
            walls, robot_x, robot_y, tof_sensor_offset)

        for angle, distance in lidar_data:
            if distance < np.Infinity:
                wall_x = robot_x + (distance + tof_sensor_offset) * \
                    np.cos(np.radians(angle))
                wall_y = robot_y + (distance + tof_sensor_offset) * \
                    np.sin(np.radians(angle))
                if int(wall_y) < height and int(wall_x) < width:
                    constructed_map[int(wall_y)][int(wall_x)] = True

        img = np.ones((height, width, 3), np.uint8) * 255

        for i in range(height):
            for j in range(width):
                x = j
                y = height - i
                if constructed_map[i][j]:
                    cv.circle(img, (x, y), 0, (0, 0, 0))
                elif walls[i][j]:
                    cv.circle(img, (x, y), 0, (150, 150, 150))

        draw_robot(img=img, robot_x=robot_x,
                   robot_y=robot_y, robot_angle=30, robot_radius=robot_radius)

        cv.imshow('image', img)
        k = cv.waitKey(0)
        if k == ord('w'):
            robot_y += robot_delta
        if k == ord('a'):
            robot_x -= robot_delta
        if k == ord('s'):
            robot_y -= robot_delta
        if k == ord('d'):
            robot_x += robot_delta
        if k == 27:  # Esc
            break

    cv.destroyAllWindows()
