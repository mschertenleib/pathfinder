import matplotlib.pyplot as plt
import numpy as np
import cv2 as cv
import environment_map


def create_lidar_data():
    arr = np.zeros((360, 2))
    for i in range(arr.shape[0]):
        arr[i] = (i, 30 + np.random.normal() * 5)
    return arr


def raycast(walls: environment_map.Environment_map, start_x, start_y, angle):
    px, py = start_x, start_y
    delta = 0.1
    while int(py) > 0 and int(py) < walls.height and int(px) > 0 and int(px) < walls.width:
        if walls.get_value_at(x=int(px), y=int(py)):
            return np.sqrt((px - start_x)**2 + (py - start_y)**2)
        px += delta * np.cos(angle)
        py += delta * np.sin(angle)

    return np.Infinity


def get_lidar_data(walls, robot_x, robot_y, tof_sensor_offset):
    num_samples = 100
    arr = np.zeros((num_samples, 2))
    for i in range(num_samples):
        angle_deg = i / num_samples * 360
        angle_rad = np.radians(angle_deg)
        start_x = robot_x + tof_sensor_offset * np.cos(angle_rad)
        start_y = robot_y + tof_sensor_offset * np.sin(angle_rad)
        exact_distance = raycast(walls, start_x, start_y, angle_rad)
        distance = exact_distance * (1 + (np.random.rand() * 2 - 1) * 0.1)
        arr[i] = (angle_deg, distance)
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

    walls = environment_map.Environment_map(width, height)
    walls.generate_horizontal_wall(x1=0.1, x2=0.9, y=0.2)
    walls.generate_horizontal_wall(x1=0.5, x2=0.8, y=0.7)
    walls.generate_vertical_wall(x=0.1, y1=0.1, y2=0.9)
    walls.generate_vertical_wall(x=0.7, y1=0.3, y2=0.7)
    walls.generate_vertical_wall(x=0.4, y1=0.6, y2=0.9)

    constructed_map = environment_map.Environment_map(
        width=width, height=height)

    cv.namedWindow('image', cv.WINDOW_NORMAL)

    while True:

        lidar_data = get_lidar_data(
            walls, robot_x, robot_y, tof_sensor_offset)

        img = np.ones((height, width, 3), np.uint8) * 255

        for angle, distance in lidar_data:
            if distance < np.Infinity:
                wall_x = robot_x + (distance + tof_sensor_offset) * \
                    np.cos(np.radians(angle))
                wall_y = robot_y + (distance + tof_sensor_offset) * \
                    np.sin(np.radians(angle))
                if int(wall_y) < height and int(wall_x) < width:
                    constructed_map.set_value_at(
                        x=int(wall_x), y=int(wall_y), value=True)
                    cv.line(img, (int(robot_x + tof_sensor_offset * np.cos(np.radians(angle))), int(robot_y +
                            tof_sensor_offset * np.sin(np.radians(angle)))), (int(wall_x), int(wall_y)), (200, 200, 200))

        for y in range(height):
            for x in range(width):
                if constructed_map.get_value_at(x, y):
                    cv.circle(img, (x, y), 0, (0, 0, 0))
                elif walls.get_value_at(x, y):
                    cv.circle(img, (x, y), 0, (150, 150, 150))

        draw_robot(img=img, robot_x=robot_x,
                   robot_y=robot_y, robot_angle=30, robot_radius=robot_radius)

        cv.imshow('image', img)
        k = cv.waitKey(0)
        if k == ord('w'):
            robot_y -= robot_delta
        if k == ord('a'):
            robot_x -= robot_delta
        if k == ord('s'):
            robot_y += robot_delta
        if k == ord('d'):
            robot_x += robot_delta
        if k == 27:  # Esc
            break

    cv.destroyAllWindows()
