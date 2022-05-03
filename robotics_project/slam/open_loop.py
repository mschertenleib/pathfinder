import numpy as np
import cv2
import environment_map as envmap


def robot_data_generator(true_map: envmap.Grid_map, tof_sensor_offset_mm, tof_default_distance_mm, add_uncertainty = False):
    i = 0
    while(True):

        # Robot rotates at the center of the map
        robot_angle_rad = i / 360 * 2 * np.pi
        robot_x_mm = true_map.width_mm * (0.5 + 0.05 * np.sin(robot_angle_rad))
        robot_y_mm = true_map.height_mm * (0.5 - 0.05 * np.cos(robot_angle_rad))

        start_x_mm = robot_x_mm + tof_sensor_offset_mm * np.cos(robot_angle_rad)
        start_y_mm = robot_y_mm + tof_sensor_offset_mm * np.sin(robot_angle_rad)
        distance_mm = true_map.raycast((start_x_mm, start_y_mm), robot_angle_rad, tof_default_distance_mm)

        if add_uncertainty:
            robot_angle_rad = np.random.normal(robot_angle_rad, np.radians(5))
            robot_x_mm = np.random.normal(robot_x_mm, 3)
            robot_y_mm = np.random.normal(robot_y_mm, 3)
            distance_mm = np.random.normal(distance_mm, 5)

        yield (robot_x_mm, robot_y_mm, robot_angle_rad, distance_mm)

        i += 1


def draw_robot(img, robot_x_mm, robot_y_mm, robot_angle_rad, robot_radius_mm, color):
    cv2.circle(img, (int(robot_x_mm), int(robot_y_mm)),
               robot_radius_mm, color, 1)
    cv2.line(img, (int(robot_x_mm), int(robot_y_mm)), (int(robot_x_mm + robot_radius_mm *
             np.cos(robot_angle_rad)), int(robot_y_mm + robot_radius_mm * np.sin(robot_angle_rad))), color, 1)


def main():

    width_mm = 640
    height_mm = 480
    cell_size_mm = 5
    robot_radius_mm = 35
    tof_sensor_offset_mm = robot_radius_mm
    tof_max_distance_mm = 1500
    tof_default_distance_mm = 8000
    width_px = width_mm
    height_px = height_mm

    true_map = envmap.Grid_map(
        width_mm=width_mm, height_mm=height_mm, cell_size_mm=1)
    true_map.line_normalized((0.2, 0.3), (0.8, 0.1))
    true_map.line_normalized((0.5, 0.7), (0.8, 0.7))
    true_map.line_normalized((0.1, 0.1), (0.1, 0.9))
    true_map.line_normalized((0.7, 0.3), (0.7, 0.7))
    true_map.line_normalized((0.4, 0.6), (0.4, 0.9))
    cv2.imshow('True map', true_map.to_image(height_px, width_px))

    constructed_map = envmap.Constructed_map(
        width_mm=width_mm, height_mm=height_mm, cell_size_mm=cell_size_mm)

    cv2.namedWindow('Constructed map')
    cv2.namedWindow('Samples')

    data_generator = robot_data_generator(true_map, tof_sensor_offset_mm, tof_default_distance_mm, True)

    while(True):

        robot_x_mm, robot_y_mm, robot_angle_rad, distance_mm = next(data_generator)

        constructed_map.construct((robot_x_mm, robot_y_mm), robot_angle_rad,
                                  robot_radius_mm, distance_mm, tof_sensor_offset_mm, tof_max_distance_mm)

        img = constructed_map.to_image(height=height_px, width=width_px)
        img = 1-img
        draw_robot(img, robot_x_mm, robot_y_mm, robot_angle_rad, robot_radius_mm, 0)
        cv2.imshow('Constructed map', img)

        cv2.imshow('Samples', constructed_map.samples_image(height=height_px, width=width_px))

        k = cv2.waitKey(1)
        if k == 27:  # Esc
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
