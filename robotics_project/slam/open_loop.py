import numpy as np
import cv2
import environment_map


def get_robot_data(true_map: environment_map.Environment_map, tof_sensor_offset_mm):
    num_samples = 100
    arr = np.zeros((num_samples, 4))
    robot_x_mm = true_map.width_mm / 2
    robot_y_mm = true_map.height_mm / 2
    for i in range(num_samples):
        angle_rad = i / num_samples * 2 * np.pi
        start_x_mm = robot_x_mm + tof_sensor_offset_mm * np.cos(angle_rad)
        start_y_mm = robot_y_mm + tof_sensor_offset_mm * np.sin(angle_rad)
        exact_distance_mm = true_map.raycast(start_x_mm, start_y_mm, angle_rad)
        distance_mm = exact_distance_mm * \
            (1 + (np.random.rand() * 2 - 1) * 0.1)
        arr[i] = (robot_x_mm, robot_y_mm, angle_rad, distance_mm)
    return arr


def draw_robot(img, robot_x_mm, robot_y_mm, robot_angle_rad, robot_radius_mm):
    color = (0, 0, 255)

    cv2.circle(img, (int(robot_x_mm), int(robot_y_mm)),
               robot_radius_mm, color, 2)

    cv2.line(img, (int(robot_x_mm), int(robot_y_mm)), (int(robot_x_mm + robot_radius_mm *
             np.cos(robot_angle_rad)), int(robot_y_mm + robot_radius_mm * np.sin(robot_angle_rad))), color, 2)


if __name__ == '__main__':

    width_mm = 640
    height_mm = 480
    cell_size_mm = 5
    robot_radius_mm = 50
    tof_sensor_offset_mm = robot_radius_mm
    robot_delta_mm = robot_radius_mm
    width_px = width_mm
    height_px = height_mm

    true_map = environment_map.Environment_map(
        width_mm=width_mm, height_mm=height_mm, cell_size_mm=1)
    true_map.generate_horizontal_wall(x1=0.1, x2=0.9, y=0.2)
    true_map.generate_horizontal_wall(x1=0.5, x2=0.8, y=0.7)
    true_map.generate_vertical_wall(x=0.1, y1=0.1, y2=0.9)
    true_map.generate_vertical_wall(x=0.7, y1=0.3, y2=0.7)
    true_map.generate_vertical_wall(x=0.4, y1=0.6, y2=0.9)

    constructed_map = environment_map.Environment_map(
        width_mm=width_mm, height_mm=height_mm, cell_size_mm=cell_size_mm)

    cv2.namedWindow('image')  # , cv2.WINDOW_NORMAL)

    robot_data = get_robot_data(true_map, tof_sensor_offset_mm)

    for robot_x_mm, robot_y_mm, robot_angle_rad, distance_mm in robot_data:

        img = np.ones((height_px, width_px, 3), np.uint8) * 255

        if distance_mm < np.Infinity:
            wall_x_mm = robot_x_mm + (distance_mm + tof_sensor_offset_mm) * \
                np.cos(robot_angle_rad)
            wall_y_mm = robot_y_mm + (distance_mm + tof_sensor_offset_mm) * \
                np.sin(robot_angle_rad)
            constructed_map.set_at_mm(wall_x_mm, wall_y_mm, True)
            cv2.line(img, (int(robot_x_mm), int(robot_y_mm)),
                     (int(wall_x_mm), int(wall_y_mm)), (200, 200, 200), cell_size_mm)

        draw_robot(img, robot_x_mm, robot_y_mm,
                   robot_angle_rad, robot_radius_mm)

        true_map.to_image(img, (255, 0, 0))
        constructed_map.to_image(img, (0, 0, 0))

        cv2.imshow('image', img)
        cv2.waitKey(1)

        #k = cv2.waitKey(0)
        # if k == ord('w'):
        #    robot_y_mm -= robot_delta_mm
        # if k == ord('a'):
        #    robot_x_mm -= robot_delta_mm
        # if k == ord('s'):
        #    robot_y_mm += robot_delta_mm
        # if k == ord('d'):
        #    robot_x_mm += robot_delta_mm
        #if k == 27:  # Esc
        #    break

    cv2.waitKey(0)
    cv2.destroyAllWindows()
