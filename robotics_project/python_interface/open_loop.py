import numpy as np
import cv2
import matplotlib.pyplot as plt
import serial
import struct
import sys
import environment_map as envmap
import EPuck2


def draw_robot(img, robot_x_mm, robot_y_mm, robot_angle_rad, robot_radius_mm, color):
    cv2.circle(img, (int(robot_x_mm), int(robot_y_mm)),
               int(robot_radius_mm), color, 1)
    cv2.line(img, (int(robot_x_mm), int(robot_y_mm)), (int(robot_x_mm + robot_radius_mm *
             np.cos(robot_angle_rad)), int(robot_y_mm + robot_radius_mm * np.sin(robot_angle_rad))), color, 1)


def main():

    width_mm = 640
    height_mm = 480
    cell_size_mm = 10
    ROBOT_RADIUS_MM = EPuck2.Epuck.RADIUS_MM
    TOF_SENSOR_OFFSET_MM = EPuck2.Epuck.TOF_SENSOR_OFFSET_MM
    TOF_MAX_DISTANCE_MM = EPuck2.Epuck.TOF_MAX_DISTANCE_MM
    WALKABLE_MM = ROBOT_RADIUS_MM * 1.5
    width_px = width_mm
    height_px = height_mm

    true_map = envmap.Environment_map(width_mm, height_mm, cell_size_mm=1)
    true_map.set_all_free()
    true_map.set_occupied_line_normalized((0.2, 0.3), (0.8, 0.1))
    true_map.set_occupied_line_normalized((0.5, 0.7), (0.8, 0.7))
    true_map.set_occupied_line_normalized((0.1, 0.1), (0.1, 0.9))
    true_map.set_occupied_line_normalized((0.7, 0.3), (0.7, 0.7))
    true_map.set_occupied_rectangle_normalized((0.38, 0.65), (0.4, 1))
    #cv2.imshow('True map', true_map.to_image(width=width_px, height=height_px))

    constructed_map = envmap.Environment_map(
        width_mm=width_mm, height_mm=height_mm, cell_size_mm=cell_size_mm)

    data_generator = true_map.robot_data_generator(
        TOF_SENSOR_OFFSET_MM, TOF_MAX_DISTANCE_MM)#, True)

    while True:

        robot_x_mm, robot_y_mm, robot_angle_rad, distance_mm = next(
            data_generator)

        constructed_map.construct((robot_x_mm, robot_y_mm), robot_angle_rad,
                                  ROBOT_RADIUS_MM, distance_mm, TOF_SENSOR_OFFSET_MM, TOF_MAX_DISTANCE_MM)

        map_image = constructed_map.to_image(height=height_px, width=width_px)
        walkable = constructed_map.walkable(WALKABLE_MM, height_px, width_px)

        img = cv2.merge([map_image] * 3)
        walkable_img = cv2.merge([walkable] * 3)

        draw_robot(img, robot_x_mm, robot_y_mm, robot_angle_rad,
                   ROBOT_RADIUS_MM, (0, 255, 0))
        draw_robot(walkable_img, robot_x_mm, robot_y_mm, robot_angle_rad,
                   ROBOT_RADIUS_MM, (0, 255, 0))

        goal_mm = (width_mm//4, height_mm*9//12)
        
        cv2.rectangle(img, goal_mm, np.add(
            goal_mm, (cell_size_mm, cell_size_mm)), (255, 0, 0), 2)
        cv2.rectangle(walkable_img, goal_mm, np.add(
            goal_mm, (cell_size_mm, cell_size_mm)), (255, 0, 0), 2)
        
        path = constructed_map.find_path(
            (robot_x_mm, robot_y_mm), goal_mm, WALKABLE_MM)
        if len(path) >= 2:
            for i in range(len(path)-1):
                start = (int(path[i][0]), int(path[i][1]))
                end = (int(path[i+1][0]), int(path[i+1][1]))
                cv2.line(img, start, end, (0, 0, 255), 2)
                cv2.line(walkable_img, start, end, (0, 0, 255), 2)
        
        cv2.imshow('Constructed map', img)
        cv2.imshow('Walkable', walkable_img)

        k = cv2.waitKey(1)
        if k == 27:  # Esc
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
