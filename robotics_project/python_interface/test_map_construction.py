import numpy as np
import cv2
import environment_map as envmap
import EPuck2


def main():

    width_mm = 800
    height_mm = 800
    cell_size_mm = 10
    
    ROBOT_RADIUS_MM = EPuck2.EPuck2.RADIUS_MM
    TOF_SENSOR_OFFSET_MM = EPuck2.EPuck2.TOF_SENSOR_OFFSET_MM
    TOF_MAX_DISTANCE_MM = EPuck2.EPuck2.TOF_MAX_DISTANCE_MM
    WALKABLE_MM = ROBOT_RADIUS_MM * 2
    width_px = width_mm
    height_px = height_mm

    true_map = envmap.Environment_map(width_mm, height_mm, cell_size_mm=1)
    true_map.set_all_free()
    true_map.set_occupied_line_normalized((0.2, 0.3), (0.8, 0.1))
    true_map.set_occupied_line_normalized((0.5, 0.7), (0.8, 0.7))
    true_map.set_occupied_line_normalized((0.1, 0.1), (0.1, 0.9))
    true_map.set_occupied_line_normalized((0.7, 0.3), (0.7, 0.7))
    true_map.set_occupied_rectangle_normalized((0.38, 0.65), (0.4, 1))

    constructed_map = envmap.Environment_map(
        width_mm=width_mm, height_mm=height_mm, cell_size_mm=cell_size_mm)

    data_generator = true_map.robot_data_generator(
        TOF_SENSOR_OFFSET_MM, TOF_MAX_DISTANCE_MM)#, True)

    WALKABLE_COLOR = (255, 255, 0)
    ROBOT_COLOR = (0, 255, 0)
    GOAL_COLOR = (255, 0, 0)
    PATH_COLOR = (0, 0, 255)

    while True:

        robot_x_mm, robot_y_mm, robot_angle_rad, distance_mm = next(
            data_generator)

        constructed_map.construct((robot_x_mm, robot_y_mm), robot_angle_rad,
                                  ROBOT_RADIUS_MM, distance_mm, TOF_SENSOR_OFFSET_MM, TOF_MAX_DISTANCE_MM)

        map_bgr = constructed_map.as_image_with_walkable(
            WALKABLE_MM, WALKABLE_COLOR)
        map_bgr = cv2.resize(map_bgr, dsize=(
            width_px, height_px), interpolation=cv2.INTER_NEAREST)

        # Draw robot
        center_mm = (int(robot_x_mm + cell_size_mm / 2),
                     int(robot_y_mm + cell_size_mm / 2))
        front_mm = (int(center_mm[0] + ROBOT_RADIUS_MM * np.cos(robot_angle_rad)),
                    int(center_mm[1] + ROBOT_RADIUS_MM * np.sin(robot_angle_rad)))
        cv2.circle(map_bgr, center_mm, int(ROBOT_RADIUS_MM), ROBOT_COLOR, 2)
        cv2.line(map_bgr, center_mm, front_mm, ROBOT_COLOR, 2)

        # Draw goal
        goal_mm = (int(width_mm * 0.25), int(height_mm * 0.75))
        cv2.circle(map_bgr, np.add(goal_mm, (cell_size_mm // 2,
                   cell_size_mm // 2)), cell_size_mm // 2, GOAL_COLOR, 2)
        
        # Draw path
        path = constructed_map.find_path(
            (robot_x_mm, robot_y_mm), goal_mm, WALKABLE_MM)
        if len(path) >= 2:
            for i in range(len(path) - 1):
                start = (int(path[i][0]), int(path[i][1]))
                end = (int(path[i + 1][0]), int(path[i + 1][1]))
                cv2.line(map_bgr, start, end, PATH_COLOR, 2)

        cv2.imshow('Constructed map', map_bgr)

        k = cv2.waitKey(1)
        if k == 27:  # Esc
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
