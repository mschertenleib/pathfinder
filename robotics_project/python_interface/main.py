from turtle import st
import EPuck2
import environment_map as envmap
import move
import communication as comm


import cv2
import serial
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons
from matplotlib.backend_bases import MouseButton
import matplotlib
matplotlib.use('TkAgg')


########################  Button events  ############################


def on_clear_comm_button_clicked(event):
    if ser and ser.is_open:
        ser.reset_input_buffer()


def on_reset_map_button_clicked(event):
    global constructed_map
    robot.trail.clear()
    constructed_map = envmap.Environment_map(
        width_mm=WIDTH_MM, height_mm=HEIGHT_MM, cell_size_mm=CELL_SIZE_MM)
    update_view()


def on_beep_button_clicked(event):
    if ser and ser.is_open:
        comm.beep(ser, 440)


def on_set_button_clicked(event):
    robot.trail.clear()
    robot.x_mm, robot.y_mm = current_move.points[-1]
    robot.angle_rad = 0
    current_move.reset_points(robot.x_mm, robot.y_mm)

    if ser and ser.is_open:
        comm.clear_and_set(ser, robot.x_mm, robot.y_mm, robot.angle_rad)

    update_view()


def on_get_button_clicked(event):
    if ser and ser.is_open:
        robot_data = comm.get_robot_pos(ser)
        if robot_data is not None:
            robot.x_mm, robot.y_mm, robot.angle_rad = robot_data
        update_view()


def on_stop_button_clicked(event):
    comm.stop_robot(ser)
    robot.trail.clear()


def on_scan_button_clicked(event):
    robot.trail.clear()
    if ser and ser.is_open:
        res = comm.request_scan(ser)
        if res is None:
            return
        (robot.x_mm, robot.y_mm, robot.angle_rad) = res

        scan_generator = comm.scan_generator(ser)

        for robot.angle_rad, distance_mm in scan_generator:
            constructed_map.construct((robot.x_mm, robot.y_mm), robot.angle_rad, EPuck2.EPuck2.RADIUS_MM,
                                      distance_mm, EPuck2.EPuck2.TOF_SENSOR_OFFSET_MM, EPuck2.EPuck2.TOF_MAX_DISTANCE_MM, LINE_THICKNESS)
            update_view()


def on_cancel_button_clicked(event):
    current_move.reset_points(robot.x_mm, robot.y_mm)
    robot.trail.clear()
    update_view()


def execute_current_move():
    robot.trail.clear()

    for speed_left, speed_right, duration in current_move.commands:
        robot.move_speed(speed_left, speed_right, duration)

    if ser and ser.is_open:
        comm.move_robot(ser, current_move)

    update_view()

    current_move.reset_points(robot.x_mm, robot.y_mm)


def on_stg_button_clicked(event):
    current_move.gen_stg_command(STEPS_PER_SECOND, robot.angle_rad)
    execute_current_move()


def on_smooth_turn_button_clicked(event):
    current_move.gen_smooth_turn_command(STEPS_PER_SECOND, robot.angle_rad)
    execute_current_move()


def on_bezier_button_clicked(event):
    APPROX_CURVE_STEPS = 20
    current_move.gen_bezier_path(APPROX_CURVE_STEPS)
    current_move.gen_bezier_command(STEPS_PER_SECOND, robot.angle_rad)
    execute_current_move()


def on_image_button_clicked(event):
    if not ser or not ser.is_open:
        return

    image_data = comm.acquire_image(ser)
    if not image_data:
        return
    (image_width, image_height, image_buffer) = image_data
    image = np.reshape(image_buffer, (image_height, image_width, 3))

    image_ax = fig.add_subplot(1, 2, 2)
    image_ax.clear()
    image_ax.set_title('Photo')
    # image_ax.get_xaxis().set_visible(False)
    # image_ax.get_yaxis().set_visible(False)
    image_ax.imshow(image)

    fig.canvas.draw()
    fig.canvas.flush_events()


#########################  Mouse event  #############################


def mouse_action_direct(mouse_xy_mm):
    current_move.points.append(mouse_xy_mm)


def mouse_action_shortest(mouse_xy_mm):
    # Find a path from the last point of the move set to the mouse position
    start_mm = current_move.points[-1]
    goal_mm = mouse_xy_mm
    path = constructed_map.find_path(
        start_mm, goal_mm, WALKABLE_MIN_RADIUS)
    if len(path) >= 2:
        # If a path exists, add it to the current move set
        for point_mm in path:
            current_move.points.append(point_mm)


def mouse_action_draw_free_line(mouse_xy_mm):
    global start_draw_mm
    if start_draw_mm is None:
        start_draw_mm = mouse_xy_mm
    else:
        constructed_map.set_free_line(start_draw_mm, mouse_xy_mm, int(
            1.5 * EPuck2.EPuck2.RADIUS_MM / CELL_SIZE_MM))
        start_draw_mm = None


def mouse_action_draw_occupied_line(mouse_xy_mm):
    global start_draw_mm
    if start_draw_mm is None:
        start_draw_mm = mouse_xy_mm
    else:
        constructed_map.set_occupied_line(start_draw_mm, mouse_xy_mm)
        start_draw_mm = None


def mouse_action_draw_free_rectangle(mouse_xy_mm):
    global start_draw_mm
    if start_draw_mm is None:
        start_draw_mm = mouse_xy_mm
    else:
        constructed_map.set_free_filled_rectangle(start_draw_mm, mouse_xy_mm)
        start_draw_mm = None


def mouse_action_draw_occupied_rectangle(mouse_xy_mm):
    global start_draw_mm
    if start_draw_mm is None:
        start_draw_mm = mouse_xy_mm
    else:
        constructed_map.set_occupied_rectangle(start_draw_mm, mouse_xy_mm)
        start_draw_mm = None


def mouse_action_draw_free_circle(mouse_xy_mm):
    constructed_map.set_free_filled_circle(
        mouse_xy_mm, 1.5 * EPuck2.EPuck2.RADIUS_MM)


def on_mouse_button_press(event):
    if event.inaxes is map_ax and event.button is MouseButton.LEFT:
        action = action_radio_button.value_selected
        mouse_xy_mm = (event.xdata, event.ydata)

        if action == ACTION_DIRECT:
            mouse_action_direct(mouse_xy_mm)
        elif action == ACTION_SHORTEST:
            mouse_action_shortest(mouse_xy_mm)
        else:
            robot.trail.clear()
            if action == ACTION_OCCUPIED_LINE:
                mouse_action_draw_occupied_line(mouse_xy_mm)
            elif action == ACTION_FREE_LINE:
                mouse_action_draw_free_line(mouse_xy_mm)
            elif action == ACTION_OCCUPIED_RECTANGLE:
                mouse_action_draw_occupied_rectangle(mouse_xy_mm)
            elif action == ACTION_FREE_RECTANGLE:
                mouse_action_draw_free_rectangle(mouse_xy_mm)
            elif action == ACTION_FREE_CIRCLE:
                mouse_action_draw_free_circle(mouse_xy_mm)

        update_view()


############################  View  #################################

def update_view():
    map_image = constructed_map.as_image_with_walkable(
        WALKABLE_MIN_RADIUS, (255, 255, 0))
    img = cv2.resize(map_image, dsize=(HEIGHT_PX, WIDTH_PX),
                     interpolation=cv2.INTER_NEAREST)

    map_ax.clear()
    map_ax.set_title('Map')
    map_ax.set_aspect('equal', 'box')
    map_ax.set_xlim([0, WIDTH_MM])
    map_ax.set_ylim([0, HEIGHT_MM])
    map_ax.set_xlabel('millimeters')
    map_ax.set_ylabel('millimeters')

    # Draw map
    map_ax.imshow(img)

    # Draw move
    current_move.draw_path(map_ax, '#0000ff')

    # Draw goal
    points_per_circle = 100
    angles_rad = np.linspace(0, 2 * np.pi, points_per_circle)
    goal_mm = current_move.points[len(current_move.points) - 1]
    pxs_mm = goal_mm[0] + CELL_SIZE_MM / 2 * np.cos(angles_rad)
    pys_mm = goal_mm[1] + CELL_SIZE_MM / 2 * np.sin(angles_rad)
    map_ax.plot(pxs_mm, pys_mm, color='#ff0000')

    # Draw robot trail
    robot.draw_trail(map_ax, color='#00ffff')

    # Draw robot
    robot.draw(map_ax, color='#00ff00')

    fig.canvas.draw()
    fig.canvas.flush_events()


##########################   main   #################################

ser = serial.Serial()
if len(sys.argv) >= 2:
    ser = comm.open_port(sys.argv[1])
else:
    print('Running without serial port')

WIDTH_MM = 999
HEIGHT_MM = 999
CELL_SIZE_MM = 20
WIDTH_PX = WIDTH_MM
HEIGHT_PX = HEIGHT_MM

LINE_THICKNESS = 1

WALKABLE_MIN_RADIUS = EPuck2.EPuck2.RADIUS_MM * 2

STEPS_PER_SECOND = 400

# Create robot simulator
robot = EPuck2.EPuck2(x_mm=WIDTH_MM/2, y_mm=HEIGHT_MM/2, angle_rad=0)

# Create move set handler
current_move = move.Move(robot.x_mm, robot.y_mm)

# Create map
constructed_map = envmap.Environment_map(
    width_mm=WIDTH_MM, height_mm=HEIGHT_MM, cell_size_mm=CELL_SIZE_MM)

# Create figures and subplots
fig = plt.figure('Pathfinder')
map_ax = fig.add_subplot(1, 2, 1)
plt.subplots_adjust(bottom=0.3)

# Create buttons
button_color = 'lightgoldenrodyellow'
button_hovercolor = '0.95'

BUTTON_WIDTH = 0.09
LARGE_BUTTON_WIDTH = 0.15
BUTTON_HEIGHT = 0.04
BUTTON_MARGIN = 0.01
RADIO_BUTTON_WIDTH = 0.3
RADIO_BUTTON_HEIGHT = 0.2

SMALL_BUTTON_START_X = BUTTON_MARGIN + LARGE_BUTTON_WIDTH + BUTTON_MARGIN
BUTTON_Y_HIGH = BUTTON_MARGIN + BUTTON_HEIGHT + BUTTON_MARGIN
BUTTON_SPACING = BUTTON_WIDTH + BUTTON_MARGIN

clear_comm_button_ax = plt.axes(
    [BUTTON_MARGIN, BUTTON_MARGIN, LARGE_BUTTON_WIDTH, BUTTON_HEIGHT])
reset_map_button_ax = plt.axes(
    [BUTTON_MARGIN, BUTTON_Y_HIGH, LARGE_BUTTON_WIDTH, BUTTON_HEIGHT])
beep_button_ax = plt.axes(
    [SMALL_BUTTON_START_X, BUTTON_Y_HIGH, BUTTON_WIDTH, BUTTON_HEIGHT])
set_button_ax = plt.axes(
    [SMALL_BUTTON_START_X + BUTTON_SPACING, BUTTON_Y_HIGH, BUTTON_WIDTH, BUTTON_HEIGHT])
get_button_ax = plt.axes(
    [SMALL_BUTTON_START_X + 2 * BUTTON_SPACING, BUTTON_Y_HIGH, BUTTON_WIDTH, BUTTON_HEIGHT])
stop_button_ax = plt.axes(
    [SMALL_BUTTON_START_X + 3 * BUTTON_SPACING, BUTTON_Y_HIGH, BUTTON_WIDTH, BUTTON_HEIGHT])
scan_button_ax = plt.axes(
    [SMALL_BUTTON_START_X + 4 * BUTTON_SPACING, BUTTON_Y_HIGH, BUTTON_WIDTH, BUTTON_HEIGHT])
cancel_button_ax = plt.axes(
    [SMALL_BUTTON_START_X + 4 * BUTTON_SPACING, BUTTON_MARGIN, BUTTON_WIDTH, BUTTON_HEIGHT])
stg_button_ax = plt.axes(
    [SMALL_BUTTON_START_X, BUTTON_MARGIN, BUTTON_WIDTH, BUTTON_HEIGHT])
smooth_turn_button_ax = plt.axes(
    [SMALL_BUTTON_START_X + BUTTON_SPACING, BUTTON_MARGIN, BUTTON_WIDTH, BUTTON_HEIGHT])
bezier_button_ax = plt.axes(
    [SMALL_BUTTON_START_X + 2 * BUTTON_SPACING, BUTTON_MARGIN, BUTTON_WIDTH, BUTTON_HEIGHT])
image_button_ax = plt.axes(
    [SMALL_BUTTON_START_X + 3 * BUTTON_SPACING, BUTTON_MARGIN, BUTTON_WIDTH, BUTTON_HEIGHT])
action_radio_button_ax = plt.axes(
    [SMALL_BUTTON_START_X + 5 * BUTTON_SPACING, BUTTON_MARGIN, RADIO_BUTTON_WIDTH, RADIO_BUTTON_HEIGHT])

clear_comm_button = Button(clear_comm_button_ax, 'Clear comm',
                           color='#ff9999', hovercolor=button_hovercolor)
reset_map_button = Button(reset_map_button_ax, 'Reset map',
                          color='#ff9999', hovercolor=button_hovercolor)
beep_button = Button(beep_button_ax, 'Beep',
                     color=button_color, hovercolor=button_hovercolor)
set_button = Button(set_button_ax, 'Set', color=button_color,
                    hovercolor=button_hovercolor)
get_button = Button(get_button_ax, 'Get', color=button_color,
                    hovercolor=button_hovercolor)
stop_button = Button(stop_button_ax, 'Stop',
                     color=button_color, hovercolor=button_hovercolor)
scan_button = Button(scan_button_ax, 'Scan',
                     color=button_color, hovercolor=button_hovercolor)
cancel_button = Button(cancel_button_ax, 'Cancel',
                       color=button_color, hovercolor=button_hovercolor)
stg_button = Button(stg_button_ax, 'STG', color=button_color,
                    hovercolor=button_hovercolor)
smooth_turn_button = Button(
    smooth_turn_button_ax, 'Smooth', color=button_color, hovercolor=button_hovercolor)
bezier_button = Button(bezier_button_ax, 'Bezier',
                       color=button_color, hovercolor=button_hovercolor)
image_button = Button(image_button_ax, 'Image',
                      color=button_color, hovercolor=button_hovercolor)

# Create action selector radio button
ACTION_DIRECT = 'Direct'
ACTION_SHORTEST = 'Shortest'
ACTION_OCCUPIED_LINE = 'Occupied line'
ACTION_FREE_LINE = 'Free line'
ACTION_OCCUPIED_RECTANGLE = 'Occupied rectangle'
ACTION_FREE_RECTANGLE = 'Free rectangle'
ACTION_FREE_CIRCLE = 'Free circle'
action_radio_button = RadioButtons(action_radio_button_ax, (ACTION_DIRECT, ACTION_SHORTEST, ACTION_OCCUPIED_LINE,
                                   ACTION_FREE_LINE, ACTION_OCCUPIED_RECTANGLE, ACTION_FREE_RECTANGLE, ACTION_FREE_CIRCLE))
start_draw_mm = None

clear_comm_button.on_clicked(on_clear_comm_button_clicked)
reset_map_button.on_clicked(on_reset_map_button_clicked)
beep_button.on_clicked(on_beep_button_clicked)
set_button.on_clicked(on_set_button_clicked)
get_button.on_clicked(on_get_button_clicked)
stop_button.on_clicked(on_stop_button_clicked)
scan_button.on_clicked(on_scan_button_clicked)
cancel_button.on_clicked(on_cancel_button_clicked)
stg_button.on_clicked(on_stg_button_clicked)
smooth_turn_button.on_clicked(on_smooth_turn_button_clicked)
bezier_button.on_clicked(on_bezier_button_clicked)
image_button.on_clicked(on_image_button_clicked)

# Connect events
fig.canvas.mpl_connect('button_press_event', on_mouse_button_press)

update_view()

# Set the robot position, signal with a beep
if ser and ser.is_open:
    comm.clear_and_set(ser, robot.x_mm, robot.y_mm, robot.angle_rad)
    comm.beep(ser, 440)

# Start the matplotlib main
plt.show()
