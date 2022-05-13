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


def on_set_button_clicked(event):
    robot.x_mm, robot.y_mm = current_move.data[-1]
    robot.angle_rad = 0
    current_move.reset_data(robot.x_mm, robot.y_mm)

    if ser and ser.is_open:
        comm.clean_and_set(ser, robot.x_mm, robot.y_mm, robot.angle_rad)

    update_view()


def on_get_button_clicked(event):
    if ser and ser.is_open:
        robot.x_mm, robot.y_mm, robot.angle_rad = comm.get_robot_pos(ser)
        print('Get pos:', robot.x_mm, robot.y_mm, robot.angle_rad)
        current_move.reset_data(robot.x_mm, robot.y_mm)

        update_view()


def on_send_instruction_button_clicked(event):
    if ser and ser.is_open:
        comm.send_instruction_file(ser, instruction_file)


def test_scan_generator(ser):
    for i in range(100):
        angle_rad = i / 100 * 2 * np.pi
        distance_mm = 250 + 100 * np.sin(4*angle_rad)
        yield angle_rad, distance_mm


def on_scan_button_clicked(event):
    if ser and ser.is_open:
        res = comm.request_scan(ser)
        if res is None:
            return
        (robot.x_mm, robot.y_mm, robot.angle_rad) = res
        print(robot.x_mm, robot.y_mm, robot.angle_rad)

        scan_generator = comm.scan_generator(ser)

        for robot.angle_rad, distance_mm in scan_generator:
            print(robot.angle_rad, distance_mm)
            constructed_map.construct((robot.x_mm, robot.y_mm), robot.angle_rad, EPuck2.EPuck2.RADIUS_MM,
                                      distance_mm, EPuck2.EPuck2.TOF_SENSOR_OFFSET_MM, EPuck2.EPuck2.TOF_MAX_DISTANCE_MM, line_thickness)
            update_view()


def on_bezier_button_clicked(event):
    print('Generating Bezier command')

    APPROX_PER_LINE = 3
    current_move.gen_bezier_path(APPROX_PER_LINE)
    current_move.gen_bez_command(STEPS_PER_SECOND, robot.angle_rad)
    current_move.gen_file(instruction_file)

    robot.trail = []

    robot.read_command_file(instruction_file)

    if ser and ser.is_open:
        comm.move_robot(ser, current_move)

    update_view()

    current_move.reset_data(robot.x_mm, robot.y_mm)


def on_stg_button_clicked(event):
    print('Generating Stop-Turn-Go command')

    current_move.gen_stg_command(STEPS_PER_SECOND, robot.angle_rad)
    current_move.gen_file(instruction_file)

    robot.trail = []

    robot.read_command_file(instruction_file)

    if ser and ser.is_open:
        comm.move_robot(ser, current_move)

    update_view()

    current_move.reset_data(robot.x_mm, robot.y_mm)

def on_smooth_turn_button_clicked(event):
    print('Generating smooth turn command')

    current_move.gen_smooth_turn_command(STEPS_PER_SECOND, robot.angle_rad)
    
    current_move.gen_file(instruction_file)

    robot.trail = []

    robot.read_command_file(instruction_file)

    if ser and ser.is_open:
        comm.move_robot(ser, current_move)

    update_view()

    current_move.reset_data(robot.x_mm, robot.y_mm)


def on_image_button_clicked(event):

    if not ser or not ser.is_open:
        return

    print('Acquiring image')

    image_data = comm.acquire_image(ser)
    if not image_data:
        return
    (image_width, image_height, image_buffer) = image_data
    print((image_width, image_height, image_buffer))
    image = np.reshape(image_buffer, (image_height, image_width, 3))

    image_ax.clear()
    image_ax.set_title('Photo')
    image_ax.get_xaxis().set_visible(False)
    image_ax.get_xaxis().set_visible(False)
    image_ax.imshow(image)

    fig.canvas.draw()
    fig.canvas.flush_events()


def on_radio_button_clicked(label):
    global use_shortest_path
    if label == LABEL_SHORTEST:
        use_shortest_path = True
    if label == LABEL_DIRECT:
        use_shortest_path = False


def on_mouse_button_press(event):
    if event.inaxes is map_ax:
        if event.button is MouseButton.LEFT:
            goal_mm = (event.xdata, event.ydata)
            if use_shortest_path:
                start_mm = current_move.data[-1]
                path = constructed_map.find_path(
                    start_mm, goal_mm, WALKABLE_MIN_RADIUS)
                if len(path) >= 2:
                    for point_mm in path:
                        current_move.data.append(point_mm)
                else:
                    print('Can not find a path to the specified goal')
            else:
                current_move.data.append(goal_mm)

            update_view()


def update_view():
    map_image = constructed_map.as_image_with_walkable(
        WALKABLE_MIN_RADIUS, (255, 255, 0))
    img = cv2.resize(map_image, dsize=(height_px, width_px),
                     interpolation=cv2.INTER_NEAREST)

    map_ax.clear()
    map_ax.set_title('Map')
    map_ax.set_aspect('equal', 'box')
    map_ax.set_xlim([0, width_mm])
    map_ax.set_ylim([0, height_mm])
    map_ax.set_xlabel('millimeters')
    map_ax.set_ylabel('millimeters')

    # Draw map
    map_ax.imshow(img)

    # Draw robot
    robot.draw(map_ax, color='#00ff00')

    # Draw robot trail
    robot.draw_trail(map_ax, color='#00ffff')

    # Draw move
    current_move.draw_path(map_ax, '#0000ff')

    # Draw goal
    points_per_circle = 100
    angles_rad = np.linspace(0, 2 * np.pi, points_per_circle)
    goal_mm = current_move.data[len(current_move.data) - 1]
    pxs_mm = goal_mm[0] + cell_size_mm / 2 * np.cos(angles_rad)
    pys_mm = goal_mm[1] + cell_size_mm / 2 * np.sin(angles_rad)
    map_ax.plot(pxs_mm, pys_mm, color='#ff0000')

    fig.canvas.draw()
    fig.canvas.flush_events()


ser = serial.Serial()
if len(sys.argv) >= 2:
    ser = comm.open_port(sys.argv[1])
else:
    print('Running without serial port')

width_mm = 999
height_mm = 999
cell_size_mm = 20
width_px = width_mm
height_px = height_mm

line_thickness = 1

WALKABLE_MIN_RADIUS = EPuck2.EPuck2.RADIUS_MM * 2

STEPS_PER_SECOND = 400

instruction_file = 'instructions.txt'

robot = EPuck2.EPuck2(x_mm=width_mm / 2, y_mm=height_mm / 2, angle_rad=0)
current_move = move.Move(robot.x_mm, robot.y_mm)

constructed_map = envmap.Environment_map(
    width_mm=width_mm, height_mm=height_mm, cell_size_mm=cell_size_mm)

# Create figures and subplots
fig = plt.figure()
map_ax = fig.add_subplot(1, 2, 1)
image_ax = fig.add_subplot(1, 2, 2)
plt.subplots_adjust(bottom=0.2)

# Create buttons
button_color = 'lightgoldenrodyellow'
button_hovercolor = '0.95'

BUTTON_Y = 0.025
BUTTON_HEIGHT = 0.04
NUM_BUTTONS = 9
BUTTON_MARGIN = 0.01
BUTTON_SPACING = (1 - BUTTON_MARGIN) / NUM_BUTTONS
BUTTON_WIDTH = BUTTON_SPACING - BUTTON_MARGIN

send_instruction_button_ax = plt.axes(
    [BUTTON_MARGIN + 0 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
set_button_ax = plt.axes(
    [BUTTON_MARGIN + 1 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
get_button_ax = plt.axes(
    [BUTTON_MARGIN + 2 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
bezier_button_ax = plt.axes(
    [BUTTON_MARGIN + 3 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
stg_button_ax = plt.axes(
    [BUTTON_MARGIN + 4 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
smooth_turn_button_ax = plt.axes(
    [BUTTON_MARGIN + 5 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
scan_button_ax = plt.axes(
    [BUTTON_MARGIN + 6 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
image_button_ax = plt.axes(
    [BUTTON_MARGIN + 7 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, BUTTON_HEIGHT])
radio_button_ax = plt.axes(
    [BUTTON_MARGIN + 8 * BUTTON_SPACING, BUTTON_Y, BUTTON_WIDTH, 3 * BUTTON_HEIGHT])

set_button = Button(set_button_ax, 'Set',
                    color=button_color, hovercolor=button_hovercolor)
get_button = Button(get_button_ax, 'Get',
                    color=button_color, hovercolor=button_hovercolor)
send_instruction_button = Button(
    send_instruction_button_ax, 'Send', color=button_color, hovercolor=button_hovercolor)
bezier_button = Button(bezier_button_ax, 'Bezier',
                       color=button_color, hovercolor=button_hovercolor)
stg_button = Button(stg_button_ax, 'STG',
                    color=button_color, hovercolor=button_hovercolor)
smooth_turn_button = Button(smooth_turn_button_ax, 'Smooth',
                    color=button_color, hovercolor=button_hovercolor)
image_button = Button(image_button_ax, 'Photo',
                      color=button_color, hovercolor=button_hovercolor)
scan_button = Button(scan_button_ax, 'Scan',
                     color=button_color, hovercolor=button_hovercolor)

LABEL_DIRECT = 'Direct'
LABEL_SHORTEST = 'Shortest'
radio_button = RadioButtons(
    radio_button_ax, (LABEL_DIRECT, LABEL_SHORTEST))

set_button.on_clicked(on_set_button_clicked)
get_button.on_clicked(on_get_button_clicked)
send_instruction_button.on_clicked(on_send_instruction_button_clicked)
bezier_button.on_clicked(on_bezier_button_clicked)
stg_button.on_clicked(on_stg_button_clicked)
smooth_turn_button.on_clicked(on_smooth_turn_button_clicked)
scan_button.on_clicked(on_scan_button_clicked)
image_button.on_clicked(on_image_button_clicked)
radio_button.on_clicked(on_radio_button_clicked)

use_shortest_path = False

# Connect events
fig.canvas.mpl_connect('button_press_event', on_mouse_button_press)

update_view()

if ser and ser.is_open:
    comm.clean_and_set(ser, robot.x_mm, robot.y_mm, robot.angle_rad)
    comm.beep(ser, 440)

# Start the matplotlib main
plt.show()
