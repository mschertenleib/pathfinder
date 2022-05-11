import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

import EPuck2


def update_view():
    ax.clear()
    ax.set_title('Robot simulator')
    ax.set_aspect('equal', 'box')
    ax.set_xlim([0, width_mm])
    ax.set_ylim([0, height_mm])
    ax.set_xlabel('millimeters')
    ax.set_ylabel('millimeters')

    robot.draw(ax, '#ff0000')
    robot.draw_trail(ax, '#0000ff')

    fig.canvas.draw()
    fig.canvas.flush_events()


def on_next_clicked(event):
    global current_move, moves

    steps_l, steps_r = moves[current_move]
    print('Move:', steps_l, steps_r)
    robot.move_steps(int(steps_l), int(steps_r))
    update_view()
    current_move = (current_move + 1) % len(moves)


if __name__ == '__main__':

    width_mm = 800
    height_mm = 600

    robot = EPuck2.EPuck2(x_mm=width_mm / 2, y_mm=height_mm / 2, angle_rad=np.pi / 6)

    STEPS_PER_90_DEG = EPuck2.EPuck2.WHEEL_SPACING_MM / \
        2 * np.pi / 2 / EPuck2.EPuck2.MM_PER_STEP
    STEPS_PER_90_DEG_ONE_WHEEL = STEPS_PER_90_DEG * 2

    moves = [(1000, 1000),
             (0, STEPS_PER_90_DEG_ONE_WHEEL)]
             #(1000, 1000),
             #(-STEPS_PER_90_DEG, STEPS_PER_90_DEG),
             #(1000, 1000),
             #(0, STEPS_PER_90_DEG_ONE_WHEEL),
             #(1000, 1000),
             #(-STEPS_PER_90_DEG, STEPS_PER_90_DEG)]

    current_move = 0

    # Create figures and subplots
    fig = plt.figure()
    ax = fig.add_subplot()
    plt.subplots_adjust(bottom=0.2)

    button_color = 'lightgoldenrodyellow'
    button_hovercolor = '0.9'
    next_button_ax = plt.axes([0.4, 0.02, 0.2, 0.06])
    next_button = Button(next_button_ax, 'Next',
                         color=button_color, hovercolor=button_hovercolor)
    next_button.on_clicked(on_next_clicked)
    
    #robot.read_command_file('test_instructions.txt')
    
    update_view()

    plt.show()
