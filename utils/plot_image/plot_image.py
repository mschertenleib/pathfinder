import matplotlib.pyplot as plt
import numpy as np
import serial
import struct
import sys


def rgb565_to_r8g8b8(rgb565):
    r5 = (rgb565 & 0b1111_1000_0000_0000) >> 11
    g6 = (rgb565 & 0b0000_0111_1110_0000) >> 5
    b5 = (rgb565 & 0b0000_0000_0001_1111) >> 0
    rp = (float)(r5) / 31.0
    gp = (float)(g6) / 63.0
    bp = (float)(b5) / 31.0
    r8 = (np.uint8)(rp * 255.0)
    g8 = (np.uint8)(gp * 255.0)
    b8 = (np.uint8)(bp * 255.0)
    return [r8, g8, b8]


# reads the data in uint8 from the serial
def read_uint8_serial(port):
    state = 0

    while state != 5:

        # reads 1 byte
        c1 = port.read(1)
        # timeout condition
        if c1 == b'':
            print('Timeout...')
            return []

        if state == 0:
            if c1 == b'S':
                state = 1
            else:
                state = 0
        elif state == 1:
            if c1 == b'T':
                state = 2
            elif c1 == b'S':
                state = 1
            else:
                state = 0
        elif state == 2:
            if c1 == b'A':
                state = 3
            elif c1 == b'S':
                state = 1
            else:
                state = 0
        elif state == 3:
            if c1 == b'R':
                state = 4
            elif c1 == b'S':
                state = 1
            else:
                state = 0
        elif state == 4:
            if c1 == b'T':
                state = 5
            elif c1 == b'S':
                state = 1
            else:
                state = 0

    # reads the size
    # converts as uint16 in little endian the two bytes read
    size = struct.unpack('<H', port.read(2))
    # removes the second element which is void
    size = size[0]

    # reads the data
    rcv_buffer = port.read(size)
    data = []

    # if we receive the good amount of data, we convert them
    if len(rcv_buffer) == size:
        for i in range(0, size, 2):
            # convert to uint16 big endian
            rgb565 = struct.unpack_from('>H', rcv_buffer, i)[0]
            rgb = rgb565_to_r8g8b8(rgb565)
            data.append(rgb)

        print('Received !')
        return data
    else:
        print('Timeout...')
        return []


if __name__ == '__main__':

    # test if the serial port as been given as argument in the terminal
    if len(sys.argv) != 2:
        print(
            'Usage: {} <port>'.format(sys.argv[0]))
        sys.exit(0)

    port = sys.argv[1]
    print('Connecting to port {}'.format(port))
    try:
        port = serial.Serial(port)
    except:
        print('Cannot connect to the e-puck2')
        sys.exit(0)

    image_subsampling = 4
    image_width = 300 // image_subsampling
    image_height = 256 // image_subsampling
    filename = 'images/image{}.png'
    num_images = 100
    save = True

    plt.ion()
    fig = plt.figure()

    for i in range(num_images):
        cam_data = read_uint8_serial(port)

        if len(cam_data) > 0:
            cam_data = np.reshape(cam_data, (image_height, image_width, 3))
            plt.imshow(cam_data)
            if save:
                plt.imsave(filename.format(i), cam_data)
            fig.canvas.draw()
            fig.canvas.flush_events()
