import matplotlib.pyplot as plt
import numpy as np
import serial
import struct
import sys


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
    size = struct.unpack('<H', port.read(2))[0]

    # reads the data
    rcv_buffer = port.read(size)
    data = []

    # if we receive the good amount of data, we convert them
    if len(rcv_buffer) == size:
        # convert to uint16 big endian
        distance = struct.unpack_from('>H', rcv_buffer, i)[0]
        data.append(distance)

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
        port = serial.Serial(port, timeout=0.5)
    except:
        print('Cannot connect to the e-puck2')
        sys.exit(0)

    num_points = 100

    for i in range(num_points):
        distance = read_uint8_serial(port)

        if len(distance) > 0:
            distance = distance[0]
            print('Distance:', distance, 'mm')
