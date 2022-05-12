from turtle import speed
import move


import struct
import serial
import numpy as np


"""
!BEEP <frequency_hz>            Emit sound at given frequency for 100ms
!CLR <x_cm> <y_cm> <angle_rad>  Set position, clean movement buffer on the robot
!MOVE <size> <data>             Give a set of moves
!PIC                            Ask for an image
!POS                            Get current position
!SCAN <turns>                   Turn in place while getting distance readings
!STOP                           Stop motors, clean movement buffer on the robot
"""


def open_port(port: str):
    ser = serial.Serial()
    try:
        ser = serial.Serial(port, 57600, timeout=5)
        print(f'Opened port {port}')
        return ser
    except serial.SerialException as e:
        print(e)


def beep(ser: serial.Serial, frequency_Hz: int):
    ser.write('!BEEP'.encode('ascii'))
    ser.write(struct.pack('<h', frequency_Hz))


def clean_and_set(ser: serial.Serial, x_mm, y_mm, angle_rad):
    ser.write('!CLR'.encode('ascii'))
    ser.write(struct.pack('<f', x_mm / 10))
    ser.write(struct.pack('<f', y_mm / 10))
    ser.write(struct.pack('<f', angle_rad))


def move_robot(ser: serial.Serial, move: move.Move):
    ser.write('!MOVE'.encode('ascii'))

    num_moves = len(move.command)
    ser.write(struct.pack('<h', num_moves))

    for movement_step in move.command:
        for value in movement_step:
            ser.write(struct.pack('<h', value))
    
    ser.write('END'.encode('ascii'))

    print(f'Sent !MOVE: {num_moves} moves: {move.command}')


def acquire_image(ser: serial.Serial):
    # Send command
    ser.write('!PIC'.encode('ascii'))

    width_bytes = ser.read(2)
    image_width = struct.unpack('<H', width_bytes)[0]
    height_bytes = ser.read(2)
    image_height = struct.unpack('<H', height_bytes)[0]
    buffer_size = image_width * image_height * 2
    
    rcv_buffer = ser.read(buffer_size)

    image = []

    if len(rcv_buffer) != buffer_size:
        print(
            f'Expected {buffer_size} bytes of image data, received {len(rcv_buffer)}')
        return

    # If we received the good amount of data, we convert them
    for i in range(0, buffer_size, 2):
        # Convert to uint16
        rgb565 = struct.unpack_from('>H', rcv_buffer, i)[0]
        rgb = rgb565_to_rgb888(rgb565)
        image.append(rgb)

    print('Image received !')
    return (image_width, image_height, image)


def get_robot_pos(ser: serial.Serial):
    # Send command
    ser.write('!POS'.encode('ascii'))

    # Receive robot data
    robot_x_mm = struct.unpack('<f', ser.read(4))[0] * 10
    robot_y_mm = struct.unpack('<f', ser.read(4))[0] * 10
    robot_angle_rad = struct.unpack('<f', ser.read(4))[0]
    return (robot_x_mm, robot_y_mm, robot_angle_rad)


def request_scan(ser: serial.Serial):
    ser.write('!SCAN'.encode('ascii'))
    
    data_bytes = ser.read(12)
    if len(data_bytes) != 12:
        print(f'Read only {len(data_bytes)} out of 12 in request_scan')
        return
    
    robot_x_mm = struct.unpack('<f', data_bytes[0:4])[0] * 10
    robot_y_mm = struct.unpack('<f', data_bytes[4:8])[0] * 10
    robot_angle_rad = struct.unpack('<f', data_bytes[8:12])[0]
    return (robot_x_mm, robot_y_mm, robot_angle_rad)


def scan_generator(ser: serial.Serial):

    while True:
        data_bytes = ser.read(6)
        
        if len(data_bytes) != 6:
            print(f'Read only {len(data_bytes)} out of 6 in get_next_scan')
            return
        
        # Stop condition
        if data_bytes == bytes([0xff] * 6):
            return
        
        angle_rad = struct.unpack('<f', data_bytes[0:4])[0]
        distance_mm = struct.unpack('<H', data_bytes[4:6])[0]
        yield angle_rad, distance_mm


def stop_robot(ser: serial.Serial):
    ser.write('!STOP'.encode('ascii'))


def send_instruction_file(ser: serial.Serial, filename: str):

    file = open(filename, 'r')
    
    instruction = file.readline()
    
    if instruction.find('!BEEP') != -1:
        frequency_strings = file.readLine().split()
        frequency_Hz = int(frequency_strings[0])
        beep(ser, frequency_Hz)
        
    elif instruction.find('!CLR') != -1:
        data_strings = file.readline().split()
        x_mm = float(data_strings[0])
        y_mm = float(data_strings[1])
        angle_rad = float(data_strings[2])
        clean_and_set(ser, x_mm, y_mm, angle_rad)
        
    elif instruction.find('!MOVE') != -1:
        size_string = file.readline().split()
        size = int(size_string[0])
        move_to_send = move.Move(0, 0)
        for i in range(size):
            data_strings = file.readline().split()
            speed_left = int(data_strings[0])
            speed_right = int(data_strings[1])
            duration = int(data_strings[2])
            move_to_send.command.append((speed_left, speed_right, duration))
        move_robot(ser, move_to_send)
        
    elif instruction.find('!PIC') != -1:
        return acquire_image(ser)
    
    elif instruction.find('!POS') != -1:
        return get_robot_pos(ser)
    
    elif instruction.find('!SCAN') != -1:
        return scan(ser)
    
    elif instruction.find('!STOP') != -1:
        stop_robot(ser)


def rgb565_to_rgb888(rgb565):
    r5 = (rgb565 & 0b1111_1000_0000_0000) >> 11
    g6 = (rgb565 & 0b0000_0111_1110_0000) >> 5
    b5 = (rgb565 & 0b0000_0000_0001_1111) >> 0
    rp = (float)(r5) / 31.0
    gp = (float)(g6) / 63.0
    bp = (float)(b5) / 31.0
    r8 = (np.uint8)(rp * 255.0)
    g8 = (np.uint8)(gp * 255.0)
    b8 = (np.uint8)(bp * 255.0)
    return (r8, g8, b8)