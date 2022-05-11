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
    ser.write(struct.pack('>h', frequency_Hz))


def clean_and_set(ser: serial.Serial, x_mm, y_mm, angle_rad):
    ser.write('!CLR'.encode('ascii'))
    ser.write(struct.pack('>f', x_mm / 10))
    ser.write(struct.pack('>f', y_mm / 10))
    ser.write(struct.pack('>f', angle_rad))


def move_robot(ser: serial.Serial, move: move.Move):
    ser.write('!MOVE'.encode('ascii'))

    num_moves = len(move.command)
    num_values = num_moves * 3
    ser.write(struct.pack('>h', num_values))

    for movement_step in move.command:
        for value in movement_step:
            ser.write(struct.pack('>h', value))
    
    ser.write('END'.encode('ascii'))

    print(f'Sent !MOVE: {num_moves} moves\r\n{move.command}')


def acquire_image(ser: serial.Serial):
    # Send command
    ser.write('!PIC'.encode('ascii'))

    # Receive image data. First the size of the buffer as uint16, then the pixel data as rgb565
    buffer_size = struct.unpack('>H', ser.read(2))[0]
    rcv_buffer = ser.read(buffer_size)

    image = []

    if len(rcv_buffer) == buffer_size:
        print(
            f'Expected {buffer_size} bytes of image data, received {len(rcv_buffer)}')
        return []

    # If we received the good amount of data, we convert them
    for i in range(0, buffer_size, 2):
        # Convert to uint16
        rgb565 = struct.unpack_from('>H', rcv_buffer, i)[0]
        rgb = rgb565_to_rgb888(rgb565)
        image.append(rgb)

    print('Image received !')
    return image


def get_robot_pos(ser: serial.Serial):
    # Send command
    ser.write('!POS'.encode('ascii'))

    # Receive robot data
    robot_x_mm = struct.unpack('>f', ser.read(4))[0] * 10
    robot_y_mm = struct.unpack('>f', ser.read(4))[0] * 10
    robot_angle_rad = struct.unpack('>f', ser.read(4))[0] * 10
    return (robot_x_mm, robot_y_mm, robot_angle_rad)


def scan(ser: serial.Serial):
    ser.write('!SCAN'.encode('ascii'))
    TURNS = 1
    ser.write(struct.pack('>B', TURNS))

    i = 0
    ret = str()
    time = 1e6
    while i < time:
        bytesToRead = ser.inWaiting()
        serstr = ser.read(bytesToRead)
        if serstr != b'':
            i = 0
            ascii_string = serstr.decode('ascii')
            print(ascii_string)
            if len(ret) == 0 and ascii_string.find('SCAN') != 0:
                print('Unexpected bytes received')
                return str()

            ret += ascii_string
            if ascii_string.find('END') != -1:
                break
        else:
            i += 1
    print('stopped listening.', i)
    ser.close()
    return ret


def stop_robot(ser: serial.Serial):
    ser.write('!STOP'.encode('ascii'))


def send_instruction_file(ser: serial.Serial, filename: str):

    file = open(filename, 'r')
    
    instruction = file.readline()
    
    if instruction.find('!BEEP') != -1:
        frequency_strings = file.readLine().split()
        frequency_Hz = int(frequency_strings[0])
        beep(ser, frequency_Hz)
        
    elif instruction.find('!CLS') != -1:
        data_strings = file.readline().split()
        x_mm = float(data_strings[0])
        y_mm = float(data_strings[1])
        angle_rad = float(data_strings[2])
        clean_and_set(ser, x_mm, y_mm, angle_rad)
        
    elif instruction.find('!MOVE') != -1:
        size_string = file.readline().split()
        size = int(size_string[0])
        move_to_send = move.Move(0, 0)
        for i in range(size // 3):
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


def send_move_instruction_file(ser: serial.Serial, filename: str):

    file = open(filename, 'r')

    command = []

    instr = file.readline().rstrip('\n')
    len = int(file.readline().rstrip('\n'))
    for i in range(len):
        for inf in file.readline().split():
            if(inf == 'END'):
                print('Finished reading', filename)
                break
            command.append(int(inf))

    for c in instr:  # send instr like "MOVE"
        ser.write(c.encode('utf-8'))

    time.sleep(1)  # send size of data
    ser.write(struct.pack('>h', len))

    for inf in command:  # sends data
        ser.write(struct.pack('>h', inf))

    for c in "END":
        ser.write(c.encode('utf-8'))

    print("sent:", instr, len, command)
    confirm = ser.read(1).decode('ascii')
    print(confirm)
    if(confirm == 'c'):
        print(' Confirmed !')
    else:
        print(' EPuck didn\'t listen. again. ')
    ser.close()


def listen_serial(ser: serial.Serial, time):
    i = 0
    while i < time:
        bytesToRead = ser.inWaiting()
        serstr = ser.read(bytesToRead)
        if serstr != b'':
            i = 0
            ret = serstr.decode("ascii")
            print("->", ret)
            if ret.find("DONE") != -1:
                break
        else:
            i += 1
    print("stopped listening.", i)
    ser.close()


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