import serial
import time
import struct

def send_instr(file,port):
    ser = serial.Serial(port, 19200, timeout=0.1)
    if(not ser.is_open):
        ser.open()

    file = open(file,"r")

    command = []

    instr = file.readline().rstrip("\n")
    len = int(file.readline().rstrip("\n"))
    for i in range(len):
        for inf in file.readline().split():
            command.append(int(inf))

    print("sending:",instr,len,command)

    for c in instr :
        ser.write(c.encode('utf-8'))

    time.sleep(1)

    ser.write(struct.pack(">H",len))


    for inf in command:
        ser.write(struct.pack(">H",inf))

