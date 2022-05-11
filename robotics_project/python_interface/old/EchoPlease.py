
import serial
import keyboard
import struct

port = "COM6"

ser = serial.Serial(port, 57600, timeout=5)

if(not ser.is_open):
    ser.open()

ser.write("MOVE".encode('utf-8'))

ser.write(struct.pack(">H",15))

for instr in [500,500,2000,400,50,400,50,400,400,500,500,200,500,500,200]:
    ser.write(struct.pack(">H",instr))

i = 0
while i < 1000000:
    
    bytesToRead = ser.inWaiting()
    serstr = ser.read(bytesToRead)
    if serstr != b'':
        i= 0
        print("->",serstr.decode("ascii"))
    else:
        i += 1
print("stopped listening.",i)
ser.close()