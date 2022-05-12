import communication as comm

r, g, b = comm.rgb565_to_rgb888(0b0000_0000_0000_0000)
print(r, g, b)
r, g, b = comm.rgb565_to_rgb888(0b1111_1000_0000_0000)
print(r, g, b)
r, g, b = comm.rgb565_to_rgb888(0b0000_0111_1110_0000)
print(r, g, b)
r, g, b = comm.rgb565_to_rgb888(0b0000_0000_0001_1111)
print(r, g, b)
r, g, b = comm.rgb565_to_rgb888(0b1001_1001_1001_1011)
print(r, g, b)
r, g, b = comm.rgb565_to_rgb888(0b1111_1111_1111_1111)
print(r, g, b)