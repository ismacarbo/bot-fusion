import serial

ser = serial.Serial('/dev/ttyUSB0', 115200)
while True:
    ser.write('b\xA5\x20') #equals to start scan
