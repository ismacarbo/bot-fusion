import serial
import lidarLib

ser = serial.Serial('/dev/ttyUSB0', 115200)

descriptor=ser.read(7) #read the descriptor (not needed)
while True:
    ser.write('b\xA5\x20') #equals to start scan
    raw=ser.read(5)
    point=lidarLib.decodeScanPacket(raw)
    print(point.angle, point.distance, point.quality, point.start)
