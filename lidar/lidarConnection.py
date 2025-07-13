from lidarLib import Lidar
from rplidar import RPLidar

lidar = Lidar('/dev/ttyUSB0')
lidar.connect()

info = lidar.getInfo()
print("Info:", info)

status, err = lidar.getHealth()
print("Health:", status, err)

lidar.startScan()
for i, scan in enumerate(lidar.iterScan()):
    print(f"Scan {i}: {len(scan)} points")
    if i >= 5:
        break
lidar.stopScan()
lidar.disconnect()
