import numpy as np
import matplotlib.pyplot as plt
from lidarLib import Lidar


lidar = Lidar('/dev/ttyUSB0')
lidar.connect()
print("INFO:", lidar.getInfo())
print("HEALTH:", lidar.getHealth())

lidar.startScan()

fig, ax = plt.subplots()
scatter = ax.scatter([], [], c='k', s=0.5)
ax.set_aspect('equal', 'box')
ax.set_xlim(-4000, 4000)
ax.set_ylim(-4000, 4000)
plt.show(block=False)

try:
    for scan in lidar.iterScan():
        rs = np.array([pt[2] for pt in scan])
        thetas = np.deg2rad([pt[1] for pt in scan])
        xs = rs * np.cos(thetas)
        ys = rs * np.sin(thetas)

        scatter.set_offsets(np.column_stack((xs, ys)))
        fig.canvas.draw_idle()
        plt.pause(0.02)
finally:
    lidar.stopScan()
    lidar.disconnect()
    plt.close()