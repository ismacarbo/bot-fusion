import occupancyGrid as og
import numpy as np
import matplotlib.pyplot as plt

grid=og.OccupancyGrid(-10,10,-10,10,0.1)

robotPose=(0,0,0)

for _ in range(10):
    angles=np.linspace(-np.pi/2,np.pi/2,100)
    dists=np.random.uniform(1.0,8.0,size=angles.shape)
    scan=list(zip(angles,dists))
    grid.inverse_sensor_update(robotPose,scan)

grid.clampLogOdds()
probMap=grid.getProbabilityMap()

plt.figure(figsize=(10,10))
plt.imshow(probMap,origin='lower')
plt.title("Randomized occupancy grid")
plt.xlabel("col")
plt.ylabel("row")
plt.show()