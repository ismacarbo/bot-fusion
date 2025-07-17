import occupancyGrid as og
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


LOG_FILE        = "orebro.raw.log"   
MAX_RANGE       = 10.0              
N_BEAMS         = 181               
ANGLES          = np.deg2rad(np.linspace(-90, 90, N_BEAMS))
ROBOT_POSE      = (0.0, 0.0, 0.0)    
GRID_RESOLUTION = 0.05              
GRID_SIZE       = 20.0              


grid = og.OccupancyGrid(
    xMin=-GRID_SIZE, xMax=GRID_SIZE,
    yMin=-GRID_SIZE, yMax=GRID_SIZE,
    resolution=GRID_RESOLUTION
)


scans = []
with open(LOG_FILE, "r") as f:
    for line in f:
        parts = line.strip().split()
        if not parts or parts[0] != "FLASER":
            continue
        raw = parts[2:2+N_BEAMS]
        
        ranges = np.array([float(r) if r.replace('.','',1).isdigit() else np.nan
                           for r in raw])
        
        ranges = np.where((ranges>0)&np.isfinite(ranges), ranges, MAX_RANGE)
        scans.append(ranges)




fig, ax = plt.subplots(figsize=(6,6))
ax.set_aspect('equal')
extent = [
    grid.x_min,
    grid.x_min + grid.width * grid.resolution,
    grid.y_min,
    grid.y_min + grid.height * grid.resolution
]

im = ax.imshow(
    grid.getProbabilityMap().T,
    cmap='gray', origin='lower',
    extent=extent, vmin=0, vmax=1
)

scatter = ax.scatter([], [], c='red', s=5)
title = ax.set_title("Frame 0")




def animate(frame):
    
    ranges = scans[frame]                
    scan = list(zip(ANGLES, ranges))     

    
    grid.inverse_sensor_update(ROBOT_POSE, scan)
    grid.clampLogOdds()

    
    prob = grid.getProbabilityMap().T
    im.set_data(prob)

    
    xR, yR, thR = ROBOT_POSE
    
    
    thetas = ANGLES + thR
    xs = xR + ranges * np.cos(thetas)
    ys = yR + ranges * np.sin(thetas)
    scatter.set_offsets(np.column_stack([xs, ys]))

    
    title.set_text(f"Frame {frame+1}/{len(scans)}")

    return im, scatter, title




ani = FuncAnimation(
    fig, animate,
    frames=len(scans),
    interval=100,           
    blit=True, 
    cache_frame_data=False
)

plt.show()
