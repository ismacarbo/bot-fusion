import occupancyGrid as og
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


grid = og.OccupancyGrid(xMin=-10, xMax=10, yMin=-10, yMax=10, resolution=0.1)
robot_pose = [0.0, 0.0, 0.0]
step = 0.5                  
turn = np.radians(15)      


obstacles = [(x, y) for x in np.linspace(-5, 5, 5) for y in np.linspace(-5, 5, 5)]
obstacle_radius = 0.5      


def setup_plot():
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim(grid.x_min, grid.x_min + grid.width * grid.resolution)
    ax.set_ylim(grid.y_min, grid.y_min + grid.height * grid.resolution)
    return fig, ax

fig, ax = setup_plot()


def simulateScan(robotPose, numRays, maxRange):
    xR, yR, thR = robotPose
    angles = np.linspace(thR - np.pi, thR + np.pi, numRays)
    scan = []
    for th in angles:
        dists = []
        for (xO, yO) in obstacles:
            dx, dy = xO - xR, yO - yR
            proj = dx * np.cos(th) + dy * np.sin(th)
            if proj <= 0:
                continue
            perp = abs(-dx * np.sin(th) + dy * np.cos(th))
            if perp < obstacle_radius:
                entry = proj - np.sqrt(obstacle_radius**2 - perp**2)
                dists.append(entry)
        d = min(dists) if dists else maxRange
        scan.append((th, min(d, maxRange)))
    return scan



def on_key(event):
    moved = False
    x, y, th = robot_pose

    if event.key == 'up':
        robot_pose[0] += step * np.cos(th)
        robot_pose[1] += step * np.sin(th)
        moved = True
    elif event.key == 'down':
        robot_pose[0] -= step * np.cos(th)
        robot_pose[1] -= step * np.sin(th)
        moved = True
    elif event.key == 'left':
        robot_pose[2] += turn
        moved = True
    elif event.key == 'right':
        robot_pose[2] -= turn
        moved = True
    else:
        return
    if not moved:
        return

    scan = simulateScan(robot_pose, numRays=36, maxRange=5.0)
    grid.inverse_sensor_update(tuple(robot_pose), scan)
    grid.clampLogOdds()

    
    prob_map = grid.getProbabilityMap()
    ax.clear()
    extent = [grid.x_min,
              grid.x_min + grid.width * grid.resolution,
              grid.y_min,
              grid.y_min + grid.height * grid.resolution]
    ax.imshow(prob_map, cmap='gray', origin='lower', extent=extent)
    ax.scatter(robot_pose[0], robot_pose[1], c='red', s=20)
    ax.set_title(f"Pose: x={robot_pose[0]:.1f}, y={robot_pose[1]:.1f}, θ={np.degrees(robot_pose[2])%360:.0f}°")
    ax.set_aspect('equal')
    fig.canvas.draw_idle()


def main():
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show(block=True)

if __name__ == '__main__':
    main()