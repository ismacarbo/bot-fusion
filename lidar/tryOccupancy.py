import occupancyGrid as og
import numpy as np
import matplotlib.pyplot as plt


grid = og.OccupancyGrid(xMin=-10, xMax=10, yMin=-10, yMax=10, resolution=0.1)

robot_pose = [0.0, 0.0, 0.0]  
robot_pose_prev = robot_pose.copy()

step = 0.5
turn = np.radians(15)  


plt.ion()
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(grid.x_min, grid.x_min + grid.width * grid.resolution)
ax.set_ylim(grid.y_min, grid.y_min + grid.height * grid.resolution)


def on_key(event):
    global robot_pose, robot_pose_prev

    moved = False
    
    if event.key == 'w':
        robot_pose[0] += step * np.cos(robot_pose[2])
        robot_pose[1] += step * np.sin(robot_pose[2])
        moved = True
    elif event.key == 's':
        robot_pose[0] -= step * np.cos(robot_pose[2])
        robot_pose[1] -= step * np.sin(robot_pose[2])
        moved = True
    
    elif event.key == 'a':
        robot_pose[2] += turn
        moved = True
    elif event.key == 'd':
        robot_pose[2] -= turn
        moved = True
    else:
        return  

    if not moved:
        return

    robot_pose_prev = robot_pose.copy()
    angle = robot_pose[2]
    dist  = 12.0
    scan = [(angle, dist)]

    grid.inverse_sensor_update(tuple(robot_pose), scan)
    grid.clampLogOdds()

    
    prob_map = grid.getProbabilityMap()
    ax.clear()
    extent = [
        grid.x_min,
        grid.x_min + grid.width * grid.resolution,
        grid.y_min,
        grid.y_min + grid.height * grid.resolution,
    ]
    ax.imshow(
        prob_map,
        cmap='gray',
        origin='lower',
        extent=extent
    )
    
    ax.scatter(robot_pose[0], robot_pose[1], c='red', s=20)
    ax.set_title("Use WASD to move")
    ax.set_aspect('equal')
    fig.canvas.draw_idle()


def main():
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show(block=True)

if __name__ == '__main__':
    main()