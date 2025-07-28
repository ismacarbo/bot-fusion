import gtsam
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from lidarLib import Lidar
from occupancyGrid import OccupancyGrid

MAX_STEPS=500

#convert scan to point cloud
def scan_to_pcd(scan):
    rs     = np.array([pt[2] for pt in scan])
    thetas = np.deg2rad([pt[1] for pt in scan])
    xyz = np.vstack((rs * np.cos(thetas),
                     rs * np.sin(thetas),
                     np.zeros_like(rs))).T
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))

#compute ICP (iterative closest point), maps the point cloud pcd_source to pcd_target (optimal transformation T)
def computeIcp(pcd_source, pcd_target, init=np.eye(4)):
    threshold = 0.5
    reg = o3d.pipelines.registration.registration_icp(
        pcd_source, pcd_target, threshold,
        init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    return reg.transformation


class SLAM2D:

    def __init__(self,sigma_odom=(0.1,0.1,0.1)):
        #intial factors
        self.graph=gtsam.NonlinearFactorGraph()
        self.initial=gtsam.Values()

        sigmas=np.array(sigma_odom)
        self.noiseOdom=gtsam.noiseModel.Diagonal.Sigmas(sigmas)

        #prior on 0 index to set the initial position
        priorNoise = gtsam.noiseModel.Diagonal.Sigmas([1e-6,1e-6,1e-8])
        self.graph.add(gtsam.PriorFactorPose2(0,
                                      gtsam.Pose2(0,0,0),
                                      priorNoise))

        self.initial.insert(0, gtsam.Pose2(0,0,0))
        self.last_pose = gtsam.Pose2(0,0,0)
        self.counter   = 0

    def addOdom(self,T4x4):
        i=self.counter+1
        dx=T4x4[0,3]
        dy=T4x4[1,3]
        theta=np.arctan2(T4x4[1,0],T4x4[0,0])

        #creating the odometry factor
        odo=gtsam.Pose2(dx,dy,theta) #measurement between i-1 and i (two nodes)
        self.graph.add(gtsam.BetweenFactorPose2(i-1,i,
                                        odo,
                                        self.noiseOdom)) #add the factor to the graph for the optimization
        #update the last pose
        estimate=self.last_pose.compose(odo)
        self.initial.insert(i,estimate)
        self.last_pose=estimate
        self.counter +=1

    def optimize(self):
        #run the optimizer
        params=gtsam.LevenbergMarquardtParams() #optimization algorithm (fitting parameters)
        optimizer=gtsam.LevenbergMarquardtOptimizer(self.graph,self.initial,params)
        result=optimizer.optimize()
        #updating the values
        self.initial=result
        self.last_pose=result.atPose2(self.counter)
        return result


def main():
    #grid parameters
    xMin, xMax, yMin, yMax, res = -100, 100, -100, 100, 0.5
    grid    = OccupancyGrid(xMin,xMax,yMin,yMax,res)
    slam    = SLAM2D(sigma_odom=(0.05,0.05,0.02))
    lidar   = Lidar('/dev/ttyUSB0')
    scans   = []

    lidar.connect()
    lidar.startScan()

    fig, ax = plt.subplots()
    scat = ax.scatter([],[],c='k',s=1)
    ax.set_aspect('equal','box')
    ax.set_xlim(xMin*100, xMax*100)
    ax.set_ylim(yMin*100, yMax*100)
    plt.ion(); plt.show()

    prev_pcd = None

    try:
        for scan in lidar.iterScan():
            scans.append(scan)
            pcd = scan_to_pcd(scan)

            if prev_pcd is None:
                T = np.eye(4)
            else:
                T = computeIcp(pcd, prev_pcd, init=np.eye(4))
            slam.addOdom(T)
            prev_pcd = pcd

            if slam.counter % 20 == 0 and slam.counter>0:
                result = slam.optimize()
                print(f"Optimize @ step {slam.counter}")

            if slam.counter >= MAX_STEPS:
                break

            posesol = slam.initial.atPose2(slam.counter)
            x0,y0,th0 = posesol.x(), posesol.y(), posesol.theta()
            coords = np.array([ (pt[2]*np.cos(np.deg2rad(pt[1]))+x0*100,
                                 pt[2]*np.sin(np.deg2rad(pt[1]))+y0*100)
                                for pt in scan ])
            scat.set_offsets(coords)
            fig.canvas.draw_idle()
            plt.pause(0.01)

    finally:
        lidar.stopScan()
        lidar.disconnect()
        plt.ioff()
        plt.close()

    
    print("Building final occupancy grid…")
    result = slam.optimize()
    for i in range(len(scans)):
        pose_i = result.atPose2(i)
        grid.inverse_sensor_update(
            (pose_i.x(), pose_i.y(), pose_i.theta()),
            scans[i]
        )
    grid.clampLogOdds()
    prob = grid.getProbabilityMap()


    plt.figure(figsize=(6,6))
    plt.title("Occupancy Grid SLAM")
    plt.imshow(prob, origin='lower',
               extent=(xMin, xMax, yMin, yMax),
               cmap='gray_r')
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.savefig("occupancy_grid.png", dpi=300, bbox_inches="tight")
    plt.show()

if __name__ == "__main__":
    main()
