import gtsam
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
from lidarLib import Lidar
from occupancyGrid import OccupancyGrid


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
