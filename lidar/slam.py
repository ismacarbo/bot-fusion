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
