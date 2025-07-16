import numpy as np

class OccupancyGrid:

    L_FREE = np.log(0.3/0.7)
    L_OCC  = np.log(0.9/0.1)

    def __init__(self,xMin,xMax,yMin,yMax,resolution):
        self.resolution=resolution
        self.width=int(np.ceil((xMax-yMin)/resolution))
        self.height=int(np.ceil((yMax-yMin)/resolution))
        self.xMin,self.yMin=xMin,yMin
        self.log_odds=np.zeros((self.height,self.width),dtype=float)
        pass

    def world_to_map(self, x, y):
        j = int((x - self.x_min) / self.resolution)
        i = int((y - self.y_min) / self.resolution)
        return i, j

    def map_to_world(self, i, j):
        x = self.x_min + (j + 0.5) * self.resolution
        y = self.y_min + (i + 0.5) * self.resolution
        return x, y
    
    def bresenhamLine(i0, j0, i1, j1):
        cells = []
        di = abs(i1 - i0)
        dj = abs(j1 - j0)
        si = 1 if i0 < i1 else -1
        sj = 1 if j0 < j1 else -1
        err = di - dj
        i, j = i0, j0

        while True:
            cells.append((i,j))
            if i == i1 and j == j1:
                break
            e2 = 2*err
            if e2 > -dj:
                err -= dj
                i += si
            if e2 < di:
                err += di
                j += sj
        return cells
    
    def inverse_sensor_update(self,robotPose,scan):
        xR,yR,thetaR=robotPose
        i0,j0=self.world_to_map(xR,yR)
        
        for angle,dist in scan:
            #impact point in world cordinates
            xHit,yHit=xR+dist*np.cos(thetaR+angle),yR+dist*np.sin(thetaR+angle)
            i1,j1=self.world_to_map(xHit,yHit)

            #freeing the cells till the obstacle
            cells=self.BresenhamLine(i0,j0,i1,j1)
            for i,j in cells:
                self.log_odds[i,j]+=self.L_FREE

            #impact cell
            if 0<=i1<self.height and 0<=j1<self.width:
                self.log_odds[i1,j1]+=self.L_OCC
