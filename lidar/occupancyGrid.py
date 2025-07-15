import numpy as np

class OccupancyGrid:
    def __init__(self,xMin,xMax,yMin,yMax,resolution):
        self.resolution=resolution
        self.width=int(np.ceil((xMax-yMin)/resolution))
        self.height=int(np.ceil((yMax-yMin)/resolution))
        self.xMin,self.yMin=xMin,yMin
        self.log_odds=np.zeros((self.height,self.width),dtype=float)
        pass