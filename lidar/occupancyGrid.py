
import numpy as np

class OccupancyGrid:
    L_FREE = np.log(0.3 / 0.7)
    L_OCC  = np.log(0.9 / 0.1)

    def __init__(self, xMin, xMax, yMin, yMax, resolution):
        self.resolution = resolution
        self.x_min, self.y_min = xMin, yMin
        self.width  = int(np.ceil((xMax - xMin) / resolution))
        self.height = int(np.ceil((yMax - yMin) / resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=float)

    def world_to_map(self, x, y):
        j = int((x - self.x_min) / self.resolution)
        i = int((y - self.y_min) / self.resolution)
        return i, j

    @staticmethod
    def bresenhamLine(i0, j0, i1, j1):
        cells = []
        di = abs(i1 - i0)
        dj = abs(j1 - j0)
        si = 1 if i0 < i1 else -1
        sj = 1 if j0 < j1 else -1
        err = di - dj
        i, j = i0, j0

        while True:
            cells.append((i, j))
            if i == i1 and j == j1:
                break
            e2 = 2 * err
            if e2 > -dj:
                err -= dj
                i += si
            if e2 < di:
                err += di
                j += sj
        return cells

    def inverse_sensor_update(self, robotPose, scan):
        xR, yR, thetaR = robotPose
        i0, j0 = self.world_to_map(xR, yR)

        for meas in scan:
            if len(meas) == 3:
                _, angle, dist = meas
            elif len(meas) == 2:
                angle, dist = meas
            else:
                continue

            if dist <= 0:
                continue

            # Compatibility with both radians and degrees input.
            if abs(angle) > (2 * np.pi + 1e-6):
                angle = np.deg2rad(angle)

            # impact point in world coordinates
            xHit = xR + dist * np.cos(thetaR + angle)
            yHit = yR + dist * np.sin(thetaR + angle)
            i1, j1 = self.world_to_map(xHit, yHit)

            ray = _bresenham(i0, j0, i1, j1)

            
            for (ii, jj) in ray[:-1]:
                if self.in_bounds(ii, jj):
                    layer[ii, jj] += self.lo_free

            # occupied cell
            if 0 <= i1 < self.height and 0 <= j1 < self.width:
                self.log_odds[i1, j1] += self.L_OCC

    def clampLogOdds(self, lMin=-10, lMax=10):
        np.clip(self.log_odds, lMin, lMax, out=self.log_odds)

    def getProbabilityMap(self):
        return 1 - 1 / (1 + np.exp(self.log_odds))
