import numpy as np


def _bresenham(i0, j0, i1, j1):
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


class OccupancyGrid:
    L_FREE = np.log(0.3 / 0.7)
    L_OCC = np.log(0.9 / 0.1)

    def __init__(self, xMin, xMax, yMin, yMax, resolution, lo_free=None, lo_occ=None):
        self.resolution = float(resolution)
        self.x_min = float(xMin)
        self.y_min = float(yMin)
        self.width = int(np.ceil((xMax - xMin) / self.resolution))
        self.height = int(np.ceil((yMax - yMin) / self.resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=float)

        self.lo_free = float(self.L_FREE if lo_free is None else lo_free)
        self.lo_occ = float(self.L_OCC if lo_occ is None else lo_occ)

    def in_bounds(self, i, j):
        return 0 <= i < self.height and 0 <= j < self.width

    def world_to_map(self, x, y):
        j = int((x - self.x_min) / self.resolution)
        i = int((y - self.y_min) / self.resolution)
        return i, j

    @staticmethod
    def bresenhamLine(i0, j0, i1, j1):
        return _bresenham(i0, j0, i1, j1)

    def inverse_sensor_update(self, robotPose, scan):
        xR, yR, thetaR = robotPose
        i0, j0 = self.world_to_map(xR, yR)
        if not self.in_bounds(i0, j0):
            return

        for meas in scan:
            if len(meas) == 3:
                _, angle, dist = meas
            elif len(meas) == 2:
                angle, dist = meas
            else:
                continue

            if dist <= 0:
                continue

            # Backward compatibility: accept degrees or radians.
            if abs(angle) > (2.0 * np.pi + 1e-6):
                angle = np.deg2rad(angle)

            x_hit = xR + dist * np.cos(thetaR + angle)
            y_hit = yR + dist * np.sin(thetaR + angle)
            i1, j1 = self.world_to_map(x_hit, y_hit)

            ray = _bresenham(i0, j0, i1, j1)
            for ii, jj in ray[:-1]:
                if self.in_bounds(ii, jj):
                    self.log_odds[ii, jj] += self.lo_free

            if self.in_bounds(i1, j1):
                self.log_odds[i1, j1] += self.lo_occ

    def clampLogOdds(self, lMin=-10, lMax=10):
        np.clip(self.log_odds, lMin, lMax, out=self.log_odds)

    def getProbabilityMap(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))


class OccupancyGrid2D(OccupancyGrid):
    def to_grayscale(self):
        # Free=white, occupied=black
        p_occ = self.getProbabilityMap()
        img = np.clip((1.0 - p_occ) * 255.0, 0.0, 255.0)
        return img.astype(np.uint8)


class RollingOccupancyGrid2D(OccupancyGrid2D):
    # "Rolling" name kept for compatibility with existing code.
    # This implementation keeps a fixed world frame grid.
    def __init__(
        self,
        xMin,
        xMax,
        yMin,
        yMax,
        resolution,
        K=6,
        lo_free=-1.0,
        lo_occ=1.8,
        min_hit_range=0.12,
        max_hit_range=8.0,
        quality_min=0,
    ):
        super().__init__(xMin, xMax, yMin, yMax, resolution, lo_free=lo_free, lo_occ=lo_occ)
        self.K = int(K)
        self.min_hit_range = float(min_hit_range)
        self.max_hit_range = float(max_hit_range)
        self.quality_min = int(quality_min)

    def integrate_scan(self, robotPose, scan):
        filtered = []
        for m in scan:
            if len(m) == 3:
                q, a, d = m
            elif len(m) == 2:
                a, d = m
                q = self.quality_min
            else:
                continue

            if q < self.quality_min:
                continue
            if d < self.min_hit_range or d > self.max_hit_range:
                continue
            filtered.append((q, a, d))

        if filtered:
            self.inverse_sensor_update(robotPose, filtered)

