
import numpy as np
import math

__all__ = [
    "OccupancyGrid2D",
    "RollingOccupancyGrid2D",
]



def _bresenham(i0, j0, i1, j1):
    """Ritorna le celle dal punto (i0,j0) a (i1,j1) inclusi (integer grid)."""
    di = abs(i1 - i0)
    dj = abs(j1 - j0)
    si = 1 if i0 < i1 else -1
    sj = 1 if j0 < j1 else -1
    err = di - dj
    i, j = i0, j0
    out = []
    while True:
        out.append((i, j))
        if i == i1 and j == j1:
            break
        e2 = 2 * err
        if e2 > -dj:
            err -= dj
            i += si
        if e2 < di:
            err += di
            j += sj
    return out


def _sigmoid(x):
    
    return 1.0 - 1.0 / (1.0 + np.exp(x))




class OccupancyGrid2D:
    """
    Log-odds occupancy grid 2D con:
    - coordinate in METRI
    - update per raggio (celle libere + cella d'impatto occupata)
    - clamp e decadimento opzionale per 'ripulire' la storia
    """
    def __init__(self, x_min, x_max, y_min, y_max, resolution,
                 lo_free=-0.85, lo_occ=+1.2, lo_min=-8.0, lo_max=+8.0,
                 decay=1.0, min_hit_range=0.12, max_hit_range=None,
                 quality_min=0):
        """
        decay: 1.0 = nessun decadimento; <1.0 = svanisce nel tempo (e.g. 0.98)
        min_hit_range: ignora ritorni troppo vicini (m)
        max_hit_range: tronca i raggi oltre questa distanza (m) se non None
        quality_min: ignorare misure con quality < quality_min
        """
        self.x_min = float(x_min)
        self.x_max = float(x_max)
        self.y_min = float(y_min)
        self.y_max = float(y_max)
        self.res = float(resolution)

        self.height = int(np.ceil((self.y_max - self.y_min) / self.res))
        self.width  = int(np.ceil((self.x_max - self.x_min) / self.res))

        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

        self.lo_free = float(lo_free)
        self.lo_occ  = float(lo_occ)
        self.lo_min  = float(lo_min)
        self.lo_max  = float(lo_max)
        self.decay   = float(decay)

        self.min_hit_range = float(min_hit_range)
        self.max_hit_range = None if max_hit_range is None else float(max_hit_range)
        self.quality_min = int(quality_min)

    
    def world_to_map(self, x, y):
        j = int((x - self.x_min) / self.res)
        i = int((y - self.y_min) / self.res)
        return i, j

    def map_to_world(self, i, j):
        x = self.x_min + (j + 0.5) * self.res
        y = self.y_min + (i + 0.5) * self.res
        return x, y

    def in_bounds(self, i, j):
        return (0 <= i < self.height) and (0 <= j < self.width)

    
    def clear(self):
        self.log_odds.fill(0.0)

    def clamp(self):
        np.clip(self.log_odds, self.lo_min, self.lo_max, out=self.log_odds)

    def apply_decay(self):
        """Applica decadimento globale (se decay < 1)."""
        if self.decay < 1.0:
            self.log_odds *= self.decay

    
    def integrate_scan(self, pose, scan):
        """
        Aggiorna la mappa con un'intera SCANSIONE (un giro Lidar).

        pose: (xR, yR, thetaR) in metri e radianti (world frame).
        scan: lista di tuple (quality, angle_rad, dist_m) nel frame del robot.
              (Se quality non serve, metti 0.)
        """
        xR, yR, th = pose
        i0, j0 = self.world_to_map(xR, yR)
        if not self.in_bounds(i0, j0):
            
            return

        
        self.apply_decay()

        c, s = math.cos(th), math.sin(th)

        for q, ang, d in scan:
            if q < self.quality_min:
                continue
            if d <= self.min_hit_range:
                continue
            if self.max_hit_range is not None and d > self.max_hit_range:
                d = self.max_hit_range

            
            xr = d * math.cos(ang)
            yr = d * math.sin(ang)
            
            X = xR + c * xr - s * yr
            Y = yR + s * xr + c * yr
            i1, j1 = self.world_to_map(X, Y)

            
            ray = _bresenham(i0, j0, i1, j1)

            
            
            for (ii, jj) in ray[:-1]:
                if self.in_bounds(ii, jj):
                    self.log_odds[ii, jj] += self.lo_free

            
            if self.in_bounds(i1, j1):
                self.log_odds[i1, j1] += self.lo_occ

        self.clamp()

    
    def probability(self):
        return _sigmoid(self.log_odds)

    def to_grayscale(self):
        """
        Restituisce immagine uint8: bianco=LIBERO, nero=OCCUPATO, grigio=ignoto.
        """
        p = self.probability()
        img = (255.0 * (1.0 - p)).clip(0, 255).astype(np.uint8)
        return img



class RollingOccupancyGrid2D:
    """
    Mantiene SOLO gli ultimi K giri LiDAR.
    Implementazione: somma di K 'layers' di log-odds (buffer circolare).
    Ogni nuova scansione:
      - rimuove il layer K-vecchio dalla somma,
      - lo ricalcola con i raggi del giro corrente,
      - lo aggiunge alla somma.
    """
    def __init__(self, x_min, x_max, y_min, y_max, resolution,
                 K=6, lo_free=-1.0, lo_occ=+1.8, lo_min=-8.0, lo_max=+8.0,
                 min_hit_range=0.12, max_hit_range=None, quality_min=0):
        self.x_min = float(x_min)
        self.x_max = float(x_max)
        self.y_min = float(y_min)
        self.y_max = float(y_max)
        self.res = float(resolution)

        self.height = int(np.ceil((self.y_max - self.y_min) / self.res))
        self.width  = int(np.ceil((self.x_max - self.x_min) / self.res))

        self.accum = np.zeros((self.height, self.width), dtype=np.float32)
        self.layers = np.zeros((K, self.height, self.width), dtype=np.float32)
        self.K = int(K)
        self.idx = 0  

        self.lo_free = float(lo_free)
        self.lo_occ  = float(lo_occ)
        self.lo_min  = float(lo_min)
        self.lo_max  = float(lo_max)

        self.min_hit_range = float(min_hit_range)
        self.max_hit_range = None if max_hit_range is None else float(max_hit_range)
        self.quality_min = int(quality_min)

    
    def world_to_map(self, x, y):
        j = int((x - self.x_min) / self.res)
        i = int((y - self.y_min) / self.res)
        return i, j

    def in_bounds(self, i, j):
        return (0 <= i < self.height) and (0 <= j < self.width)

    def clamp(self):
        np.clip(self.accum, self.lo_min, self.lo_max, out=self.accum)

    def clear_all(self):
        self.accum.fill(0.0)
        self.layers.fill(0.0)
        self.idx = 0

    
    def integrate_scan(self, pose, scan):
        """
        Integra UN giro LiDAR come nuovo layer (sostituendo il più vecchio).
        pose = (xR, yR, thetaR)
        scan = [(quality, angle_rad, dist_m), ...]
        """
        xR, yR, th = pose
        i0, j0 = self.world_to_map(xR, yR)
        if not self.in_bounds(i0, j0):
            return

        
        self.accum -= self.layers[self.idx]
        self.layers[self.idx].fill(0.0)
        layer = self.layers[self.idx]

        c, s = math.cos(th), math.sin(th)

        for q, ang, d in scan:
            if q < self.quality_min:
                continue
            if d <= self.min_hit_range:
                continue
            if self.max_hit_range is not None and d > self.max_hit_range:
                d = self.max_hit_range

            xr = d * math.cos(ang)
            yr = d * math.sin(ang)
            X = xR + c * xr - s * yr
            Y = yR + s * xr + c * yr
            i1, j1 = self.world_to_map(X, Y)

            ray = _bresenham(i0, j0, i1, j1)

            
            for (ii, jj) in ray[:-1]:
                if self.in_bounds(ii, jj):
                    layer[ii, jj] += self.lo_free

            
            if self.in_bounds(i1, j1):
                layer[i1, j1] += self.lo_occ

        
        self.accum += layer
        self.clamp()
        self.idx = (self.idx + 1) % self.K

    
    def probability(self):
        return _sigmoid(self.accum)

    def to_grayscale(self):
        p = self.probability()
        img = (255.0 * (1.0 - p)).clip(0, 255).astype(np.uint8)
        return img
