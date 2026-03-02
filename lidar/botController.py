
import argparse, math, threading, time
import pygame
import numpy as np
import serial

from lidarLib import Lidar, LidarError
from occupancyGrid import OccupancyGrid2D, RollingOccupancyGrid2D

def parse_args():
    ap = argparse.ArgumentParser(description="Realtime global map + last-scan overlay")
    ap.add_argument("--serial", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--lidar", default="/dev/ttyUSB0")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=1100)
    ap.add_argument("--height", type=int, default=900)

    
    ap.add_argument("--kv", type=float, default=0.0035, help="m/s per 1 PWM")
    ap.add_argument("--deadzone", type=int, default=0, help="pwm sotto cui v≈0")

    
    ap.add_argument("--map-min", nargs=2, type=float, default=[-10.0, -10.0])
    ap.add_argument("--map-max", nargs=2, type=float, default=[+10.0, +10.0])
    ap.add_argument("--res", type=float, default=0.05, help="cell size (m)")

    
    ap.add_argument("--view-radius", type=float, default=6.0)
    return ap.parse_args()


class Shared:
    def __init__(self, args):
        self.lock = threading.Lock()
        
        self.t = {
            "ms": 0, "distC": -1, "distL": -1, "distR": -1,
            "irL": 0, "irR": 0, "servo": 0,
            "pwmL": 0, "pwmR": 0, "state": 0,
            "yaw_deg": 0.0, "yaw_rate_dps": 0.0
        }
        self.last_ms = None

        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  

        
        self.last_scan_world = []  
        self.lidar_ok = False

        
        xmin, ymin = args.map_min
        xmax, ymax = args.map_max
        self.grid = RollingOccupancyGrid2D(
            xmin, xmax, ymin, ymax, args.res,
            K=6, lo_free=-1.0, lo_occ=+1.8,
            min_hit_range=0.12, max_hit_range=args.view_radius, quality_min=0
        )

        
        self.kv = args.kv
        self.deadzone = args.deadzone

    def snapshot_pose(self):
        with self.lock:
            return self.x, self.y, self.theta


def arduino_thread(port, baud, shared: Shared):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2.0)
        print(f"[SER] Connected to {port} @ {baud}")
    except serial.SerialException as e:
        print(f"[SER] ERROR opening {port}: {e}")
        return

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line or not line.startswith("STAT,"):
                continue
            parts = line.split(",")
            if len(parts) < 13:
                continue

            try:
                vals = {
                    "ms": int(parts[1]),
                    "distC": int(parts[2]),
                    "distL": int(parts[3]),
                    "distR": int(parts[4]),
                    "irL": int(parts[5]),
                    "irR": int(parts[6]),
                    "servo": int(parts[7]),
                    "pwmL": int(parts[8]),
                    "pwmR": int(parts[9]),
                    "state": int(parts[10]),
                    "yaw_deg": float(parts[11]),
                    "yaw_rate_dps": float(parts[12]),
                }
            except ValueError:
                continue

            with shared.lock:
                
                prev_ms = shared.t["ms"]
                shared.t.update(vals)

                
                dt = 0.0
                if shared.last_ms is None:
                    shared.last_ms = vals["ms"]
                else:
                    dt = max(0.0, (vals["ms"] - shared.last_ms) / 1000.0)
                    shared.last_ms = vals["ms"]

                
                shared.theta = math.radians(vals["yaw_deg"])

                
                pwmL = vals["pwmL"]
                pwmR = vals["pwmR"]
                if abs(pwmL) < shared.deadzone: pwmL = 0
                if abs(pwmR) < shared.deadzone: pwmR = 0
                vL = shared.kv * pwmL
                vR = shared.kv * pwmR
                v = 0.5 * (vL + vR)

                
                if dt > 0.0:
                    shared.x += v * dt * math.cos(shared.theta)
                    shared.y += v * dt * math.sin(shared.theta)

    except Exception as e:
        print(f"[SER] thread stopped: {e}")
    finally:
        try: ser.close()
        except: pass


def lidar_thread(dev, shared: Shared):
    try:
        ld = Lidar(dev)
        ld.connect()
        print("[LIDAR] INFO:", ld.getInfo())
        print("[LIDAR] HEALTH:", ld.getHealth())
        ld.startScan()

        for scan in ld.iterScan():
            
            with shared.lock:
                xR, yR, th = shared.x, shared.y, shared.theta

            
            world_pts = []
            
            grid_scan = []
            for q, ang_deg, dist_mm in scan:
                if dist_mm <= 0:
                    continue
                ang = math.radians(ang_deg)
                dist_m = dist_mm / 1000.0
                
                xr = dist_m * math.cos(ang)
                yr = dist_m * math.sin(ang)
                
                c, s = math.cos(th), math.sin(th)
                X = xR + c * xr - s * yr
                Y = yR + s * xr + c * yr
                world_pts.append((X, Y))
                
                grid_scan.append((0, ang, dist_m))

            
            with shared.lock:
                shared.last_scan_world = world_pts
                shared.lidar_ok = True
                
                shared.grid.integrate_scan((xR, yR, th), grid_scan)

    except LidarError as e:
        print("[LIDAR] error:", e)
    except Exception as e:
        print("[LIDAR] thread stopped:", e)


def draw_texts(screen, font, tl, t, fps):
    lines = [
        f"FPS: {fps:4.1f}  LiDAR: OK",
        f"dist C/L/R: {t['distC']}/{t['distL']}/{t['distR']} cm",
        f"IR L/R: {t['irL']}/{t['irR']}  SERVO: {t['servo']} deg",
        f"PWM L/R: {t['pwmL']}/{t['pwmR']}  state: {t['state']}",
        f"Yaw: {t['yaw_deg']:.1f}°  YawRate: {t['yaw_rate_dps']:.1f} dps",
    ]
    x, y = tl
    for s in lines:
        surf = font.render(s, True, (30,30,30))
        screen.blit(surf, (x, y))
        y += surf.get_height() + 2

def draw_world_map(screen, rect, shared: Shared, view_radius_m, res):
    x0, y0, w, h = rect
    sub = pygame.Surface((w,h))
    sub.fill((235,235,235))

    with shared.lock:
        
        img = shared.grid.to_grayscale()  
        xmin, ymin = shared.grid.x_min, shared.grid.y_min
        xmax = shared.grid.x_min + shared.grid.width * res
        ymax = shared.grid.y_min + shared.grid.height * res
        xR, yR, th = shared.x, shared.y, shared.theta
        pts = list(shared.last_scan_world)


    
    vxmin = xR - view_radius_m
    vxmax = xR + view_radius_m
    vymin = yR - view_radius_m
    vymax = yR + view_radius_m

    
    vxmin = max(vxmin, xmin); vxmax = min(vxmax, xmax)
    vymin = max(vymin, ymin); vymax = min(vymax, ymax)

    
    i_min = int((vymin - ymin) / res)
    i_max = int((vymax - ymin) / res)
    j_min = int((vxmin - xmin) / res)
    j_max = int((vxmax - xmin) / res)

    i_min = max(0, min(img.shape[0]-1, i_min))
    i_max = max(0, min(img.shape[0],   i_max))
    j_min = max(0, min(img.shape[1]-1, j_min))
    j_max = max(0, min(img.shape[1],   j_max))

    if i_max <= i_min or j_max <= j_min:
        screen.blit(sub, (x0,y0)); return

    crop = img[i_min:i_max, j_min:j_max]
    
    crop_rgb = np.stack([np.rot90(crop)]*3, axis=-1)
    surf = pygame.surfarray.make_surface(crop_rgb)
    surf = pygame.transform.smoothscale(surf, (w,h))
    sub.blit(surf, (0,0))

    
    cx = int(w * (xR - vxmin) / (vxmax - vxmin))
    cy = int(h * (vymax - yR) / (vymax - vymin))
    pygame.draw.circle(sub, (252,132,3), (cx,cy), 6)
    xh = cx + int(20 * math.cos(th))
    yh = cy - int(20 * math.sin(th))
    pygame.draw.line(sub, (0,0,255), (cx,cy), (xh,yh), 2)

    
    for (X,Y) in pts[::3]:  
        px = int(w * (X - vxmin) / (vxmax - vxmin))
        py = int(h * (vymax - Y) / (vymax - vymin))
        if 0 <= px < w and 0 <= py < h:
            sub.set_at((px,py), (0,0,0))

    pygame.draw.rect(sub, (80,80,80), sub.get_rect(), 1)
    screen.blit(sub, (x0,y0))

def main_loop(args, shared: Shared):
    pygame.init()
    screen = pygame.display.set_mode((args.width, args.height))
    pygame.display.set_caption("Realtime Bot Dashboard (global map)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(pygame.font.get_default_font(), 18)

    
    map_rect = (320, 20, args.width-340, args.height-40)  

    running = True
    while running:
        dt = clock.tick(args.fps) / 1000.0
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT: running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE): running = False
                elif ev.key == pygame.K_s:
                    pygame.image.save(screen, "dashboard_global.png")
                    print("[UI] screenshot salvato")

        screen.fill((250,250,250))

        with shared.lock:
            t = dict(shared.t)

        draw_texts(screen, font, (20,20), t, clock.get_fps())
        draw_world_map(screen, map_rect, shared, args.view_radius, args.res)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    args = parse_args()
    shared = Shared(args)

    th_ser = threading.Thread(target=arduino_thread, args=(args.serial, args.baud, shared), daemon=True)
    th_ld  = threading.Thread(target=lidar_thread,   args=(args.lidar, shared), daemon=True)
    th_ser.start()
    th_ld.start()

    main_loop(args, shared)
