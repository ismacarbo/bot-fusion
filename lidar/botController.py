






import argparse, math, time, sys
import numpy as np
from collections import deque
import serial

from lidarLib import Lidar               
from occupancyGrid import OccupancyGrid  

def setup_matplotlib(headless: bool):
    import matplotlib
    if headless:
        matplotlib.use('Agg')
        backend = 'Agg'
    else:
        
        try:
            matplotlib.use('TkAgg')
            backend = 'TkAgg'
        except Exception:
            try:
                matplotlib.use('Qt5Agg')
                backend = 'Qt5Agg'
            except Exception:
                matplotlib.use('Agg')
                backend = 'Agg'
                headless = True
    import matplotlib.pyplot as _plt
    return _plt, backend, headless

class BotController:
    def __init__(self, plt, headless, out_map="map_latest.png", out_tel="telemetry_latest.png",
                 port_serial="/dev/ttyACM0", port_lidar="/dev/ttyUSB0",
                 baud=115200,
                 kv=0.0035, deadzone=20,
                 wheel_base=0.15,
                 grid=( -6.0, 6.0, -6.0, 6.0 ),
                 res=0.05,
                 imu_direct=True,
                 clamp_dist_m=8.0,
                 update_ms=50,
                 history_secs=30,
                 save_every_s=0.5):
        self.plt = plt
        self.headless = headless
        self.out_map = out_map
        self.out_tel = out_tel

        self.port_serial = port_serial
        self.port_lidar  = port_lidar
        self.baud        = baud

        self.KV = float(kv)
        self.DZ = int(deadzone)
        self.WHEEL_BASE = float(wheel_base)

        xMin, xMax, yMin, yMax = grid
        self.grid = OccupancyGrid(xMin, xMax, yMin, yMax, float(res))
        self.grid_last_update = 0.0
        self.grid_update_dt   = 0.10

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.imu_direct = bool(imu_direct)
        self.FUSE_ALPHA = 0.9

        self.clamp_dist_m = float(clamp_dist_m)
        self.update_ms = int(update_ms)

        self.ser = serial.Serial(self.port_serial, self.baud, timeout=0.0)
        time.sleep(2.0)

        self.lidar = Lidar(self.port_lidar)
        self.lidar.connect()
        print("[LIDAR] INFO:", self.lidar.getInfo())
        print("[LIDAR] HEALTH:", self.lidar.getHealth())
        self.lidar.startScan()

        self.last_ser = None
        self.t0 = time.time()
        self.last_time = self.t0

        
        self.fig_map, self.ax_map = self.plt.subplots(1, 1, figsize=(6.5,6))
        self.im = self.ax_map.imshow(
            np.zeros_like(self.grid.log_odds),
            origin='lower',
            extent=(xMin, xMax, yMin, yMax),
            cmap='gray_r',
            vmin=0.0, vmax=1.0
        )
        self.scan_plot = self.ax_map.plot([], [], '.', markersize=1)[0]
        self.pose_plot = self.ax_map.plot([], [], 'ro', markersize=5)[0]
        self.ax_map.set_title("Occupancy Grid + LiDAR")
        self.ax_map.set_aspect('equal', 'box')
        self.ax_map.set_xlim(xMin, xMax)
        self.ax_map.set_ylim(yMin, yMax)
        self.fig_map.tight_layout()

        
        self.fig_plot, (self.ax_yaw, self.ax_pwm, self.ax_dist) = self.plt.subplots(3, 1, figsize=(8,7), sharex=True)
        self.ax_yaw.set_ylabel("Yaw (deg) / YawRate (dps)")
        self.ax_pwm.set_ylabel("PWM L/R")
        self.ax_dist.set_ylabel("Dist (cm)")
        self.ax_dist.set_xlabel("Time (s)")

        N = max(200, int(history_secs * (1000.0/max(1,update_ms))))
        self.ts = deque(maxlen=N)
        self.yaw_deg_hist = deque(maxlen=N)
        self.yaw_rate_hist = deque(maxlen=N)
        self.pwmL_hist = deque(maxlen=N)
        self.pwmR_hist = deque(maxlen=N)
        self.dC_hist = deque(maxlen=N)
        self.dL_hist = deque(maxlen=N)
        self.dR_hist = deque(maxlen=N)

        (self.l_yaw,) = self.ax_yaw.plot([], [], label="yaw_deg")
        (self.l_wz,)  = self.ax_yaw.plot([], [], label="yaw_rate_dps")
        self.ax_yaw.legend(loc="upper right")

        (self.l_pwmL,) = self.ax_pwm.plot([], [], label="pwmL")
        (self.l_pwmR,) = self.ax_pwm.plot([], [], label="pwmR")
        self.ax_pwm.legend(loc="upper right")

        (self.l_dC,) = self.ax_dist.plot([], [], label="distC")
        (self.l_dL,) = self.ax_dist.plot([], [], label="distL")
        (self.l_dR,) = self.ax_dist.plot([], [], label="distR")
        self.ax_dist.legend(loc="upper right")

        self.fig_plot.tight_layout()

        if not self.headless:
            self.plt.ion()
            self.fig_map.show()
            self.fig_plot.show()

        self.last_save = 0.0
        self.save_every_s = float(save_every_s)
        print(f"[INFO] headless={self.headless}  (map->{self.out_map}, telemetry->{self.out_tel})")

    def close(self):
        try:
            self.lidar.stopScan()
            self.lidar.disconnect()
        except Exception:
            pass
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    
    def _parse_serial_line(self, line):
        parts = line.strip().split(',')
        if len(parts) < 13 or parts[0] != 'STAT':
            return None
        try:
            return {
                't_ms':   int(parts[1]),
                'distC':  int(parts[2]),
                'distL':  int(parts[3]),
                'distR':  int(parts[4]),
                'pwmL':   int(parts[7]),  
                'pwmR':   int(parts[8]),
                'yaw_deg':       float(parts[11]),
                'yaw_rate_dps':  float(parts[12]),
            }
        except Exception:
            return None

    def read_serial_nonblocking(self):
        out = None
        while True:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore')
            except Exception:
                break
            if not line:
                break
            if line.startswith('STAT'):
                parsed = self._parse_serial_line(line)
                if parsed:
                    out = parsed
        return out

    
    def _pwm_to_v(self, pwm):
        s = 1.0 if pwm >= 0 else -1.0
        ap = abs(pwm)
        if ap <= self.DZ:
            return 0.0
        return s * self.KV * (ap - self.DZ)

    def step_pose(self, dt, sd):
        pwmL = sd['pwmL']; pwmR = sd['pwmR']
        vL = self._pwm_to_v(pwmL)
        vR = self._pwm_to_v(pwmR)
        v  = 0.5*(vL+vR)
        w_model = (vR - vL)/self.WHEEL_BASE if self.WHEEL_BASE > 1e-6 else 0.0

        if True:  
            self.th = math.radians(sd['yaw_deg'])
        else:
            th_pred = self.th + w_model*dt
            th_imu  = math.radians(sd['yaw_deg'])
            self.th = 0.9 * th_pred + 0.1 * th_imu

        self.th = (self.th + math.pi) % (2*math.pi) - math.pi
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt

    
    def process_lidar_scan(self, scan):
        scan_m = []
        for (q, ang_deg, dist_mm) in scan:
            d_m = (dist_mm / 1000.0)
            if d_m <= 0 or d_m > self.clamp_dist_m:
                continue
            ang = math.radians(ang_deg)
            scan_m.append( (q, ang, d_m) )

        if not scan_m:
            return None

        self.grid.inverse_sensor_update((self.x, self.y, self.th), scan_m)
        return scan_m

    
    def _update_plots(self, scan_m):
        now = time.time()

        if now - self.grid_last_update >= self.grid_update_dt:
            self.grid_last_update = now
            prob = self.grid.getProbabilityMap()
            self.im.set_data(prob)

        if scan_m:
            xs, ys = [], []
            for (_, ang, d) in scan_m:
                xs.append(self.x + d * math.cos(self.th + ang))
                ys.append(self.y + d * math.sin(self.th + ang))
            self.scan_plot.set_data(xs, ys)

        self.pose_plot.set_data([self.x], [self.y])

        if self.last_ser:
            t = now - self.t0
            self.ts.append(t)
            self.yaw_deg_hist.append(self.last_ser['yaw_deg'])
            self.yaw_rate_hist.append(self.last_ser['yaw_rate_dps'])
            self.pwmL_hist.append(self.last_ser['pwmL'])
            self.pwmR_hist.append(self.last_ser['pwmR'])
            self.dC_hist.append(self.last_ser['distC'])
            self.dL_hist.append(self.last_ser['distL'])
            self.dR_hist.append(self.last_ser['distR'])

            T = list(self.ts)
            self.l_yaw.set_data(T, list(self.yaw_deg_hist))
            self.l_wz.set_data(T, list(self.yaw_rate_hist))
            self.ax_yaw.relim(); self.ax_yaw.autoscale_view()

            self.l_pwmL.set_data(T, list(self.pwmL_hist))
            self.l_pwmR.set_data(T, list(self.pwmR_hist))
            self.ax_pwm.relim(); self.ax_pwm.autoscale_view()

            self.l_dC.set_data(T, list(self.dC_hist))
            self.l_dL.set_data(T, list(self.dL_hist))
            self.l_dR.set_data(T, list(self.dR_hist))
            self.ax_dist.relim(); self.ax_dist.autoscale_view()

            for ax in (self.ax_yaw, self.ax_pwm, self.ax_dist):
                if len(T) >= 2:
                    ax.set_xlim(max(0.0, T[-1]-30), T[-1])

        if self.headless:
            if now - self.last_save >= self.save_every_s:
                self.last_save = now
                try:
                    self.fig_map.savefig(self.out_map, dpi=110, bbox_inches="tight")
                    self.fig_plot.savefig(self.out_tel, dpi=110, bbox_inches="tight")
                except Exception as e:
                    print("[WARN] savefig:", e)
        else:
            self.fig_map.canvas.draw_idle()
            self.fig_plot.canvas.draw_idle()
            self.plt.pause(0.001)

    def run(self):
        print("[INFO] running… Ctrl+C per uscire")
        try:
            for scan in self.lidar.iterScan():
                s = self.read_serial_nonblocking()
                if s: self.last_ser = s

                now = time.time()
                dt = now - self.last_time
                self.last_time = now
                if self.last_ser:
                    self.step_pose(dt, self.last_ser)

                scan_m = self.process_lidar_scan(scan)
                self._update_plots(scan_m)

                if self.update_ms > 0:
                    target = self.update_ms/1000.0
                    spent = time.time() - now
                    if target - spent > 0:
                        time.sleep(target - spent)
        except KeyboardInterrupt:
            print("\n[INFO] stop richiesto")
        finally:
            self.close()
            if not self.headless:
                self.plt.ioff(); self.plt.show()

def main():
    ap = argparse.ArgumentParser(description="Bot Controller: headless plotting + occupancy")
    ap.add_argument("--serial", default="/dev/ttyACM0")
    ap.add_argument("--lidar",  default="/dev/ttyUSB0")
    ap.add_argument("--baud",   type=int, default=115200)
    ap.add_argument("--kv",     type=float, default=0.0035)
    ap.add_argument("--dz",     type=int,   default=20)
    ap.add_argument("--wheel-base", type=float, default=0.15)
    ap.add_argument("--grid", nargs=4, type=float, default=[-6.0,6.0,-6.0,6.0],
                    metavar=('xmin','xmax','ymin','ymax'))
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--clamp-dist", type=float, default=8.0)
    ap.add_argument("--update-ms",  type=int, default=50)
    ap.add_argument("--headless", dest="headless", action="store_true", default=True)
    ap.add_argument("--no-headless", dest="headless", action="store_false")
    ap.add_argument("--out-map", default="map_latest.png")
    ap.add_argument("--out-telemetry", default="telemetry_latest.png")
    args = ap.parse_args()

    plt, backend, headless = setup_matplotlib(args.headless)
    print(f"[INFO] matplotlib backend: {backend} (headless={headless})")

    bc = BotController(
        plt=plt, headless=headless,
        out_map=args.out_map, out_tel=args.out_telemetry,
        port_serial=args.serial,
        port_lidar=args.lidar,
        baud=args.baud,
        kv=args.kv,
        deadzone=args.dz,
        wheel_base=args.wheel_base,
        grid=tuple(args.grid),
        res=args.res,
        imu_direct=True,
        clamp_dist_m=args.clamp_dist,
        update_ms=args.update_ms
    )
    bc.run()

if __name__ == "__main__":
    main()
