import argparse
import math
import time
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.collections import LineCollection

from lidarLib import Lidar
from occupancyGrid import OccupancyGrid

try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None


def wrap_angle(theta: float) -> float:
    return (theta + math.pi) % (2.0 * math.pi) - math.pi


def transform_points(points: np.ndarray, pose: Tuple[float, float, float]) -> np.ndarray:
    if points.size == 0:
        return np.empty((0, 2), dtype=float)
    x, y, th = pose
    ct, st = math.cos(th), math.sin(th)
    rot = np.array([[ct, -st], [st, ct]], dtype=float)
    return points @ rot.T + np.array([x, y], dtype=float)


def nearest_neighbors(source_pts: np.ndarray, target_pts: np.ndarray, max_distance: float):
    if source_pts.size == 0 or target_pts.size == 0:
        return np.zeros((0,), dtype=bool), np.zeros((0,), dtype=np.int32), np.zeros((0,), dtype=float)

    if cKDTree is not None:
        tree = cKDTree(target_pts)
        dists, idx = tree.query(source_pts, k=1)
    else:
        idx = np.empty(source_pts.shape[0], dtype=np.int32)
        d2_min = np.full(source_pts.shape[0], np.inf, dtype=float)
        chunk = 256
        for i in range(0, source_pts.shape[0], chunk):
            part = source_pts[i:i + chunk]
            diff = part[:, None, :] - target_pts[None, :, :]
            d2 = np.einsum("ijk,ijk->ij", diff, diff)
            local_idx = np.argmin(d2, axis=1)
            idx[i:i + len(part)] = local_idx
            d2_min[i:i + len(part)] = d2[np.arange(len(part)), local_idx]
        dists = np.sqrt(d2_min)

    valid = dists <= max_distance
    return valid, idx, dists


def estimate_translation_known_rotation(
    current_pts: np.ndarray,
    previous_pts: np.ndarray,
    delta_theta: float,
    max_iters: int,
    max_corr_dist: float,
    min_pairs: int,
):
    if current_pts.shape[0] < min_pairs or previous_pts.shape[0] < min_pairs:
        return None, float("inf"), 0

    ct, st = math.cos(delta_theta), math.sin(delta_theta)
    rot = np.array([[ct, -st], [st, ct]], dtype=float)
    src0 = current_pts @ rot.T

    trans = np.zeros(2, dtype=float)
    rmse = float("inf")
    pairs = 0

    for _ in range(max_iters):
        src = src0 + trans
        valid, nn_idx, nn_d = nearest_neighbors(src, previous_pts, max_corr_dist)
        pairs = int(valid.sum())
        if pairs < min_pairs:
            return None, float("inf"), pairs

        residual = previous_pts[nn_idx[valid]] - src[valid]
        step = residual.mean(axis=0)
        trans = trans + step
        rmse = float(np.sqrt(np.mean(np.square(nn_d[valid]))))
        if float(np.linalg.norm(step)) < 1e-4:
            break

    return trans, rmse, pairs


def pwm_to_speed(pwm: int, kv: float, deadzone: int) -> float:
    sign = 1.0 if pwm >= 0 else -1.0
    ap = abs(int(pwm))
    if ap <= deadzone:
        return 0.0
    return sign * kv * float(ap - deadzone)


def sector_min_range(points: np.ndarray, amin: float, amax: float, default: float) -> float:
    if points.size == 0:
        return default
    ang = np.arctan2(points[:, 1], points[:, 0])
    rng = np.hypot(points[:, 0], points[:, 1])
    mask = (ang >= amin) & (ang <= amax)
    if not np.any(mask):
        return default
    return float(np.min(rng[mask]))


class ArduinoBridge:
    def __init__(self, port: str, baudrate: int, timeout: float):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.ser = None
        self.rx_buf = ""

        self.last_sent = (None, None)
        self.last_send_ts = 0.0

        self.telemetry = None

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(1.8)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def _parse_line(self, line: str):
        if not line or not line.startswith("STAT,"):
            return
        parts = line.split(",")
        if len(parts) < 9:
            return
        try:
            self.telemetry = {
                "ms": int(parts[1]),
                "yaw_deg": float(parts[2]),
                "yaw_rate_dps": float(parts[3]),
                "pwmL": int(parts[4]),
                "pwmR": int(parts[5]),
                "distC_cm": int(parts[6]),
                "irL_raw": int(parts[7]),
                "irR_raw": int(parts[8]),
                "rx_ts": time.monotonic(),
            }
        except ValueError:
            return

    def poll(self):
        if not self.ser or not self.ser.is_open:
            return

        n = self.ser.in_waiting
        if n <= 0:
            return
        data = self.ser.read(n).decode("utf-8", errors="ignore")
        self.rx_buf += data

        while "\n" in self.rx_buf:
            line, self.rx_buf = self.rx_buf.split("\n", 1)
            self._parse_line(line.strip())

    def send_pwm(self, pwm_l: int, pwm_r: int, force: bool = False):
        if not self.ser or not self.ser.is_open:
            return

        pwm_l = int(max(-255, min(255, pwm_l)))
        pwm_r = int(max(-255, min(255, pwm_r)))
        now = time.monotonic()
        if not force and (pwm_l, pwm_r) == self.last_sent and (now - self.last_send_ts) < 0.15:
            return

        msg = f"CMD,{pwm_l},{pwm_r}\n".encode("ascii")
        self.ser.write(msg)
        self.last_sent = (pwm_l, pwm_r)
        self.last_send_ts = now

    def stop(self):
        if self.ser and self.ser.is_open:
            self.ser.write(b"STOP\n")
        self.last_sent = (0, 0)
        self.last_send_ts = time.monotonic()

    def gyro_zero(self):
        if self.ser and self.ser.is_open:
            self.ser.write(b"GYRO_ZERO\n")


class RobotViewer:
    def __init__(self, grid_bounds, max_range_m, grid_shape):
        x_min, x_max, y_min, y_max = grid_bounds
        self.fig, (self.ax_map, self.ax_local) = plt.subplots(1, 2, figsize=(14, 6))

        self.map_img = self.ax_map.imshow(
            np.full(grid_shape, 0.5, dtype=float),
            origin="lower",
            extent=(x_min, x_max, y_min, y_max),
            cmap="gray_r",
            vmin=0.0,
            vmax=1.0,
            interpolation="nearest",
        )
        self.traj_line = self.ax_map.plot([], [], color="#1f77b4", linewidth=1.5)[0]
        self.pose_dot = self.ax_map.plot([], [], "ro", markersize=5)[0]
        self.map_rays = LineCollection([], colors="#ff7f0e", linewidths=0.35, alpha=0.55)
        self.ax_map.add_collection(self.map_rays)
        self.status_txt = self.ax_map.text(
            0.02,
            0.98,
            "",
            transform=self.ax_map.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            family="monospace",
        )

        self.ax_map.set_title("Robot SLAM (Global)")
        self.ax_map.set_xlabel("X [m]")
        self.ax_map.set_ylabel("Y [m]")
        self.ax_map.set_aspect("equal", "box")
        self.ax_map.set_xlim(x_min, x_max)
        self.ax_map.set_ylim(y_min, y_max)

        self.ax_local.set_title("LiDAR Rays (Local Frame)")
        self.ax_local.set_xlabel("X [m]")
        self.ax_local.set_ylabel("Y [m]")
        self.ax_local.set_aspect("equal", "box")
        self.ax_local.set_xlim(-max_range_m, max_range_m)
        self.ax_local.set_ylim(-max_range_m, max_range_m)
        self.ax_local.grid(True, alpha=0.25)
        self.ax_local.plot([0.0], [0.0], "ro", markersize=5)

        self.local_rays = LineCollection([], colors="#2ca02c", linewidths=0.5, alpha=0.85)
        self.local_hits = self.ax_local.scatter([], [], s=6, c="#1f77b4", alpha=0.9)
        self.ax_local.add_collection(self.local_rays)

        self.fig.tight_layout()
        plt.ion()
        plt.show(block=False)

    def is_open(self):
        return plt.fignum_exists(self.fig.number)

    def update(self, prob_map, traj_x, traj_y, pose, local_pts, status_lines):
        self.map_img.set_data(prob_map)
        self.traj_line.set_data(traj_x, traj_y)
        self.pose_dot.set_data([pose[0]], [pose[1]])

        if local_pts.size > 0:
            hits_world = transform_points(local_pts, pose)
            origins = np.repeat(np.array([[pose[0], pose[1]]], dtype=float), len(hits_world), axis=0)
            seg_world = np.stack([origins, hits_world], axis=1)
            self.map_rays.set_segments(seg_world)

            seg_local = np.stack([np.zeros_like(local_pts), local_pts], axis=1)
            self.local_rays.set_segments(seg_local)
            self.local_hits.set_offsets(local_pts)
        else:
            self.map_rays.set_segments([])
            self.local_rays.set_segments([])
            self.local_hits.set_offsets(np.empty((0, 2), dtype=float))

        self.status_txt.set_text("\n".join(status_lines))
        self.fig.canvas.draw_idle()
        plt.pause(0.001)


class RobotController:
    def __init__(self, args):
        self.args = args
        self.arduino = ArduinoBridge(args.serial_port, args.serial_baud, timeout=args.serial_timeout)
        self.lidar = Lidar(
            args.lidar_port,
            baudrate=args.lidar_baud,
            timeout=args.lidar_timeout,
            motor_pwm=args.lidar_motor_pwm,
        )

        x_min, x_max, y_min, y_max = args.grid
        self.grid_bounds = (x_min, x_max, y_min, y_max)
        self.grid = OccupancyGrid(x_min, x_max, y_min, y_max, args.resolution)

        self.viewer = None
        if not args.no_gui:
            self.viewer = RobotViewer(self.grid_bounds, args.max_range, self.grid.log_odds.shape)

        self.pose = (0.0, 0.0, 0.0)
        self.trajectory = [self.pose]
        self.traj_x = [0.0]
        self.traj_y = [0.0]

        self.yaw0 = None
        self.prev_theta = None
        self.prev_local_pts = None
        self.last_scan_ts = None
        self.last_cmd = (0, 0)
        self.last_rmse = 0.0
        self.last_pairs = 0

    def _on_post_scan(self, raw_scan, local_pts, telem, cmd, front, left, right, scan_idx):
        # Extension hook for derived controllers (e.g. HTTP streaming).
        return

    def _on_shutdown(self, scan_idx):
        # Extension hook for derived controllers.
        return

    def _scan_to_local(self, scan):
        points = []
        grid_scan = []
        step = max(1, int(self.args.scan_decimation))
        for i, (q, angle_deg, dist_mm) in enumerate(scan):
            if i % step != 0:
                continue
            if q < self.args.min_quality:
                continue
            d = dist_mm / 1000.0
            if d < self.args.min_range or d > self.args.max_range:
                continue
            a = math.radians(angle_deg)
            points.append((d * math.cos(a), d * math.sin(a)))
            grid_scan.append((q, a, d))
        if not points:
            return np.empty((0, 2), dtype=float), []
        return np.asarray(points, dtype=float), grid_scan

    def _nav_command(self, local_pts: np.ndarray, telem):
        fov = math.radians(self.args.front_fov_deg * 0.5)
        front = sector_min_range(local_pts, -fov, fov, self.args.max_range)
        left = sector_min_range(local_pts, math.radians(30), math.radians(120), self.args.max_range)
        right = sector_min_range(local_pts, math.radians(-120), math.radians(-30), self.args.max_range)

        if telem is not None:
            if telem["distC_cm"] > 0:
                front = min(front, telem["distC_cm"] / 100.0)

        ir_l = False
        ir_r = False
        if telem is not None:
            ir_l = telem["irL_raw"] > self.args.ir_threshold
            ir_r = telem["irR_raw"] > self.args.ir_threshold

        if front < self.args.stop_distance or ir_l or ir_r:
            if ir_l and not ir_r:
                return (self.args.turn_pwm, -self.args.turn_pwm), front, left, right
            if ir_r and not ir_l:
                return (-self.args.turn_pwm, self.args.turn_pwm), front, left, right
            if left >= right:
                return (-self.args.turn_pwm, self.args.turn_pwm), front, left, right
            return (self.args.turn_pwm, -self.args.turn_pwm), front, left, right

        if front < self.args.slow_distance:
            base = self.args.slow_pwm
        else:
            base = self.args.cruise_pwm

        turn = int((left - right) * self.args.steer_gain)
        turn = int(max(-self.args.turn_pwm, min(self.args.turn_pwm, turn)))
        pwm_l = int(max(-255, min(255, base - turn)))
        pwm_r = int(max(-255, min(255, base + turn)))
        return (pwm_l, pwm_r), front, left, right

    def _fallback_motion(self, dt):
        pwm_l, pwm_r = self.last_cmd
        v_l = pwm_to_speed(pwm_l, self.args.kv, self.args.deadzone)
        v_r = pwm_to_speed(pwm_r, self.args.kv, self.args.deadzone)
        v = 0.5 * (v_l + v_r)
        x, y, th = self.pose
        x = x + v * dt * math.cos(th)
        y = y + v * dt * math.sin(th)
        self.pose = (x, y, th)

    def _update_pose_fused(self, local_pts: np.ndarray, theta_now: float, dt_scan: float):
        x, y, _ = self.pose
        self.pose = (x, y, theta_now)

        if self.prev_local_pts is None or self.prev_theta is None:
            self.prev_local_pts = local_pts
            self.prev_theta = theta_now
            return

        dtheta = wrap_angle(theta_now - self.prev_theta)
        trans_prev, rmse, pairs = estimate_translation_known_rotation(
            current_pts=local_pts,
            previous_pts=self.prev_local_pts,
            delta_theta=dtheta,
            max_iters=self.args.icp_iters,
            max_corr_dist=self.args.icp_max_corr,
            min_pairs=self.args.icp_min_pairs,
        )

        used_icp = False
        if trans_prev is not None:
            step = float(np.linalg.norm(trans_prev))
            if rmse <= self.args.icp_max_rmse and step <= self.args.max_step_translation:
                c, s = math.cos(self.prev_theta), math.sin(self.prev_theta)
                dx = c * trans_prev[0] - s * trans_prev[1]
                dy = s * trans_prev[0] + c * trans_prev[1]
                x, y, _ = self.pose
                self.pose = (x + float(dx), y + float(dy), theta_now)
                used_icp = True
                self.last_rmse = rmse
                self.last_pairs = pairs

        if not used_icp:
            self._fallback_motion(dt_scan)
            self.last_pairs = pairs
            self.last_rmse = rmse if trans_prev is not None else float("inf")

        self.prev_local_pts = local_pts
        self.prev_theta = theta_now

    def _save_outputs(self):
        self.grid.clampLogOdds()
        prob = self.grid.getProbabilityMap()
        x_min, x_max, y_min, y_max = self.grid_bounds

        fig, ax = plt.subplots(figsize=(7, 7))
        ax.set_title("Robot SLAM Occupancy Map")
        ax.imshow(
            prob,
            origin="lower",
            extent=(x_min, x_max, y_min, y_max),
            cmap="gray_r",
            vmin=0.0,
            vmax=1.0,
        )
        traj = np.asarray(self.trajectory, dtype=float)
        if traj.shape[0] > 0:
            ax.plot(traj[:, 0], traj[:, 1], color="#1f77b4", linewidth=1.2)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_aspect("equal", "box")
        fig.tight_layout()
        fig.savefig(self.args.map_out, dpi=240, bbox_inches="tight")
        plt.close(fig)

        header = "x_m,y_m,theta_rad"
        np.savetxt(self.args.traj_out, traj, delimiter=",", header=header, comments="")

    def run(self):
        scan_idx = 0
        last_log = time.monotonic()

        print(f"[INFO] Arduino: {self.args.serial_port} @ {self.args.serial_baud}")
        self.arduino.connect()
        self.arduino.gyro_zero()

        print(f"[INFO] LiDAR: {self.args.lidar_port} @ {self.args.lidar_baud}")
        self.lidar.connect()
        print("[LIDAR] INFO:", self.lidar.getInfo())
        print("[LIDAR] HEALTH:", self.lidar.getHealth())
        self.lidar.startScan()

        try:
            for raw_scan in self.lidar.iterScan(min_points=self.args.min_scan_points):
                self.arduino.poll()
                telem = self.arduino.telemetry
                if telem is None:
                    self.arduino.send_pwm(0, 0, force=True)
                    continue

                yaw_rad = math.radians(telem["yaw_deg"])
                if self.yaw0 is None:
                    self.yaw0 = yaw_rad
                theta = wrap_angle(yaw_rad - self.yaw0)

                local_pts, scan_for_grid = self._scan_to_local(raw_scan)
                if local_pts.shape[0] < self.args.icp_min_pairs:
                    continue

                now = time.monotonic()
                if self.last_scan_ts is None:
                    dt_scan = 0.0
                else:
                    dt_scan = max(0.0, now - self.last_scan_ts)
                self.last_scan_ts = now

                self._update_pose_fused(local_pts, theta, dt_scan)
                self.grid.inverse_sensor_update(self.pose, scan_for_grid)
                if self.args.clamp_every > 0 and (scan_idx % self.args.clamp_every == 0):
                    self.grid.clampLogOdds()

                cmd, front, left, right = self._nav_command(local_pts, telem)
                self.last_cmd = cmd
                self.arduino.send_pwm(cmd[0], cmd[1])

                self.trajectory.append(self.pose)
                self.traj_x.append(self.pose[0])
                self.traj_y.append(self.pose[1])
                scan_idx += 1
                self._on_post_scan(raw_scan, local_pts, telem, cmd, front, left, right, scan_idx)

                if self.viewer is not None and (scan_idx % max(1, self.args.gui_every) == 0):
                    status = [
                        f"scan={scan_idx}",
                        f"x={self.pose[0]: .2f} y={self.pose[1]: .2f}",
                        f"yaw={math.degrees(self.pose[2]): .1f} deg  gyro={telem['yaw_rate_dps']: .1f} dps",
                        f"front={front:.2f}m left={left:.2f}m right={right:.2f}m",
                        f"cmd=({cmd[0]:d},{cmd[1]:d})  icp_rmse={self.last_rmse:.3f}  pairs={self.last_pairs}",
                    ]
                    self.viewer.update(
                        prob_map=self.grid.getProbabilityMap(),
                        traj_x=self.traj_x,
                        traj_y=self.traj_y,
                        pose=self.pose,
                        local_pts=local_pts,
                        status_lines=status,
                    )
                    if not self.viewer.is_open():
                        print("[INFO] GUI closed, stopping")
                        break

                if (now - last_log) > 1.0:
                    last_log = now
                    print(
                        f"[CTRL] scan={scan_idx:5d} pose=({self.pose[0]:.2f},{self.pose[1]:.2f},"
                        f"{math.degrees(self.pose[2]):.1f}deg) "
                        f"cmd=({cmd[0]:d},{cmd[1]:d}) front={front:.2f} rmse={self.last_rmse:.3f}"
                    )

                if self.args.max_scans > 0 and scan_idx >= self.args.max_scans:
                    print(f"[INFO] reached --max-scans={self.args.max_scans}")
                    break

        except KeyboardInterrupt:
            print("\n[INFO] interrupted")
        finally:
            try:
                self.arduino.stop()
            except Exception:
                pass
            try:
                self.lidar.stopScan()
            except Exception:
                pass
            try:
                self.lidar.disconnect()
            except Exception:
                pass
            try:
                self.arduino.disconnect()
            except Exception:
                pass
            try:
                self._on_shutdown(scan_idx)
            except Exception:
                pass

            self._save_outputs()
            print(f"[INFO] map saved to: {self.args.map_out}")
            print(f"[INFO] trajectory saved to: {self.args.traj_out}")

            if self.viewer is not None and self.viewer.is_open():
                plt.ioff()
                plt.show()


def build_parser():
    parser = argparse.ArgumentParser(
        description="Autonomous robot controller with LiDAR + MPU6050 fusion and occupancy SLAM"
    )
    parser.add_argument("--serial-port", default="/dev/ttyACM0", help="Arduino serial port")
    parser.add_argument("--serial-baud", type=int, default=115200, help="Arduino serial baudrate")
    parser.add_argument("--serial-timeout", type=float, default=0.0, help="Arduino serial timeout [s]")

    parser.add_argument("--lidar-port", default="/dev/ttyUSB0", help="RPLidar serial port")
    parser.add_argument("--lidar-baud", type=int, default=115200, help="RPLidar baudrate")
    parser.add_argument("--lidar-timeout", type=float, default=1.0, help="RPLidar timeout [s]")
    parser.add_argument("--lidar-motor-pwm", type=int, default=660, help="RPLidar motor PWM (0..1023)")

    parser.add_argument("--grid", nargs=4, type=float, default=[-8.0, 8.0, -8.0, 8.0],
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
    parser.add_argument("--resolution", type=float, default=0.05, help="Occupancy grid resolution [m]")

    parser.add_argument("--min-range", type=float, default=0.12, help="Min LiDAR range [m]")
    parser.add_argument("--max-range", type=float, default=8.0, help="Max LiDAR range [m]")
    parser.add_argument("--scan-decimation", type=int, default=1, help="Keep one point every N")
    parser.add_argument("--min-scan-points", type=int, default=80, help="Min points for one LiDAR revolution")
    parser.add_argument("--min-quality", type=int, default=0, help="Minimum LiDAR quality")

    parser.add_argument("--icp-iters", type=int, default=12, help="Translation-only ICP iterations")
    parser.add_argument("--icp-max-corr", type=float, default=0.30, help="ICP max correspondence [m]")
    parser.add_argument("--icp-min-pairs", type=int, default=50, help="Minimum ICP pairs")
    parser.add_argument("--icp-max-rmse", type=float, default=0.12, help="Max ICP RMSE [m]")
    parser.add_argument("--max-step-translation", type=float, default=0.40, help="Max accepted step [m]")
    parser.add_argument("--kv", type=float, default=0.0034, help="Fallback speed model gain [m/s per PWM]")
    parser.add_argument("--deadzone", type=int, default=20, help="Fallback speed model deadzone PWM")

    parser.add_argument("--cruise-pwm", type=int, default=95, help="Cruise PWM")
    parser.add_argument("--slow-pwm", type=int, default=70, help="Slow PWM near obstacles")
    parser.add_argument("--turn-pwm", type=int, default=105, help="In-place turn PWM")
    parser.add_argument("--steer-gain", type=float, default=90.0, help="Steering gain from side clearance")
    parser.add_argument("--front-fov-deg", type=float, default=60.0, help="Front sector angle for obstacle check")
    parser.add_argument("--stop-distance", type=float, default=0.38, help="Emergency stop/turn distance [m]")
    parser.add_argument("--slow-distance", type=float, default=0.75, help="Slowdown distance [m]")
    parser.add_argument("--ir-threshold", type=int, default=650, help="IR raw threshold for emergency")

    parser.add_argument("--clamp-every", type=int, default=5, help="Clamp log-odds every N scans")
    parser.add_argument("--gui-every", type=int, default=1, help="GUI refresh every N scans")
    parser.add_argument("--no-gui", action="store_true", help="Disable GUI")
    parser.add_argument("--max-scans", type=int, default=0, help="Stop after N scans (0=infinite)")
    parser.add_argument("--map-out", default="robot_map.png", help="Output map image")
    parser.add_argument("--traj-out", default="robot_trajectory.csv", help="Output trajectory CSV")
    return parser


def parse_args():
    return build_parser().parse_args()


def main():
    args = parse_args()
    ctrl = RobotController(args)
    ctrl.run()


if __name__ == "__main__":
    main()
