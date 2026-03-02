import argparse
import math
import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

from lidarLib import Lidar, LidarError
from occupancyGrid import OccupancyGrid

try:
    from scipy.spatial import cKDTree
except Exception:
    cKDTree = None


def wrap_angle(theta):
    return (theta + math.pi) % (2.0 * math.pi) - math.pi


def transform_points(points, pose):
    if points.size == 0:
        return np.empty((0, 2), dtype=float)
    x, y, th = pose
    ct, st = math.cos(th), math.sin(th)
    rot = np.array([[ct, -st], [st, ct]], dtype=float)
    return points @ rot.T + np.array([x, y], dtype=float)


def left_compose_pose(delta, pose):
    dx, dy, dth = delta
    x, y, th = pose
    cd, sd = math.cos(dth), math.sin(dth)
    xn = cd * x - sd * y + dx
    yn = sd * x + cd * y + dy
    thn = wrap_angle(th + dth)
    return (float(xn), float(yn), float(thn))


def best_rigid_transform_2d(source_pts, target_pts):
    src_cent = source_pts.mean(axis=0)
    tgt_cent = target_pts.mean(axis=0)

    src_zero = source_pts - src_cent
    tgt_zero = target_pts - tgt_cent

    cov = src_zero.T @ tgt_zero
    u, _, vt = np.linalg.svd(cov)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0.0:
        vt[-1, :] *= -1.0
        rot = vt.T @ u.T

    trans = tgt_cent - rot @ src_cent
    theta = math.atan2(rot[1, 0], rot[0, 0])
    return (float(trans[0]), float(trans[1]), float(theta))


def nearest_neighbors(source_pts, target_pts, max_distance):
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


def icp_scan_to_map(
    source_local_pts,
    target_global_pts,
    init_pose,
    max_iters,
    max_corr_dist,
    min_pairs,
):
    if source_local_pts.shape[0] < min_pairs or target_global_pts.shape[0] < min_pairs:
        return init_pose, False, float("inf"), 0

    pose = init_pose
    rmse = float("inf")
    pairs = 0

    for _ in range(max_iters):
        source_world = transform_points(source_local_pts, pose)
        valid, nn_idx, nn_d = nearest_neighbors(source_world, target_global_pts, max_corr_dist)
        pairs = int(valid.sum())
        if pairs < min_pairs:
            return pose, False, float("inf"), pairs

        matched_src = source_world[valid]
        matched_tgt = target_global_pts[nn_idx[valid]]
        delta = best_rigid_transform_2d(matched_src, matched_tgt)

        new_pose = left_compose_pose(delta, pose)
        rmse = float(np.sqrt(np.mean(np.square(nn_d[valid]))))

        dxy = math.hypot(new_pose[0] - pose[0], new_pose[1] - pose[1])
        dth = abs(wrap_angle(new_pose[2] - pose[2]))
        pose = new_pose
        if dxy < 1e-4 and dth < 1e-4:
            break

    return pose, True, rmse, pairs


def voxel_downsample(points, voxel_size):
    if points.size == 0 or voxel_size <= 0.0:
        return points
    q = np.floor(points / voxel_size).astype(np.int32)
    _, idx = np.unique(q, axis=0, return_index=True)
    idx.sort()
    return points[idx]


def scan_to_local_points(scan, min_range_m, max_range_m, decimation):
    pts = []
    scan_for_grid = []
    step = max(1, int(decimation))

    for i, (quality, angle_deg, dist_mm) in enumerate(scan):
        if (i % step) != 0:
            continue
        dist_m = dist_mm / 1000.0
        if dist_m < min_range_m or dist_m > max_range_m:
            continue
        angle_rad = math.radians(angle_deg)
        pts.append((dist_m * math.cos(angle_rad), dist_m * math.sin(angle_rad)))
        scan_for_grid.append((quality, angle_rad, dist_m))

    if not pts:
        return np.empty((0, 2), dtype=float), []
    return np.asarray(pts, dtype=float), scan_for_grid


def rebuild_submap(keyframes, voxel_size):
    blocks = [pts for _, pts in keyframes if pts.size > 0]
    if not blocks:
        return np.empty((0, 2), dtype=float)
    return voxel_downsample(np.vstack(blocks), voxel_size)


class SlamViewer:
    def __init__(self, grid_bounds, max_range_m, grid_shape):
        x_min, x_max, y_min, y_max = grid_bounds
        self.fig, (self.ax_map, self.ax_scan) = plt.subplots(1, 2, figsize=(14, 6))

        self.map_img = self.ax_map.imshow(
            np.full(grid_shape, 0.5, dtype=float),
            origin="lower",
            extent=(x_min, x_max, y_min, y_max),
            cmap="gray_r",
            vmin=0.0,
            vmax=1.0,
            interpolation="nearest",
        )
        self.traj_line = self.ax_map.plot([], [], color="#1f77b4", linewidth=1.6)[0]
        self.pose_dot = self.ax_map.plot([], [], "ro", markersize=5)[0]
        self.map_rays = LineCollection([], colors="#ff7f0e", linewidths=0.35, alpha=0.5)
        self.ax_map.add_collection(self.map_rays)
        self.status_txt = self.ax_map.text(
            0.02, 0.98, "", transform=self.ax_map.transAxes, va="top", ha="left", fontsize=9
        )

        self.ax_map.set_title("Occupancy SLAM (Global)")
        self.ax_map.set_xlabel("X [m]")
        self.ax_map.set_ylabel("Y [m]")
        self.ax_map.set_aspect("equal", "box")
        self.ax_map.set_xlim(x_min, x_max)
        self.ax_map.set_ylim(y_min, y_max)

        self.ax_scan.set_title("Live LiDAR Rays (Local)")
        self.ax_scan.set_xlabel("X [m]")
        self.ax_scan.set_ylabel("Y [m]")
        self.ax_scan.set_aspect("equal", "box")
        self.ax_scan.set_xlim(-max_range_m, max_range_m)
        self.ax_scan.set_ylim(-max_range_m, max_range_m)
        self.ax_scan.grid(True, alpha=0.25)
        self.ax_scan.plot([0.0], [0.0], "ro", markersize=5)

        self.local_rays = LineCollection([], colors="#2ca02c", linewidths=0.45, alpha=0.8)
        self.local_hits = self.ax_scan.scatter([], [], s=6, c="#1f77b4", alpha=0.8)
        self.ax_scan.add_collection(self.local_rays)

        self.fig.tight_layout()
        plt.ion()
        plt.show(block=False)

    def is_open(self):
        return plt.fignum_exists(self.fig.number)

    def update(self, prob_map, traj_x, traj_y, pose, local_pts, rmse, pairs):
        self.map_img.set_data(prob_map)
        self.traj_line.set_data(traj_x, traj_y)
        self.pose_dot.set_data([pose[0]], [pose[1]])

        if local_pts.size > 0:
            hit_world = transform_points(local_pts, pose)
            origins = np.repeat(np.array([[pose[0], pose[1]]], dtype=float), len(hit_world), axis=0)
            seg_world = np.stack([origins, hit_world], axis=1)
            self.map_rays.set_segments(seg_world)

            local_origins = np.zeros_like(local_pts)
            seg_local = np.stack([local_origins, local_pts], axis=1)
            self.local_rays.set_segments(seg_local)
            self.local_hits.set_offsets(local_pts)
        else:
            self.map_rays.set_segments([])
            self.local_rays.set_segments([])
            self.local_hits.set_offsets(np.empty((0, 2), dtype=float))

        self.status_txt.set_text(
            f"x={pose[0]: .2f} m\ny={pose[1]: .2f} m\nyaw={math.degrees(pose[2]): .1f} deg\n"
            f"icp_rmse={rmse:.3f} m\npairs={pairs}"
        )
        self.fig.canvas.draw_idle()
        plt.pause(0.001)


def parse_args():
    parser = argparse.ArgumentParser(description="2D LiDAR SLAM for RPLidar A1 with live GUI")
    parser.add_argument("--lidar-port", default="/dev/ttyUSB0", help="RPLidar serial port")
    parser.add_argument("--baudrate", type=int, default=115200, help="RPLidar baudrate")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial timeout [s]")
    parser.add_argument("--motor-pwm", type=int, default=660, help="Motor PWM (0..1023)")

    parser.add_argument("--grid", nargs=4, type=float, default=[-8.0, 8.0, -8.0, 8.0],
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX"))
    parser.add_argument("--resolution", type=float, default=0.05, help="Occupancy grid resolution [m]")
    parser.add_argument("--min-range", type=float, default=0.12, help="Min valid LiDAR range [m]")
    parser.add_argument("--max-range", type=float, default=8.0, help="Max valid LiDAR range [m]")
    parser.add_argument("--scan-decimation", type=int, default=1, help="Keep 1 sample every N")
    parser.add_argument("--min-scan-points", type=int, default=80, help="Minimum points for one revolution")

    parser.add_argument("--icp-iters", type=int, default=20, help="ICP iterations")
    parser.add_argument("--icp-max-corr", type=float, default=0.35, help="ICP max correspondence distance [m]")
    parser.add_argument("--icp-min-pairs", type=int, default=50, help="Minimum matched pairs for ICP")
    parser.add_argument("--icp-max-rmse", type=float, default=0.12, help="Reject ICP pose if RMSE above threshold [m]")
    parser.add_argument("--max-step-trans", type=float, default=0.40, help="Max accepted translation per scan [m]")
    parser.add_argument("--max-step-rot-deg", type=float, default=25.0, help="Max accepted rotation per scan [deg]")

    parser.add_argument("--keyframe-trans", type=float, default=0.10, help="Add keyframe after translation [m]")
    parser.add_argument("--keyframe-rot-deg", type=float, default=8.0, help="Add keyframe after rotation [deg]")
    parser.add_argument("--submap-keyframes", type=int, default=40, help="Keyframes kept in local submap")
    parser.add_argument("--submap-voxel", type=float, default=0.04, help="Downsample voxel size [m]")

    parser.add_argument("--clamp-every", type=int, default=5, help="Clamp log-odds every N scans")
    parser.add_argument("--gui-every", type=int, default=1, help="GUI update period in scans")
    parser.add_argument("--no-gui", action="store_true", help="Disable live GUI")
    parser.add_argument("--max-scans", type=int, default=0, help="Stop after N scans (0=infinite)")

    parser.add_argument("--map-out", default="occupancy_grid.png", help="Output map image")
    parser.add_argument("--traj-out", default="trajectory.csv", help="Output trajectory CSV")
    return parser.parse_args()


def save_outputs(prob_map, poses, grid_bounds, map_out, traj_out):
    x_min, x_max, y_min, y_max = grid_bounds
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_title("Occupancy Grid SLAM")
    ax.imshow(
        prob_map,
        origin="lower",
        extent=(x_min, x_max, y_min, y_max),
        cmap="gray_r",
        vmin=0.0,
        vmax=1.0,
    )
    if poses:
        traj = np.asarray(poses, dtype=float)
        ax.plot(traj[:, 0], traj[:, 1], color="#1f77b4", linewidth=1.2)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_aspect("equal", "box")
    fig.tight_layout()
    fig.savefig(map_out, dpi=240, bbox_inches="tight")
    plt.close(fig)

    traj = np.asarray(poses, dtype=float) if poses else np.empty((0, 3), dtype=float)
    header = "x_m,y_m,theta_rad"
    np.savetxt(traj_out, traj, delimiter=",", header=header, comments="")


def run():
    args = parse_args()
    x_min, x_max, y_min, y_max = args.grid
    grid_bounds = (x_min, x_max, y_min, y_max)

    grid = OccupancyGrid(x_min, x_max, y_min, y_max, args.resolution)
    lidar = Lidar(
        args.lidar_port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        motor_pwm=args.motor_pwm,
    )

    viewer = None
    if not args.no_gui:
        viewer = SlamViewer(grid_bounds, args.max_range, grid.log_odds.shape)

    pose = (0.0, 0.0, 0.0)
    poses = [pose]
    traj_x = [pose[0]]
    traj_y = [pose[1]]

    last_local_scan = None
    keyframes = deque(maxlen=max(2, int(args.submap_keyframes)))
    submap_points = np.empty((0, 2), dtype=float)

    max_step_rot = math.radians(args.max_step_rot_deg)
    keyframe_rot = math.radians(args.keyframe_rot_deg)

    scan_idx = 0
    last_log_ts = time.time()
    last_rmse = 0.0
    last_pairs = 0

    print(f"[INFO] starting SLAM on {args.lidar_port} @ {args.baudrate} baud")
    try:
        lidar.connect()
        print("[LIDAR] INFO:", lidar.getInfo())
        print("[LIDAR] HEALTH:", lidar.getHealth())
        lidar.startScan()

        for raw_scan in lidar.iterScan(min_points=args.min_scan_points):
            local_pts, scan_for_grid = scan_to_local_points(
                raw_scan,
                min_range_m=args.min_range,
                max_range_m=args.max_range,
                decimation=args.scan_decimation,
            )
            if local_pts.shape[0] < args.icp_min_pairs:
                continue

            if scan_idx > 0:
                candidate_pose = pose
                candidate_ok = False
                candidate_rmse = float("inf")
                candidate_pairs = 0

                if submap_points.shape[0] >= args.icp_min_pairs:
                    candidate_pose, candidate_ok, candidate_rmse, candidate_pairs = icp_scan_to_map(
                        local_pts,
                        submap_points,
                        init_pose=pose,
                        max_iters=args.icp_iters,
                        max_corr_dist=args.icp_max_corr,
                        min_pairs=args.icp_min_pairs,
                    )

                if (not candidate_ok or candidate_rmse > args.icp_max_rmse) and last_local_scan is not None:
                    frame_target = transform_points(last_local_scan, pose)
                    ff_pose, ff_ok, ff_rmse, ff_pairs = icp_scan_to_map(
                        local_pts,
                        frame_target,
                        init_pose=pose,
                        max_iters=max(8, args.icp_iters // 2),
                        max_corr_dist=args.icp_max_corr,
                        min_pairs=args.icp_min_pairs,
                    )
                    if ff_ok and ff_rmse < candidate_rmse:
                        candidate_pose = ff_pose
                        candidate_ok = True
                        candidate_rmse = ff_rmse
                        candidate_pairs = ff_pairs

                if candidate_ok:
                    step_trans = math.hypot(candidate_pose[0] - pose[0], candidate_pose[1] - pose[1])
                    step_rot = abs(wrap_angle(candidate_pose[2] - pose[2]))
                    if (
                        step_trans <= args.max_step_trans
                        and step_rot <= max_step_rot
                        and candidate_rmse <= args.icp_max_rmse
                    ):
                        pose = candidate_pose
                        last_rmse = candidate_rmse
                        last_pairs = candidate_pairs

            poses.append(pose)
            traj_x.append(pose[0])
            traj_y.append(pose[1])

            grid.inverse_sensor_update(pose, scan_for_grid)
            if args.clamp_every > 0 and (scan_idx % args.clamp_every == 0):
                grid.clampLogOdds()

            add_keyframe = False
            if not keyframes:
                add_keyframe = True
            else:
                k_pose = keyframes[-1][0]
                k_dist = math.hypot(pose[0] - k_pose[0], pose[1] - k_pose[1])
                k_rot = abs(wrap_angle(pose[2] - k_pose[2]))
                add_keyframe = (k_dist >= args.keyframe_trans) or (k_rot >= keyframe_rot)

            if add_keyframe:
                pts_global = transform_points(local_pts, pose)
                pts_global = voxel_downsample(pts_global, args.submap_voxel)
                keyframes.append((pose, pts_global))
                submap_points = rebuild_submap(keyframes, args.submap_voxel)

            last_local_scan = local_pts
            scan_idx += 1

            if viewer is not None and (scan_idx % max(1, args.gui_every) == 0):
                viewer.update(
                    prob_map=grid.getProbabilityMap(),
                    traj_x=traj_x,
                    traj_y=traj_y,
                    pose=pose,
                    local_pts=local_pts,
                    rmse=last_rmse,
                    pairs=last_pairs,
                )
                if not viewer.is_open():
                    print("[INFO] GUI closed, stopping SLAM loop")
                    break

            now = time.time()
            if now - last_log_ts > 1.0:
                last_log_ts = now
                print(
                    f"[SLAM] scan={scan_idx:5d} points={local_pts.shape[0]:4d} "
                    f"pose=({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.1f} deg) "
                    f"submap={submap_points.shape[0]:5d} rmse={last_rmse:.3f}"
                )

            if args.max_scans > 0 and scan_idx >= args.max_scans:
                print(f"[INFO] reached --max-scans={args.max_scans}")
                break

    except KeyboardInterrupt:
        print("\n[INFO] interrupted by user")
    except LidarError as exc:
        print(f"[ERROR] lidar failure: {exc}")
    finally:
        try:
            lidar.stopScan()
        except Exception:
            pass
        try:
            lidar.disconnect()
        except Exception:
            pass

    grid.clampLogOdds()
    prob = grid.getProbabilityMap()
    save_outputs(prob, poses, grid_bounds, args.map_out, args.traj_out)
    print(f"[INFO] map saved to: {args.map_out}")
    print(f"[INFO] trajectory saved to: {args.traj_out}")

    if viewer is not None and viewer.is_open():
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    run()
