import argparse
import json
import queue
import threading
import time

try:
    import requests
except Exception as exc:
    raise SystemExit(
        "Missing dependency 'requests'. Install with: pip install requests"
    ) from exc

from lidarLib import Lidar, LidarError


class ScanSender(threading.Thread):
    def __init__(self, endpoint, timeout_s, verify_tls, headers, q, stop_evt):
        super().__init__(daemon=True)
        self.endpoint = endpoint
        self.timeout_s = float(timeout_s)
        self.verify_tls = bool(verify_tls)
        self.headers = headers
        self.q = q
        self.stop_evt = stop_evt
        self.session = requests.Session()
        self.sent_ok = 0
        self.sent_err = 0

    def run(self):
        while not self.stop_evt.is_set():
            try:
                payload = self.q.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                res = self.session.post(
                    self.endpoint,
                    data=json.dumps(payload),
                    headers=self.headers,
                    timeout=self.timeout_s,
                    verify=self.verify_tls,
                )
                if 200 <= res.status_code < 300:
                    self.sent_ok += 1
                else:
                    self.sent_err += 1
                    print(f"[HTTP] status={res.status_code} body={res.text[:120]}")
            except requests.RequestException as exc:
                self.sent_err += 1
                print(f"[HTTP] send error: {exc}")
            finally:
                self.q.task_done()


def parse_args():
    ap = argparse.ArgumentParser(
        description="Stream RPLidar scans to a remote HTTP endpoint (headless, no GUI)."
    )
    ap.add_argument("--endpoint", required=True, help="HTTP endpoint that receives LiDAR JSON")
    ap.add_argument("--lidar-port", default="/dev/ttyUSB0")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=1.0, help="Serial timeout [s]")
    ap.add_argument("--motor-pwm", type=int, default=660, help="RPLidar motor PWM")

    ap.add_argument("--request-timeout", type=float, default=1.2, help="HTTP timeout [s]")
    ap.add_argument("--insecure", action="store_true", help="Disable TLS certificate validation")
    ap.add_argument("--auth-token", default="", help="Optional bearer token")

    ap.add_argument("--min-points", type=int, default=60, help="Minimum points for one scan revolution")
    ap.add_argument("--decimate", type=int, default=1, help="Send 1 point every N")
    ap.add_argument("--max-range-mm", type=float, default=8000.0, help="Drop points beyond this range")
    ap.add_argument("--min-range-mm", type=float, default=100.0, help="Drop points below this range")
    ap.add_argument("--queue-size", type=int, default=8, help="Outgoing scan queue size")
    ap.add_argument("--print-every", type=int, default=20, help="Log every N scans")
    return ap.parse_args()


def main():
    args = parse_args()

    verify_tls = not args.insecure
    headers = {"Content-Type": "application/json"}
    if args.auth_token:
        headers["Authorization"] = f"Bearer {args.auth_token}"

    q = queue.Queue(maxsize=max(1, int(args.queue_size)))
    stop_evt = threading.Event()
    sender = ScanSender(
        endpoint=args.endpoint,
        timeout_s=args.request_timeout,
        verify_tls=verify_tls,
        headers=headers,
        q=q,
        stop_evt=stop_evt,
    )
    sender.start()

    lidar = Lidar(
        args.lidar_port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        motor_pwm=args.motor_pwm,
    )

    dec = max(1, int(args.decimate))
    min_mm = float(args.min_range_mm)
    max_mm = float(args.max_range_mm)
    scan_idx = 0
    dropped = 0
    start_ts = time.monotonic()

    print(f"[INFO] endpoint: {args.endpoint}")
    print(f"[INFO] LiDAR: {args.lidar_port} @ {args.baudrate}")

    try:
        lidar.connect()
        info = lidar.getInfo()
        health = lidar.getHealth()
        print("[LIDAR] INFO:", info)
        print("[LIDAR] HEALTH:", health)
        lidar.startScan()

        for scan in lidar.iterScan(min_points=args.min_points):
            scan_idx += 1
            ts_ms = int(time.time() * 1000.0)

            points = []
            for i, (quality, angle_deg, dist_mm) in enumerate(scan):
                if (i % dec) != 0:
                    continue
                if dist_mm < min_mm or dist_mm > max_mm:
                    continue
                points.append(
                    {"q": int(quality), "a_deg": float(angle_deg), "d_mm": float(dist_mm)}
                )

            payload = {
                "ts_ms": ts_ms,
                "scan_id": scan_idx,
                "device": {
                    "model": info.get("model"),
                    "firmware": info.get("firmware"),
                    "hardware": info.get("hardware"),
                    "serial": info.get("serial"),
                },
                "health": {"status": health[0], "error": int(health[1])},
                "points": points,
            }

            try:
                q.put_nowait(payload)
            except queue.Full:
                dropped += 1
                try:
                    _ = q.get_nowait()
                    q.task_done()
                except queue.Empty:
                    pass
                try:
                    q.put_nowait(payload)
                except queue.Full:
                    dropped += 1

            if args.print_every > 0 and (scan_idx % args.print_every == 0):
                dt = max(1e-6, time.monotonic() - start_ts)
                hz = scan_idx / dt
                print(
                    f"[STREAM] scans={scan_idx} rate={hz:.2f}Hz "
                    f"queue={q.qsize()}/{q.maxsize} sent_ok={sender.sent_ok} "
                    f"sent_err={sender.sent_err} dropped={dropped} points={len(points)}"
                )

    except KeyboardInterrupt:
        print("\n[INFO] interrupted")
    except LidarError as exc:
        print(f"[ERROR] lidar: {exc}")
    finally:
        stop_evt.set()
        try:
            lidar.stopScan()
        except Exception:
            pass
        try:
            lidar.disconnect()
        except Exception:
            pass

        # give sender a moment to flush latest payloads
        t0 = time.monotonic()
        while not q.empty() and (time.monotonic() - t0) < 1.2:
            time.sleep(0.05)

        print(
            f"[INFO] done scans={scan_idx} sent_ok={sender.sent_ok} "
            f"sent_err={sender.sent_err} dropped={dropped}"
        )


if __name__ == "__main__":
    main()
