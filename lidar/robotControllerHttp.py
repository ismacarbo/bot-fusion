import json
import queue
import threading
import time

try:
    import requests
except Exception:
    requests = None

from robotController import RobotController, build_parser


class HttpSender(threading.Thread):
    def __init__(self, endpoint, timeout_s, verify_tls, headers, out_q, stop_evt):
        super().__init__(daemon=True)
        self.endpoint = endpoint
        self.timeout_s = float(timeout_s)
        self.verify_tls = bool(verify_tls)
        self.headers = headers
        self.out_q = out_q
        self.stop_evt = stop_evt
        self.session = requests.Session()
        self.sent_ok = 0
        self.sent_err = 0

    def run(self):
        while not self.stop_evt.is_set() or not self.out_q.empty():
            try:
                payload = self.out_q.get(timeout=0.2)
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
                self.out_q.task_done()


class RobotControllerHttp(RobotController):
    def __init__(self, args):
        super().__init__(args)
        self.http_enabled = bool(args.endpoint)
        self.http_every = max(1, int(args.http_every))
        self.http_decimate = max(1, int(args.http_decimate))
        self.http_dropped = 0
        self.http_last_log = time.monotonic()

        self.http_q = None
        self.http_stop_evt = None
        self.http_sender = None
        self.endpoint = args.endpoint

        if not self.http_enabled:
            print("[HTTP] disabled (no --endpoint provided)")
            return

        if requests is None:
            raise SystemExit("Missing dependency 'requests'. Install with: pip install requests")

        headers = {"Content-Type": "application/json"}
        if args.auth_token:
            headers["Authorization"] = f"Bearer {args.auth_token}"

        self.http_q = queue.Queue(maxsize=max(1, int(args.http_queue_size)))
        self.http_stop_evt = threading.Event()
        self.http_sender = HttpSender(
            endpoint=args.endpoint,
            timeout_s=args.http_timeout,
            verify_tls=(not args.insecure),
            headers=headers,
            out_q=self.http_q,
            stop_evt=self.http_stop_evt,
        )
        self.http_sender.start()
        print(f"[HTTP] streaming enabled -> {args.endpoint}")

    def _build_http_payload(self, raw_scan, local_pts, telem, cmd, front, left, right, scan_idx):
        points = []
        for i, (q, a_deg, d_mm) in enumerate(raw_scan):
            if (i % self.http_decimate) != 0:
                continue
            points.append({"q": int(q), "a_deg": float(a_deg), "d_mm": float(d_mm)})

        payload = {
            "ts_ms": int(time.time() * 1000.0),
            "scan_id": int(scan_idx),
            "pose": {
                "x_m": float(self.pose[0]),
                "y_m": float(self.pose[1]),
                "theta_rad": float(self.pose[2]),
            },
            "control": {
                "cmd_pwm_l": int(cmd[0]),
                "cmd_pwm_r": int(cmd[1]),
                "front_m": float(front),
                "left_m": float(left),
                "right_m": float(right),
            },
            "icp": {
                "rmse_m": float(self.last_rmse) if self.last_rmse != float("inf") else None,
                "pairs": int(self.last_pairs),
                "local_points": int(local_pts.shape[0]),
            },
            "telemetry": None if telem is None else {
                "ms": int(telem.get("ms", 0)),
                "yaw_deg": float(telem.get("yaw_deg", 0.0)),
                "yaw_rate_dps": float(telem.get("yaw_rate_dps", 0.0)),
                "pwmL": int(telem.get("pwmL", 0)),
                "pwmR": int(telem.get("pwmR", 0)),
                "distC_cm": int(telem.get("distC_cm", -1)),
                "irL_raw": int(telem.get("irL_raw", 0)),
                "irR_raw": int(telem.get("irR_raw", 0)),
            },
            "points": points,
        }
        return payload

    def _on_post_scan(self, raw_scan, local_pts, telem, cmd, front, left, right, scan_idx):
        if not self.http_enabled:
            return
        if (scan_idx % self.http_every) != 0:
            return

        payload = self._build_http_payload(raw_scan, local_pts, telem, cmd, front, left, right, scan_idx)

        try:
            self.http_q.put_nowait(payload)
        except queue.Full:
            self.http_dropped += 1
            try:
                _ = self.http_q.get_nowait()
                self.http_q.task_done()
            except queue.Empty:
                pass
            try:
                self.http_q.put_nowait(payload)
            except queue.Full:
                self.http_dropped += 1

        now = time.monotonic()
        if now - self.http_last_log > 1.5:
            self.http_last_log = now
            print(
                f"[HTTP] queue={self.http_q.qsize()}/{self.http_q.maxsize} "
                f"sent_ok={self.http_sender.sent_ok} sent_err={self.http_sender.sent_err} "
                f"dropped={self.http_dropped}"
            )

    def _on_shutdown(self, scan_idx):
        if not self.http_enabled:
            return
        self.http_stop_evt.set()

        t0 = time.monotonic()
        while not self.http_q.empty() and (time.monotonic() - t0) < 1.5:
            time.sleep(0.05)

        print(
            f"[HTTP] done scans={scan_idx} sent_ok={self.http_sender.sent_ok} "
            f"sent_err={self.http_sender.sent_err} dropped={self.http_dropped}"
        )


def parse_args():
    parser = build_parser()
    parser.description = (
        "Autonomous robot controller with LiDAR+gyro fusion and HTTP scan streaming"
    )
    parser.add_argument("--endpoint", default="", help="HTTP endpoint to stream runtime data")
    parser.add_argument("--auth-token", default="", help="Bearer token for endpoint authentication")
    parser.add_argument("--http-timeout", type=float, default=1.2, help="HTTP timeout [s]")
    parser.add_argument("--http-queue-size", type=int, default=8, help="Outgoing HTTP queue size")
    parser.add_argument("--http-every", type=int, default=1, help="Stream every N scans")
    parser.add_argument("--http-decimate", type=int, default=1, help="Stream 1 point every N")
    parser.add_argument("--insecure", action="store_true", help="Disable TLS cert validation")
    return parser.parse_args()


def main():
    args = parse_args()
    ctrl = RobotControllerHttp(args)
    ctrl.run()


if __name__ == "__main__":
    main()
