"""Swing-only velocity client for the commissioned cartpole ROS node."""
import argparse
import json
import math
import signal
import time
from dataclasses import asdict
from pathlib import Path

from profile_core import validate_reference
from velocity_swing import MAX_GAP_NS, Settings, Swing


def healthy(status, q, now, status_at):
    if not 0 <= now - status_at <= 600_000_000:
        raise RuntimeError("Stale hardware status")
    if (not status["homed"] or not status["pole_calibrated"] or not status["motor_available"]
            or not status["sensor_fresh"] or status["fault"] or status["sensor_error"]
            or status["limit_latched"] or status["left_blocked"] or status["right_blocked"]):
        raise RuntimeError(f"Hardware interlock: {status}")
    age = (now - q["host_monotonic_ns"]) / 1e9
    if not 0 <= age <= MAX_GAP_NS / 1e9 or not 0 <= q["sensor_age_s"] + age <= 0.10:
        raise RuntimeError("Stale telemetry or pole sensor")
    if (q["fault"] or not q["homed"] or not q["pole_calibrated"] or not q["motor_available"]
            or not all(math.isfinite(q[key]) for key in
                       ("cart_m", "cart_m_s", "pole_rad", "pole_rad_s", "command_m_s"))):
        raise RuntimeError("Invalid telemetry or hardware state")


def ready_to_start(status, q, ref):
    return (status["mode"] == q["mode"] == "disabled" and not status["enabled"]
            and status["stop_confirmed"] is True and abs(q["command_m_s"]) < 1e-9
            and abs(q["cart_m"] - ref["cart_m"]) <= 0.0003 and abs(q["cart_m_s"]) <= 0.001
            and abs(abs(q["pole_rad"]) - math.pi) <= math.radians(5)
            and abs(q["pole_rad_s"]) <= 0.5)


class Hardware:
    """ROS is the only serial owner. Service calls and telemetry waits have deadlines."""
    def __init__(self, node):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float64, String
        from std_srvs.srv import SetBool, Trigger

        self.ros, self.node, self.message = rclpy, node, Float64
        self.status = self.q = None
        self.status_at = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(String, "/cartpole/telemetry", self.telemetry, qos)
        node.create_subscription(String, "/cartpole/status", self.on_status, 1)
        self.pub = node.create_publisher(Float64, "/cartpole/cmd_velocity", 1)
        self.services = {
            "enable": (node.create_client(SetBool, "/cartpole/enable"), SetBool.Request(data=True)),
            "stop": (node.create_client(Trigger, "/cartpole/stop"), Trigger.Request()),
        }

    def telemetry(self, msg):
        self.q = json.loads(msg.data)

    def on_status(self, msg):
        self.status = json.loads(msg.data)
        self.status_at = time.monotonic_ns()

    def spin(self):
        self.ros.spin_once(self.node, timeout_sec=0.01)

    def call(self, name):
        client, request = self.services[name]
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError(f"{name} service unavailable")
        future = client.call_async(request)
        self.ros.spin_until_future_complete(self.node, future, timeout_sec=3)
        if not future.done():
            raise RuntimeError(f"{name} service timed out")
        reply = future.result()
        if reply is None or not reply.success:
            raise RuntimeError(f"{name} failed: {getattr(reply, 'message', 'no response')}")

    def send(self, speed):
        self.pub.publish(self.message(data=speed))

    def preflight(self, reference):
        for name, (client, _) in self.services.items():
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError(f"ROS service /cartpole/{name} unavailable after 10 s")
        start = time.monotonic_ns()
        publishers = 0
        since = None
        while time.monotonic_ns() - start < 5_000_000_000:
            self.spin()
            if self.status is None or self.q is None or not self.pub.get_subscription_count():
                continue
            now = time.monotonic_ns()
            healthy(self.status, self.q, now, self.status_at)
            publishers = self.node.count_publishers("/cartpole/cmd_velocity")
            if publishers != 1:
                since = None  # DDS discovery may still show the previous preflight process.
                continue
            if not ready_to_start(self.status, self.q, reference):
                since = None
                continue
            if since is None:
                since = now
            if now - since >= 500_000_000:
                return
        if publishers > 1:
            raise RuntimeError("Another cmd_velocity publisher is present")
        raise RuntimeError("Preflight needs fresh disabled, stopped, centred, pole-down telemetry")

    def run(self, cfg, summary, events):
        start = time.monotonic_ns()
        deadline = start + round(cfg.duration * 1e9)
        controller = Swing(self.q["cart_m"], start, cfg)
        summary["started_ns"] = start
        accepted_manual = False
        last_publisher_check = start
        try:
            while time.monotonic_ns() < deadline:
                self.spin()
                now = time.monotonic_ns()
                if now >= deadline:
                    break
                q = self.q
                healthy(self.status, q, now, self.status_at)
                # Discard disabled telemetry queued before the enable response.
                if q["host_monotonic_ns"] < start:
                    continue
                if q["mode"] != "manual":
                    raise RuntimeError(f"Velocity drive left manual mode: {q['mode']}")
                accepted_manual = True
                if q["host_monotonic_ns"] == controller.previous_stamp:
                    continue
                if now - last_publisher_check >= 200_000_000:
                    if self.node.count_publishers("/cartpole/cmd_velocity") != 1:
                        raise RuntimeError("Another cmd_velocity publisher appeared")
                    last_publisher_check = now
                speed, decision = controller.update(now, q)
                if decision:
                    events.append(decision)
                self.send(speed)
                events.append(dict(kind="velocity_command", host_monotonic_ns=now,
                                   feedback_ns=q["host_monotonic_ns"], requested_m_s=speed,
                                   applied_m_s=q["command_m_s"], measured_m_s=q["cart_m_s"],
                                   sensor_age_s=q["sensor_age_s"], stroke_end_m=controller.endpoint,
                                   cart_m=q["cart_m"], estimated_cart_m=controller.estimated_x, pole_rad=q["pole_rad"], pole_rad_s=q["pole_rad_s"]))
                if controller.outcome:
                    break
            if not accepted_manual:
                raise RuntimeError("No manual-mode telemetry within the trial")
            summary["outcome"] = controller.outcome or "bounded_time_limit"
        finally:
            summary.update(upright_entered=controller.entry is not None, entry_estimate=controller.entry,
                           closest_upright_deg=math.degrees(controller.closest), strokes=controller.strokes,
                           duration_s=(time.monotonic_ns() - start) / 1e9)


def trial(hw, cfg, execute, ref, boot, summary, events):
    validate_reference(ref, time.monotonic_ns(), boot)
    hw.preflight(ref)
    validate_reference(ref, time.monotonic_ns(), boot)
    if not execute:
        summary.update(completed=True, outcome="preflight_passed_no_enable")
        return
    # A lost enable reply can follow an actual enable. Always request stop from this point.
    summary["motion_attempted"] = True
    try:
        hw.call("enable")
        hw.run(cfg, summary, events)
    finally:
        try:
            hw.send(0.0)
        except (Exception, KeyboardInterrupt) as exc:
            summary["zero_publish_error"] = str(exc) or "interrupted"
        try:
            hw.call("stop")  # success means the node verified motor feedback, then disabled
            summary["stop_confirmed"] = True
        except (Exception, KeyboardInterrupt) as exc:
            summary.update(stop_confirmed=False, stop_error=str(exc) or "interrupted")
            print("STOP UNCONFIRMED: use the physical cutoff.", flush=True)
            raise
    summary["completed"] = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--pole-down", action="store_true")
    for key, default in asdict(Settings()).items():
        parser.add_argument("--" + key, type=float, default=default)
    args = parser.parse_args()
    try:
        cfg = Settings(**{key: getattr(args, key) for key in asdict(Settings())})
    except ValueError as exc:
        parser.error(str(exc))
    if not (args.check_only or args.execute):
        print(json.dumps(dict(mode="offline_preview", **asdict(cfg))))
        return 0
    if not args.pole_down:
        parser.error("--pole-down is required with --execute or --check-only")
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    def interrupted(*_):
        raise KeyboardInterrupt

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    node = rclpy.create_node("cartpole_velocity_swing")
    path = Path("/logs") / f"velocity_final_push_{time.time_ns()}.json"
    summary = dict(completed=False, motion_attempted=False, stop_confirmed=None,
                   control_mode="velocity_final_push_swing_only", outcome="error", settings=asdict(cfg))
    events = []
    try:
        # Verify the log mount is writable before enabling the drive.
        with path.open("x") as stream:
            json.dump({"summary": summary, "events": events}, stream)
        ref = json.loads(path.with_name("position_reference.json").read_text())
        boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        summary["reference"] = ref
        trial(Hardware(node), cfg, args.execute, ref, boot, summary, events)
    except (Exception, KeyboardInterrupt) as exc:
        summary.update(completed=False, error=str(exc) or "Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(dict(summary=summary, events=events), allow_nan=False))
    tmp.replace(path)
    print("RECORDING:", path, flush=True)
    print("SUMMARY:", json.dumps(summary, indent=2), flush=True)
    print(("TEST " if args.execute else "PREFLIGHT ") + ("PASS" if summary["completed"] else "FAIL"), flush=True)
    return 0 if summary["completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
