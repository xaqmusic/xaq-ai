"""Tests for dash_run — the robot-run sequence behind picrawler_dash's "run config".

Run:  .venv/bin/python -m pytest pi_host/tools/tests/test_dash_run.py -q
A fake robot records every call, so the ORDER of what reaches the robot is checked, and the
property that matters most: nothing is sent before the countdown ends.
"""
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import dash_run  # noqa: E402

COMMANDING = {"pose.set", "mode.set", "resume", "limp", "stop", "systemctl", "spawn"}


class FakeRpc:
    def __init__(self, robot, name):
        self.robot, self.name = robot, name

    def call(self, verb, **kw):
        return self.robot.handle(self.name, verb, kw)

    def close(self):
        pass


class FakeProc:
    def __init__(self, log_path, die=False):
        self.returncode = None
        log_path.write_text("ogma_host: brain inputs ON\n" + ("" if die else "ogma_host: ACTUATION ON — publishing\n"))
        if die:
            self.returncode = 3

    def poll(self):
        return self.returncode

    def send_signal(self, sig):
        self.returncode = -sig

    def wait(self, timeout=None):
        return self.returncode

    def kill(self):
        self.returncode = -9


class FakeRobot:
    """Enough of benchd + ctl + systemd to drive the controller."""

    def __init__(self, tmp, up_y=1.0, imu_ok=True, ctl_ok=True, host_dies=False):
        self.calls = []
        self.lock = threading.Lock()
        self.mode, self.stopped = "bench", False
        self.pose_ticks = 0
        self.up_y, self.imu_ok, self.ctl_ok, self.host_dies = up_y, imu_ok, ctl_ok, host_dies
        self.tmp = tmp
        self.bench = FakeRpc(self, "bench")
        self.ctl = FakeRpc(self, "ctl")

    def log(self, what):
        with self.lock:
            self.calls.append(what)

    def handle(self, sock, verb, kw):
        if verb not in ("status", "ping", "pose.list", "pose.get", "mode.get"):
            self.log(f"{verb}" + (f"={kw.get('mode')}" if verb == "mode.set" else ""))
        if sock == "ctl":
            if not self.ctl_ok:
                return None
            if verb == "mode.get":
                return {"ok": True, "mode": self.mode}
            if verb == "mode.set":
                self.mode = kw["mode"]; self.stopped = kw["mode"] != "bench"
                return {"ok": True}
            if verb == "stop":
                self.stopped = True; return {"ok": True}
            if verb == "resume":
                self.stopped = False; return {"ok": True}
        if verb == "status":
            moving = self.pose_ticks > 0
            if moving:
                self.pose_ticks -= 1
            return {"ok": True, "mode": self.mode, "stopped": self.stopped, "brain": {"frames": 0},
                    "imu": {"ok": self.imu_ok, "up_fused": [0, self.up_y, 0]} if self.imu_ok else None,
                    "vbat": 7.8, "low_battery": False, "servo_lag_alpha": 0.0,
                    "tof": {"m": 0.03, "valid": True}, "pose_move_active": moving, "rescue_active": False}
        if verb == "ping":
            return {"ok": True}
        if verb == "pose.list":
            return {"ok": True, "poses": ["rescue", "stand", "X"]}
        if verb == "pose.get":
            return {"ok": True, "us": [1500] * 12}
        if verb == "pose.set":
            self.pose_ticks = 3
            return {"ok": True, "eta_ms": 100}
        if verb == "limp":
            return {"ok": True}
        return {"ok": True}

    # RobotIo surface
    def systemctl(self, action, unit):
        self.log(f"systemctl {action} {unit}"); return True

    def unit_active(self, unit):
        return True

    def can_sudo(self):
        return True

    def host_exists(self):
        return True

    def spawn_host(self, cfg, log_path):
        self.log("spawn")
        log_path = self.tmp / log_path.name
        self.spawned_log = log_path
        return FakeProc(log_path, die=self.host_dies)

    def now(self):
        return time.monotonic()

    def sleep(self, s):
        time.sleep(min(s, 0.01))


def cfg():
    cs = dash_run.list_configs()
    return next(c for c in cs if c.file.endswith("__fsrleg__honest__nohomeo.json"))


def make(tmp_path, monkeypatch, **kw):
    monkeypatch.setattr(dash_run, "LOG_DIR", tmp_path)
    robot = FakeRobot(tmp_path, **kw)
    ctrl = dash_run.RunController(robot, cfg(), "stand", countdown_s=0.3, ready_extra_s=0.0)
    return robot, ctrl


def wait_phase(ctrl, phases, t=5.0):
    t_end = time.time() + t
    while time.time() < t_end:
        if ctrl.st.phase in phases:
            return True
        time.sleep(0.01)
    return False


def test_allowlist_is_the_launchers_and_robot_faithful_configs_come_first():
    cs = dash_run.list_configs()
    assert len(cs) >= 5
    assert any(c.file.endswith("__fsrleg__honest__nohomeo.json") and c.faithful for c in cs)
    flags = [c.faithful for c in cs]
    assert flags == sorted(flags, reverse=True)          # all faithful before any sim-input


def test_abort_during_the_countdown_sends_nothing_to_the_robot(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.countdown_s = 5.0
    ctrl.start()
    assert wait_phase(ctrl, {"countdown"})
    time.sleep(0.1)
    ctrl.abort()
    ctrl.join(3)
    assert ctrl.st.phase == "aborted"
    assert robot.calls == []


def test_the_brain_starts_only_after_dev_and_the_end_sequence_is_ordered(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.start()
    assert wait_phase(ctrl, {"running"}), ctrl.st.events
    ctrl.end()
    ctrl.join(10)
    c = robot.calls
    order = ["systemctl stop ogma-host", "pose.set", "mode.set=dev", "spawn", "resume"]
    idx = [c.index(x) for x in order]
    assert idx == sorted(idx), c
    # No bench pre-roll: the brain process exists only once benchd is in dev (STOPPED).
    assert c.index("spawn") > c.index("mode.set=dev")
    end = c[c.index("resume") + 1:]
    eorder = ["stop", "mode.set=bench", "limp", "systemctl start ogma-host"]
    eidx = [end.index(x) for x in eorder]
    assert eidx == sorted(eidx), end
    assert ctrl.st.phase == "done"


def test_the_tilt_guard_stops_the_robot_past_80_degrees(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.start()
    assert wait_phase(ctrl, {"running"})
    n_stop = robot.calls.count("stop")
    robot.up_y = 0.15                                     # ~81 deg
    t_end = time.time() + 3
    while time.time() < t_end and robot.calls.count("stop") == n_stop:
        time.sleep(0.01)
    assert robot.calls.count("stop") > n_stop
    assert "tilt guard" in ctrl.st.detail
    ctrl.end(); ctrl.join(10)


def test_79_degrees_does_not_trip_the_guard(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.start()
    assert wait_phase(ctrl, {"running"})
    robot.up_y = 0.19                                     # ~79 deg
    n_stop = robot.calls.count("stop")
    time.sleep(0.5)
    assert robot.calls.count("stop") == n_stop
    ctrl.end(); ctrl.join(10)


def test_a_failed_preflight_moves_nothing(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch, ctl_ok=False)   # not on the robot
    ctrl.start(); ctrl.join(5)
    assert ctrl.st.phase == "aborted"
    assert not any(x.startswith(("pose.set", "mode.set", "spawn", "systemctl")) for x in robot.calls)
    assert "preflight" in ctrl.st.detail


def test_no_attitude_refuses_to_start(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch, imu_ok=False)
    ctrl.start(); ctrl.join(5)
    assert ctrl.st.phase == "aborted"
    assert "pose.set" not in robot.calls


def test_a_brain_that_dies_at_start_is_cleaned_up(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch, host_dies=True)
    ctrl.start(); ctrl.join(10)
    assert ctrl.st.phase == "done"
    assert "resume" not in robot.calls                    # it never got the servos
    after = robot.calls[robot.calls.index("spawn"):]
    assert "mode.set=bench" in after and "limp" in after and "systemctl start ogma-host" in after
