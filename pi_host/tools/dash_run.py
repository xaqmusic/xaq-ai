"""dash_run — run a brain config on the robot from picrawler_dash (ON THE PI only).

The sequence, and why each step is where it is:

  pick     a config from the Godot launcher's own allowlist (launcher.gd), tagged
           ROBOT-FAITHFUL when its body_env says the sim fed it only inputs the robot can
           publish (HONEST_JOINTS).  Anything else runs partly blind on hardware.
  confirm  preflight checks, start pose, the servo lag benchd will apply.
  COUNTDOWN 10 s, abortable by any key.  NOTHING is sent to the robot before it ends: the
           countdown is the operator's chance to make sure the robot is placed and clear.
  prepare  stop the senses-only ogma-host service (it holds the inspector port) -> pose the
           robot in bench mode, pinging so the bench deadman stays quiet -> benchd to `dev`
           (which latches STOP) -> start ogma_host --actuate.  ⚠ ogma_host starts AFTER the
           mode change, so the brain is paused from its first tick: brain_run/arm.sh started
           it in bench mode, where it ticked ~12 s with its commands ignored.
  run      resume.  A tilt guard (80 deg, the operator's limit) STOPs the robot; benchd
           itself STOPs on a lost brain stream (dev mode) and handles low battery.  SPACE is
           the dashboard's STOP/resume, on its own socket, independent of this thread.
  reset    R: stop -> benchd `pose.recall` of the start pose (control socket, STOPPED only).
           The body goes home; the brain stays paused, not reset, and SPACE resumes it once
           the pose has landed — the robot version of the sim's body reset after a fall.
  end      stop -> SIGTERM ogma_host -> bench mode -> rescue pose -> restart ogma-host.

Port doc SPEC §1.1, as amended by the operator 2026-10-03: the brain may be started from
this Pi-local console, through benchd's LOOPBACK control socket.  Nothing on the network,
the laptop's Godot dashboard included, has a path to start a brain or set the run mode —
and this module refuses to run unless the control socket answers on 127.0.0.1.
"""
from __future__ import annotations

import json
import math
import os
import re
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

try:
    import zmq
except ImportError:  # the monitoring dash degrades without it; running needs it
    zmq = None

REPO = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO / "godot_host/project/addons/ami_ogma/configs"
LAUNCHER_GD = REPO / "godot_host/project/scripts/launcher.gd"
OGMA_HOST = REPO / "pi_host/build/ogma_host"
LOG_DIR = REPO / "pi_host/log"

COUNTDOWN_S = 10.0
TILT_LIMIT_DEG = 80.0                         # operator, 2026-10-03: 60 was too tight
TILT_LIMIT_UP_Y = math.cos(math.radians(TILT_LIMIT_DEG))
BENCH_PORT, CTL_PORT, CMD_PORT = 5590, 5593, 5594
VBAT_WARN = 7.0                               # limp is 6.4 sustained; belly-up draws ~1.5 A


# --------------------------------------------------------------------------- configs

@dataclass
class ConfigEntry:
    file: str
    path: Path
    name: str
    faithful: bool
    phase: str = ""


def allowlist(launcher_gd: Path = LAUNCHER_GD) -> list[str]:
    """The picrawler allowlist exactly as the Godot launcher shows it."""
    files, inside = [], False
    for line in launcher_gd.read_text().splitlines():
        if "_PICRAWLER_CONFIG_ALLOWLIST" in line and "[" in line:
            inside = True
            continue
        if inside:
            if line.strip().startswith("]"):
                break
            m = re.match(r'\s*"([^"]+\.json)"', line)
            if m:
                files.append(m.group(1))
    return files


def list_configs(config_dir: Path = CONFIG_DIR, launcher_gd: Path = LAUNCHER_GD) -> list[ConfigEntry]:
    out = []
    for f in allowlist(launcher_gd):
        p = config_dir / f
        try:
            md = json.loads(p.read_text()).get("metadata", {})
        except (OSError, ValueError):
            continue                                    # listed but absent: not offered
        be = md.get("body_env") or {}
        out.append(ConfigEntry(file=f, path=p, name=str(md.get("name", f)),
                               faithful=str(be.get("OGMA_PICRAWLER_HONEST_JOINTS", "")) == "1",
                               phase=str(md.get("phase_tag", ""))))
    # Robot-faithful first; otherwise the launcher's own order.
    return sorted(out, key=lambda c: not c.faithful)


# --------------------------------------------------------------------------- I/O

class Rpc:
    """One ZMQ REQ socket with a timeout; recreated after any failure (a REQ that missed
    its reply is stuck by protocol).  Returns None instead of raising."""

    def __init__(self, port: int, host: str = "127.0.0.1", timeout_ms: int = 1000):
        self.port, self.host, self.timeout_ms = port, host, timeout_ms
        self._ctx = zmq.Context.instance() if zmq else None
        self._s = None

    def call(self, verb: str, **kw: Any) -> Optional[dict]:
        if not zmq:
            return None
        try:
            if self._s is None:
                s = self._ctx.socket(zmq.REQ)
                s.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
                s.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
                s.setsockopt(zmq.LINGER, 0)
                s.connect(f"tcp://{self.host}:{self.port}")
                self._s = s
            self._s.send_string(json.dumps({"verb": verb, **kw}))
            return json.loads(self._s.recv_string())
        except Exception:
            self.close()
            return None

    def close(self) -> None:
        if self._s is not None:
            self._s.close(0)
            self._s = None


class RobotIo:
    """Everything the controller does to the world.  Tests substitute a fake."""

    def __init__(self) -> None:
        self.bench = Rpc(BENCH_PORT)
        self.ctl = Rpc(CTL_PORT)            # loopback-only by benchd's bind address

    def new_ctl(self) -> "Rpc":
        """A second control socket, for the UI thread (REQ sockets are single-threaded)."""
        return Rpc(CTL_PORT)

    def systemctl(self, action: str, unit: str) -> bool:
        r = subprocess.run(["sudo", "-n", "systemctl", action, unit],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
        return r.returncode == 0

    def unit_active(self, unit: str) -> bool:
        r = subprocess.run(["systemctl", "is-active", "--quiet", unit], timeout=10)
        return r.returncode == 0

    def can_sudo(self) -> bool:
        return subprocess.run(["sudo", "-n", "true"], stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL, timeout=10).returncode == 0

    def host_exists(self) -> bool:
        return OGMA_HOST.exists()

    def spawn_host(self, cfg: Path, log_path: Path):
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        lf = open(log_path, "w")
        # Own session: Ctrl-C in the dashboard must not reach the brain mid-run; the end
        # sequence stops it deliberately.
        return subprocess.Popen(
            [str(OGMA_HOST), "--config", str(cfg), "--imu", "--brain-inputs",
             "--actuate", f"tcp://127.0.0.1:{CMD_PORT}", "--listen", "0.0.0.0", "--rt",
             "--dump-inputs", "50"],
            cwd=str(REPO), stdout=lf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
            start_new_session=True)

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, s: float) -> None:
        time.sleep(s)


# --------------------------------------------------------------------------- preflight

@dataclass
class Check:
    ok: bool
    text: str
    blocking: bool = True


def preflight(io, pose: str) -> list[Check]:
    out: list[Check] = []
    ctl = io.ctl.call("mode.get")
    out.append(Check(ctl is not None and ctl.get("ok", False),
                     "benchd control socket answers on 127.0.0.1 (run ON the robot)"
                     if ctl else "no benchd control socket on 127.0.0.1:5593 — run this on the robot, "
                     "with benchd started with --ctl-port (pi_host/systemd/ogma-benchd.service)"))
    st = io.bench.call("status")
    ok = st is not None and st.get("ok", True)
    out.append(Check(ok, "benchd reachable" if ok else "benchd unreachable on :5590"))
    if not ok:
        return out
    out.append(Check(st.get("brain") is not None,
                     "benchd has the brain command path (--cmd-port)" if st.get("brain") is not None
                     else "benchd has NO brain command path — needs --state-pub 5592 --cmd-port 5594 --ctl-port 5593"))
    out.append(Check(st.get("mode") == "bench",
                     f"benchd in bench mode" if st.get("mode") == "bench"
                     else f"benchd is in '{st.get('mode')}' mode — another run may be live"))
    imu = st.get("imu") or {}
    out.append(Check(bool(imu.get("ok")) and bool(imu.get("up_fused")),
                     "benchd has attitude (the tilt guard can see)" if imu.get("ok")
                     else "benchd has no IMU — the tilt guard would be blind; sudo systemctl restart ogma-benchd"))
    vb = float(st.get("vbat", 0.0))
    out.append(Check(not st.get("low_battery") and vb > 6.4,
                     f"battery {vb:.2f} V" + (" — LOW: belly-up draws ~1.5 A, limp is 6.4 V" if vb < VBAT_WARN else ""),
                     blocking=bool(st.get("low_battery")) or vb <= 6.4))
    lag = st.get("servo_lag_alpha")
    out.append(Check(True, f"servo output lag in brain modes: set by benchd --servo-lag-alpha "
                           f"(frame reports {lag} now; it applies once in dev)", blocking=False))
    poses = (io.bench.call("pose.list") or {}).get("poses", [])
    out.append(Check(pose in poses, f"start pose '{pose}' saved" if pose in poses
                     else f"no saved pose '{pose}'"))
    out.append(Check(io.host_exists(), "ogma_host built" if io.host_exists()
                     else f"{OGMA_HOST} missing — build with PI_HOST_BUILD_BRAIN=ON"))
    out.append(Check(io.can_sudo(), "passwordless sudo (to stop/start the ogma-host service)"
                     if io.can_sudo() else "sudo -n fails — cannot stop the ogma-host service"))
    return out


# --------------------------------------------------------------------------- controller

@dataclass
class RunState:
    phase: str = "idle"           # idle countdown prepare running ending done aborted
    detail: str = ""
    countdown_left: float = 0.0
    started_at: float = 0.0       # monotonic, when the brain got the servos
    events: list = field(default_factory=list)
    log_path: Optional[Path] = None
    belly_mm: Optional[float] = None
    tilt_deg: Optional[float] = None
    vbat: Optional[float] = None
    stopped: Optional[bool] = None


class RunController:
    def __init__(self, io, cfg: ConfigEntry, pose: str = "stand",
                 countdown_s: float = COUNTDOWN_S, ready_extra_s: float = 4.0):
        self.io, self.cfg, self.pose = io, cfg, pose
        self.countdown_s, self.ready_extra_s = countdown_s, ready_extra_s
        self.st = RunState()
        self._abort = threading.Event()        # before the brain has the servos: abort
        self._end = threading.Event()          # operator asked to end the run
        self._host = None
        self._svc_was_active = False
        self._thread: Optional[threading.Thread] = None
        self.resets = 0
        # The end sequence runs exactly once, from whichever thread gets there first: the
        # controller after E, or the dashboard's fallback if the controller does not respond.
        self._cleanup_lock = threading.Lock()
        self._cleanup_started = False
        # ⚠ EVERY STEP IS WRITTEN TO DISK.  The first dash-launched run (2026-10-03) ignored E
        # and nothing recorded what the controller was doing, so it could not be diagnosed.
        self.events_path = LOG_DIR / f"dashrun_{time.strftime('%Y%m%d_%H%M%S')}.events"

    # ---- operator actions (UI thread) ----
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def abort(self) -> None:
        self._abort.set()

    def end(self) -> None:
        self._end.set()
        self._abort.set()
        self._ev("E pressed: end requested")

    def ending_started(self) -> bool:
        return self._cleanup_started

    def end_now(self) -> None:
        """Run the end sequence in the CALLER's thread — the dashboard's fallback when the
        controller has not started it after E.  A no-op if it already ran or is running."""
        self._end.set(); self._abort.set()
        self._ev("end sequence run by the dashboard (controller did not respond)")
        self._cleanup()

    def reset(self) -> bool:
        """R: freeze, then move back to the start pose; stays STOPPED.  Called from the UI
        thread, so it uses its own socket rather than the controller's."""
        if self.st.phase not in ("running", "prepare") or not self.st.started_at:
            self._ev("reset: only once the brain has the servos (E ends a run that has not started)")
            return False
        rpc = self.io.new_ctl()
        rpc.call("stop")
        r = rpc.call("pose.recall", name=self.pose)
        rpc.close()
        if r and r.get("ok"):
            self.resets += 1
            self._ev(f"RESET #{self.resets}: moving to '{self.pose}' — STOPPED, brain paused (not reset). "
                     "SPACE resumes once it lands")
            return True
        self._ev(f"reset refused: {(r or {}).get('error', 'no reply')}")
        return False

    def busy(self) -> bool:
        return self.st.phase in ("countdown", "prepare", "running", "ending")

    def join(self, timeout: float = None) -> None:
        if self._thread:
            self._thread.join(timeout)

    # ---- the sequence (controller thread) ----
    def _ev(self, text: str) -> None:
        self.st.detail = text
        self.st.events.append((round(self.io.now(), 2), text))
        try:
            with open(self.events_path, "a") as f:
                f.write(f"{time.strftime('%H:%M:%S')} [{self.st.phase}] {text}\n")
        except OSError:
            pass                                  # a full disk must not stop a STOP

    def _run(self) -> None:
        try:
            if not self._countdown():
                self.st.phase = "aborted"
                self._ev("aborted during the countdown — nothing was sent to the robot")
                return
            moved = self._prepare()
            if moved is None:                     # aborted before anything moved
                self.st.phase = "aborted"
                return
            if moved:
                self._running()
        except Exception as e:                    # never leave a brain driving on a crash
            self._ev(f"controller error: {e!r}")
        if self.st.phase not in ("aborted",):
            self._cleanup()

    def _countdown(self) -> bool:
        self.st.phase = "countdown"
        t_end = self.io.now() + self.countdown_s
        while True:
            left = t_end - self.io.now()
            self.st.countdown_left = max(0.0, left)
            if self._abort.is_set():
                return False
            if left <= 0:
                return True
            self.io.sleep(0.05)

    def _wait(self, cond: Callable[[], bool], timeout: float, ping: bool = False) -> Optional[bool]:
        """Poll cond until true; None if aborted, False on timeout."""
        t_end = self.io.now() + timeout
        while self.io.now() < t_end:
            if self._abort.is_set():
                return None
            if ping:
                self.io.bench.call("ping")        # bench deadman: a controlling client
            if cond():
                return True
            self.io.sleep(0.2)
        return False

    def _status(self) -> dict:
        return self.io.bench.call("status") or {}

    def _prepare(self) -> Optional[bool]:
        """Returns None if aborted before anything moved, False if the run could not start
        (cleanup still runs), True when the brain has the servos."""
        self.st.phase = "prepare"
        bad = [c for c in preflight(self.io, self.pose) if c.blocking and not c.ok]
        if bad:
            self._ev("preflight failed: " + bad[0].text)
            return None
        if self._abort.is_set():
            return None
        self._svc_was_active = self.io.unit_active("ogma-host")
        if self._svc_was_active:
            self._ev("stopping the senses-only ogma-host service")
            if not self.io.systemctl("stop", "ogma-host"):
                self._ev("could not stop ogma-host — not starting")
                return False
        # ---- pose: the first motion ----
        us = (self.io.bench.call("pose.get", name=self.pose) or {}).get("us")
        r = self.io.bench.call("pose.set", us=us) if us else None
        if not r or not r.get("ok"):
            self._ev(f"pose.set {self.pose} refused: {(r or {}).get('error', 'no reply')}")
            return False
        self._ev(f"moving to '{self.pose}' (staggered, slow)")
        started = self._wait(lambda: bool(self._status().get("pose_move_active")),
                             min(3.0, 1.0 + r.get("eta_ms", 0) / 1000.0), ping=True)
        if started is None:
            return False
        landed = self._wait(lambda: not self._status().get("pose_move_active"), 30.0, ping=True)
        if not landed:
            self._ev("pose did not land" if landed is False else "aborted while posing")
            return False
        # ---- brain mode, latched STOP, then the brain ----
        m = self.io.ctl.call("mode.set", mode="dev")
        if not m or not m.get("ok"):
            self._ev(f"mode dev refused: {(m or {}).get('error', 'no reply')}")
            return False
        self._ev("benchd in dev, STOPPED; starting the brain (paused until resume)")
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.st.log_path = LOG_DIR / f"dashrun_{stamp}_{Path(self.cfg.file).stem[-40:]}.log"
        self._host = self.io.spawn_host(self.cfg.path, self.st.log_path)

        def ready() -> bool:
            if self._host.poll() is not None:
                raise RuntimeError("ogma_host exited during start — see " + str(self.st.log_path))
            try:
                return "ACTUATION ON" in self.st.log_path.read_text(errors="replace")
            except OSError:
                return False
        ok = self._wait(ready, 60.0)
        if not ok:
            self._ev("ogma_host did not come up" if ok is False else "aborted while the brain started")
            return False
        # The IMU's gyro bias seeds from the first still window; the robot is frozen now.
        if self._wait(lambda: False, self.ready_extra_s) is None:
            return False
        r = self.io.ctl.call("resume")
        if not r or not r.get("ok"):
            self._ev(f"resume refused: {(r or {}).get('error', 'no reply')}")
            return False
        self.st.started_at = self.io.now()
        self.st.phase = "running"
        self._ev(f"RUNNING {self.cfg.name[:50]}")
        return True

    def _running(self) -> None:
        while not self._end.is_set():
            f = self._status()
            if f:
                imu = f.get("imu") or {}
                tof = f.get("tof") or {}
                up = imu.get("up_fused")
                self.st.stopped = bool(f.get("stopped"))
                self.st.vbat = f.get("vbat")
                self.st.belly_mm = tof.get("m", 0) * 1000.0 if tof.get("valid") else None
                if not imu.get("ok") or not up:
                    self.st.tilt_deg = None
                    if not f.get("stopped"):
                        self.io.ctl.call("stop")
                        self._ev("attitude lost — STOPPED (the tilt guard cannot see). SPACE resumes, E ends")
                else:
                    self.st.tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, up[1]))))
                    if up[1] < TILT_LIMIT_UP_Y and not f.get("stopped"):
                        self.io.ctl.call("stop")
                        self._ev(f"tilt guard: {self.st.tilt_deg:.0f}° > {TILT_LIMIT_DEG:.0f}° — STOPPED. "
                                 "SPACE resumes, E ends")
            if self._host is not None and self._host.poll() is not None and "brain exited" not in self.st.detail:
                self._ev(f"brain exited (code {self._host.returncode}) — benchd freezes the robot; E ends")
            self.io.sleep(0.2)

    def _cleanup(self) -> None:
        with self._cleanup_lock:
            if self._cleanup_started:
                return
            self._cleanup_started = True
        self.st.phase = "ending"
        self._ev("ending: stop, brain off, bench mode, rescue pose")
        self.io.ctl.call("stop")
        if self._host is not None and self._host.poll() is None:
            try:
                self._host.send_signal(signal.SIGTERM)
                self._host.wait(timeout=8)
            except Exception:
                try:
                    self._host.kill()
                except Exception:
                    pass
        self.io.ctl.call("mode.set", mode="bench")
        r = self.io.bench.call("limp")
        self._ev("rescue pose commanded" if r and r.get("ok") else "rescue pose: no reply")
        self._wait(lambda: not self._status().get("rescue_active"), 20.0, ping=True)
        if self._svc_was_active:
            ok = self.io.systemctl("start", "ogma-host")
            self._ev("ogma-host service restarted" if ok else "⚠ could not restart ogma-host")
        dur = self.io.now() - self.st.started_at if self.st.started_at else 0.0
        self.st.phase = "done"
        self._ev(f"done — brain drove {dur:.0f} s; log {self.st.log_path}")
