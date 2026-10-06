#!/usr/bin/env python3
"""picrawler_dash — the dashboard's numbers, in a terminal on the robot.

For a portable monitor plugged into the PiCrawler, or over ssh.  Shows what the Godot
bench dashboard shows plus each EPM's GNG state, which the Godot side does not have
(that belongs to the inspector).

⚠ IT MUST NOT DISTURB THE INSPECTOR.  Both read the same brain, so this uses only
STATELESS request/reply verbs -- `module_snapshot`, never `module_subscribe_diag`.  A
subscription allocates a sub_id and a topic prefix on the DiagPublisher; a snapshot
allocates nothing, so the two tools cannot interfere no matter the order they start in.
ControlServer already handles each client on its own thread, so a second connection is
expected, not tolerated.

Monitoring by default.  Two ways it can act on the robot:

- SPACE stops the robot, and SPACE again resumes it (benchd's `stop` / `resume`).  STOP
  freezes every servo where it is; it moves nothing.  The STOP request goes on its own
  socket and the screen polls on a background thread, so the key is never stuck behind a
  slow brain query.
- C "run config" (2026-10-03, operator): pick a brain config from the Godot launcher's
  allowlist, confirm, and after a 10 s COUNTDOWN — any key aborts, and nothing reaches the
  robot before it ends — the robot is posed and the brain gets the servos (dash_run.py).
  E ends the run (rescue pose).  This works ONLY on the robot: it needs benchd's
  loopback control socket, so a dash run over ssh from elsewhere cannot start a brain
  (port doc SPEC §1.1 as amended 2026-10-03).
"""
from __future__ import annotations

import argparse
import curses
import json
import math
import os
import socket
import threading
import time
from typing import Any, Optional

import dash_run  # beside this file

try:
    import zmq
except ImportError:                      # bench metrics need it; the brain half does not
    zmq = None                           # degrade rather than refuse to start


# --------------------------------------------------------------------------- transports

class LineSpeed:
    """Joint line speed from benchd's 50 Hz state feed: the pulse on the line (out_us), sampled
    over exact 100 ms spans, mean |change| per second over the armed channels, in rad/s.

    The sim's HUD power panel measures its joints the same way (10 Hz, mean over 12), so the two
    numbers are comparable with each other.  Neither sees reversals inside 100 ms, so neither is
    the brain's own command rate.  The feed is loopback-only on the Pi, so this reads it where
    benchd runs and is quietly absent elsewhere.
    """

    def __init__(self, port: int = 5592):
        self.speed: Optional[float] = None
        self.speed_max: Optional[float] = None
        self.t_last = 0.0
        try:
            cal = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                              "..", "calib", "sensors.json")))
            self.us_per_rad = float(cal["servo"]["us_per_rad"])
        except Exception:
            self.us_per_rad = 545.2
        if zmq is not None:
            threading.Thread(target=self._loop, args=(port,), daemon=True).start()

    def _loop(self, port: int) -> None:
        s = zmq.Context.instance().socket(zmq.SUB)
        s.setsockopt(zmq.RCVTIMEO, 500)
        s.setsockopt_string(zmq.SUBSCRIBE, "state ")
        s.connect(f"tcp://127.0.0.1:{port}")
        hist: list = []                                       # (t_ms, out[12], armed bitmask)
        while True:
            try:
                d = json.loads(s.recv_string()[6:])
            except Exception:
                continue
            out, t = d.get("out"), float(d.get("t", 0))
            if not out:
                continue
            hist.append((t, out, int(d.get("armed", 0))))
            while hist and t - hist[0][0] > 400:
                hist.pop(0)
            old = next((h for h in hist if t - h[0] >= 100), None)
            if old is None or t - old[0] > 140:
                continue
            dt = (t - old[0]) / 1000.0
            v = [abs(out[c] - old[1][c]) / dt / self.us_per_rad
                 for c in range(len(out)) if (hist[-1][2] >> c) & 1 and out[c] > 0 and old[1][c] > 0]
            if v:
                self.speed, self.speed_max, self.t_last = sum(v) / len(v), max(v), time.time()

    def fresh(self) -> bool:
        return self.speed is not None and time.time() - self.t_last < 1.0

class Control:
    """Plain-TCP newline-JSON client for ogma_host's ControlServer. No dependencies."""

    def __init__(self, host: str, port: int, timeout: float = 1.5):
        self.host, self.port, self.timeout = host, port, timeout
        self._sock: Optional[socket.socket] = None
        self._buf = b""

    def _connect(self) -> None:
        if self._sock is not None:
            return
        s = socket.create_connection((self.host, self.port), self.timeout)
        s.settimeout(self.timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock, self._buf = s, b""

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock, self._buf = None, b""

    def call(self, verb: str, **kw: Any) -> Optional[dict]:
        """One request.  Returns None on any failure and drops the socket, so the next
        call reconnects — the daemon restarting must not need the dash restarted too."""
        try:
            self._connect()
            assert self._sock is not None
            self._sock.sendall((json.dumps(dict(verb=verb, **kw)) + "\n").encode())
            while b"\n" not in self._buf:
                chunk = self._sock.recv(1 << 20)
                if not chunk:
                    raise ConnectionError("closed")
                self._buf += chunk
            line, _, self._buf = self._buf.partition(b"\n")
            return json.loads(line.decode())
        except Exception:
            self.close()
            return None


class Bench:
    """ZMQ REQ to ogma_benchd. REQ/REP is fair-queued, so the Godot dashboard can hold
    its own socket at the same time. Optional: absent pyzmq, the dash still runs."""

    def __init__(self, host: str, port: int, timeout_ms: int = 700):
        self.host, self.port, self.timeout_ms = host, port, timeout_ms
        self._ctx = zmq.Context.instance() if zmq else None
        self._sock = None

    def _connect(self) -> None:
        if self._sock is not None or not zmq:
            return
        s = self._ctx.socket(zmq.REQ)
        s.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        s.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        s.setsockopt(zmq.LINGER, 0)
        s.connect(f"tcp://{self.host}:{self.port}")
        self._sock = s

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close(0)
            self._sock = None

    def call(self, verb: str, **kw: Any) -> Optional[dict]:
        if not zmq:
            return None
        try:
            self._connect()
            self._sock.send_string(json.dumps({"verb": verb, **kw}))
            return json.loads(self._sock.recv_string())
        except Exception:
            # A REQ that missed its reply is stuck by protocol; recreate it.
            self.close()
            return None

    def status(self) -> Optional[dict]:
        return self.call("status")


# --------------------------------------------------------------------------- rendering

DIM, OK, WARN, BAD, HEAD = 1, 2, 3, 4, 5


def fmt_dur(ms: float) -> str:
    if ms >= 1.0:
        return f"{ms:.2f}ms"
    if ms >= 0.001:
        return f"{ms * 1000:.1f}us"
    return f"{ms * 1e6:.0f}ns"


def gng_baked(g: dict) -> tuple:
    """(node_count, baked_count, baked_fraction) from a GNG snapshot.

    ⚠ THERE IS NO `baked` FIELD ON A NODE.  Baking is a DERIVED predicate --
    `visits >= baking_threshold` (GNG::baked_count in cpp_core/src/v3/gng.cpp) -- and a
    node carries `visits`, `bake_checked`, `post_bake_visits`, never `baked`.  Reading a
    key that is not there returns a plausible-looking 0 for every module forever, which
    is how this tool first reported "baked 0 everywhere" on EPMs that were in fact 38-88%
    baked.  Compute it; do not look it up.
    """
    nodes = g.get("nodes")
    if not isinstance(nodes, list):
        return (nodes if nodes is not None else "?", "?", "?")
    thresh = int(g.get("baking_threshold", 50))
    baked = sum(1 for x in nodes if int(x.get("visits", 0)) >= thresh)
    return (len(nodes), baked, (baked / len(nodes)) if nodes else 0.0)


def fmt_uptime(sec: float) -> str:
    sec = int(max(0, sec))
    d, r = divmod(sec, 86400)
    h, r = divmod(r, 3600)
    m, _ = divmod(r, 60)
    return f"{d}d{h:02d}h{m:02d}m" if d else f"{h}h{m:02d}m"


def bar(frac: float, width: int) -> str:
    frac = max(0.0, min(1.0, frac))
    n = int(frac * width)
    return "#" * n + "." * (width - n)


class Dash:
    def __init__(self, host: str, ctl_port: int, bench_port: int, interval: float):
        self.control = Control(host, ctl_port)
        self.bench = Bench(host, bench_port)
        # STOP gets its OWN socket: a REQ waiting on a status reply cannot send, and the
        # stop key must never queue behind the poller.
        self.stopper = Bench(host, bench_port, timeout_ms=500)
        self.st_time = 0.0                    # when self.st last arrived
        self.stop_reply: Optional[dict] = None
        self.stop_reply_time = 0.0
        self.msg = ""                         # the last STOP/RESUME outcome, shown on screen
        self.msg_bad = False
        # ---- run config ----
        self.ui = "monitor"                  # monitor | pick | confirm | run
        self.configs: list = []
        self.sel = 0
        self.poses: list = ["stand"]
        self.pose_idx = 0
        self.checks: list = []
        self.ctrl: Optional[dash_run.RunController] = None
        self.end_requested_at = 0.0          # E pressed; the fallback fires if nothing happens
        self.interval = interval
        self.host = host
        self.modules: list[dict] = []
        self.snaps: dict[str, dict] = {}
        self.sensors: Optional[dict] = None
        self.brain: Optional[dict] = None
        self.st: Optional[dict] = None
        self.info: Optional[dict] = None      # host_info: fetched once, it is static
        self.t0 = time.time()
        self.line = LineSpeed()

    def known_stopped(self) -> bool:
        """Is the robot KNOWN to be stopped right now?  Only fresh evidence counts; when it
        is unknown the answer is False, so an uncertain press sends STOP, never RESUME."""
        now = time.time()
        fresh = max(1.0, 1.5 * self.interval)
        if self.stop_reply is not None and now - self.stop_reply_time < fresh \
                and self.stop_reply_time >= self.st_time:
            return bool(self.stop_reply.get("stopped"))
        if self.st is not None and self.st.get("ok", True) and now - self.st_time < fresh:
            return bool(self.st.get("stopped"))
        return False

    def toggle_stop(self) -> None:
        resuming = self.known_stopped()
        r = self.stopper.call("resume" if resuming else "stop")
        if r is not None and r.get("ok"):
            self.stop_reply, self.stop_reply_time = r, time.time()
            self.msg = ("re-arming the servos one at a time after the HAT reset — the run resumes when they land"
                        if resuming and r.get("recovering")
                        else f"RESUMED ({r.get('mode', '?')} mode)" if resuming
                        else "STOPPED — every servo frozen where it is.  SPACE to resume.")
            self.msg_bad = False
        else:
            err = (r or {}).get("error", "no reply from ogma_benchd")
            self.msg = (f"resume refused: {err}" if resuming
                        else f"STOP FAILED: {err} — cut power if the robot is at risk")
            self.msg_bad = True

    def poll(self) -> None:
        st = self.bench.status()
        self.st, self.st_time = st, time.time()
        self.brain = self.control.call("ping")
        if self.brain is not None:
            # Static for the life of the process, so fetch it once — and re-fetch after a
            # reconnect, since the daemon may have restarted onto a different config.
            if self.info is None:
                self.info = self.control.call("host_info")
            self.sensors = self.control.call("host_sensors")
            resp = self.control.call("list_modules")
            if resp and resp.get("status") == "ok":
                self.modules = [m for m in resp.get("modules", []) if m.get("type") == "EPM"]
            for m in self.modules:
                # Stateless read — allocates nothing on the DiagPublisher, so the
                # inspector's subscription is untouched.
                r = self.control.call("module_snapshot", id=m["id"])
                if r and r.get("status") == "ok":
                    self.snaps[m["id"]] = r.get("snapshot", {})
        else:
            self.sensors = None
            self.info = None                   # force a re-read when it comes back
            self.modules = []

    # -- drawing helpers that never raise on a small terminal --
    def _line(self, scr, y: int, x: int, text: str, attr=0) -> None:
        h, w = scr.getmaxyx()
        if 0 <= y < h and x < w:
            try:
                scr.addnstr(y, x, text, max(0, w - x - 1), attr)
            except curses.error:
                pass

    # ---------------------------------------------------------------- run config UI
    def open_picker(self) -> None:
        try:
            self.configs = dash_run.list_configs()
        except OSError as e:
            self.msg, self.msg_bad = f"cannot read the config list: {e}", True
            return
        self.sel = 0
        self.ui = "pick"

    def open_confirm(self) -> None:
        io = dash_run.RobotIo()
        self.poses = (io.bench.call("pose.list") or {}).get("poses") or ["stand"]
        self.pose_idx = self.poses.index("stand") if "stand" in self.poses else 0
        self.checks = dash_run.preflight(io, self.poses[self.pose_idx])
        io.bench.close(); io.ctl.close()
        self.ui = "confirm"

    def open_confirm_keep_pose(self) -> None:
        io = dash_run.RobotIo()
        self.checks = dash_run.preflight(io, self.poses[self.pose_idx])
        io.bench.close(); io.ctl.close()

    def start_run(self) -> None:
        cfg = self.configs[self.sel]
        self.ctrl = dash_run.RunController(dash_run.RobotIo(), cfg, self.poses[self.pose_idx])
        self.ctrl.start()
        self.ui = "run"

    def handle_run_key(self, ch: int) -> bool:
        """Keys while a run screen is up.  Returns True if the poller should refresh."""
        if self.ui == "pick":
            if ch in (curses.KEY_UP, ord("k")):
                self.sel = max(0, self.sel - 1)
            elif ch in (curses.KEY_DOWN, ord("j")):
                self.sel = min(len(self.configs) - 1, self.sel + 1)
            elif ch in (10, 13, curses.KEY_ENTER) and self.configs:
                self.open_confirm()
            elif ch in (27, ord("q")):
                self.ui = "monitor"
            return False
        if self.ui == "confirm":
            blocked = any(ck.blocking and not ck.ok for ck in self.checks)
            faithful = self.configs[self.sel].faithful
            if ch in (ord("p"), ord("P")) and self.poses:
                self.pose_idx = (self.pose_idx + 1) % len(self.poses)
                self.open_confirm_keep_pose()
            elif ch in (ord("r"), ord("R")):
                self.open_confirm_keep_pose()
            elif not blocked and ((faithful and ch in (10, 13, curses.KEY_ENTER)) or (not faithful and ch == ord("Y"))):
                self.start_run()
            elif ch in (27, ord("q")):
                self.ui = "pick"
            return False
        if self.ui == "run" and self.ctrl is not None:
            ph = self.ctrl.st.phase
            if ph == "countdown":
                self.ctrl.abort()                       # ANY key: nothing has moved yet
            elif ph in ("prepare", "running"):
                if ch == ord(" "):
                    # Resuming mid-reset would hand the brain a body that is still moving.
                    if self.known_stopped() and (self.st or {}).get("pose_move_active"):
                        self.msg, self.msg_bad = "wait: the reset pose is still moving (SPACE when it lands)", True
                        return True
                    self.toggle_stop()
                    return True
                if ch in (ord("r"), ord("R")):
                    self.ctrl.reset()
                elif ch in (ord("e"), ord("E")):
                    self.ctrl.end()
                    self.end_requested_at = time.time()
                    self.msg, self.msg_bad = "ending the run…", False
                elif ch in (ord("q"), ord("Q")):
                    self.msg, self.msg_bad = "a run is live: E ends it (rescue pose), then q quits", True
            elif ph in ("done", "aborted"):
                self.ui, self.ctrl = "monitor", None
            return True
        return False

    def draw_pick(self, scr, C) -> None:
        h, w = scr.getmaxyx()
        self._line(scr, 0, 1, "RUN CONFIG — the Godot launcher's picrawler allowlist", C(HEAD) | curses.A_BOLD)
        self._line(scr, 1, 1, "↑/↓ select   ENTER confirm   ESC back      ✓ ROBOT = robot-faithful inputs "
                              "(the only kind validated on hardware)", C(DIM))
        rows = max(1, h - 4)
        top = max(0, min(self.sel - rows + 1, len(self.configs) - rows)) if self.sel >= rows else 0
        for i, c in enumerate(self.configs[top:top + rows]):
            k = top + i
            tag = "✓ ROBOT " if c.faithful else "  sim   "
            attr = (curses.A_REVERSE if k == self.sel else 0) | C(OK if c.faithful else WARN)
            self._line(scr, 3 + i, 1, f"{tag} {c.name[:max(10, w - 12)]}", attr)

    def draw_confirm(self, scr, C) -> None:
        c = self.configs[self.sel]
        y = 0
        self._line(scr, y, 1, "RUN CONFIG — confirm", C(HEAD) | curses.A_BOLD); y += 2
        self._line(scr, y, 3, c.name, C(OK if c.faithful else WARN) | curses.A_BOLD); y += 1
        self._line(scr, y, 3, c.file, C(DIM)); y += 2
        if not c.faithful:
            self._line(scr, y, 3, "⚠ SIM INPUTS: tuned on inputs the robot cannot publish (achieved joint "
                                  "angles, god's-eye signals). On hardware it runs partly blind.",
                       C(BAD) | curses.A_BOLD); y += 2
        self._line(scr, y, 3, f"start pose: {self.poses[self.pose_idx]}   (P cycles saved poses)", C(OK)); y += 1
        self._line(scr, y, 3, f"mode: autonomous — after a HAT reset the robot returns to '{self.poses[self.pose_idx]}' "
                              f"and the run continues (warning shown)", C(DIM)); y += 1
        self._line(scr, y, 3, f"tilt guard: STOP past {dash_run.TILT_LIMIT_DEG:.0f}°    STOP/resume: SPACE    "
                              f"reset to start pose: R    end: E (rescue pose)", C(DIM)); y += 1
        self._line(scr, y, 3, "HAT off mid-run: SPACE (pause) → HAT off → move the robot → HAT on → SPACE "
                              "(back to the start pose, one servo at a time, then continues)", C(DIM)); y += 2
        for ck in self.checks:
            col = OK if ck.ok else (BAD if ck.blocking else WARN)
            self._line(scr, y, 3, ("✓ " if ck.ok else ("✗ " if ck.blocking else "! ")) + ck.text, C(col)); y += 1
        y += 1
        if any(ck.blocking and not ck.ok for ck in self.checks):
            self._line(scr, y, 3, "cannot run: fix the ✗ items (R re-checks)   ESC back", C(BAD) | curses.A_BOLD)
        else:
            go = "ENTER" if c.faithful else "Y (capital — sim-input config)"
            self._line(scr, y, 3, f"{go}: start the {dash_run.COUNTDOWN_S:.0f} s countdown   R re-check   ESC back",
                       C(HEAD) | curses.A_BOLD)

    def draw_run_panel(self, scr, C, y: int) -> int:
        st = self.ctrl.st
        h, w = scr.getmaxyx()
        if st.phase == "countdown":
            n = int(math.ceil(st.countdown_left))
            self._line(scr, y, 1, f" RUN {self.ctrl.cfg.name[:60]} ", C(HEAD) | curses.A_BOLD); y += 2
            self._line(scr, y, 3, f"  STARTING IN  {n:2d} s  — the robot will move to '{self.ctrl.pose}' "
                                  f"and the brain will take the servos  ",
                       C(BAD) | curses.A_BOLD | curses.A_REVERSE); y += 2
            self._line(scr, y, 3, "ANY KEY ABORTS — nothing has been sent to the robot yet", C(WARN) | curses.A_BOLD)
            return y + 2
        col = {"prepare": WARN, "running": OK, "ending": WARN, "done": HEAD, "aborted": WARN}.get(st.phase, DIM)
        el = (time.monotonic() - st.started_at) if st.started_at and st.phase == "running" else 0
        belly = f"{st.belly_mm:.0f} mm" if st.belly_mm is not None else "—"
        tilt = f"{st.tilt_deg:.0f}°" if st.tilt_deg is not None else "—"
        vb = f"{st.vbat:.2f} V" if st.vbat is not None else "—"
        self._line(scr, y, 1, f" RUN {st.phase.upper():8} {self.ctrl.cfg.name[:44]}  {el:4.0f} s   "
                              f"belly {belly}  tilt {tilt}  vbat {vb} ", C(col) | curses.A_BOLD | curses.A_REVERSE); y += 1
        self._line(scr, y, 3, st.detail, C(col)); y += 1
        if st.recovering:
            self._line(scr, y, 3, " RECOVERING FROM A HAT RESET — re-arming servos one at a time ",
                       C(WARN) | curses.A_BOLD | curses.A_REVERSE); y += 1
        if st.hat_warning:
            self._line(scr, y, 3, st.hat_warning, C(BAD) | curses.A_BOLD); y += 1
        keys = (f"SPACE stop/resume   R reset to '{self.ctrl.pose}' (stays stopped)   E end run (rescue pose)"
                if st.phase in ("prepare", "running")
                else "any key: back to monitoring" if st.phase in ("done", "aborted") else "ending…")
        self._line(scr, y, 3, keys, C(DIM)); y += 1
        self._line(scr, y, 0, "─" * max(0, w - 1), C(DIM))
        return y + 1

    def draw(self, scr) -> None:
        scr.erase()
        h, w = scr.getmaxyx()
        C = curses.color_pair
        y = 0
        if self.ui == "pick":
            self.draw_pick(scr, C); scr.refresh(); return
        if self.ui == "confirm":
            self.draw_confirm(scr, C); scr.refresh(); return
        if self.ui == "run" and self.ctrl is not None:
            y = self.draw_run_panel(scr, C, y)
            if self.ctrl.st.phase == "countdown":
                scr.refresh(); return
        # Uptimes that mean something: the daemons', not this viewer's.
        bench_up = float((self.st or {}).get("uptime_s", 0.0))
        brain_up = float((self.sensors or {}).get("uptime_s", 0.0))
        # Right-align the clock+uptimes only if they fit; addnstr clips rather than
        # wrapping, so a narrow terminal loses the least important end.
        left = f" picrawler dash  {self.host}"
        right = (time.strftime("%H:%M:%S")
                 + f"  bench {fmt_uptime(bench_up)}  brain {fmt_uptime(brain_up)}")
        pad = max(1, w - 1 - len(left) - len(right))
        self._line(scr, y, 0, left + " " * pad + right, C(HEAD) | curses.A_BOLD)
        y += 1
        self._line(scr, y, 0, "─" * max(0, w - 1), C(DIM)); y += 1

        # ---- STOP / mode: the first thing on the screen ----
        st = self.st
        ok = st is not None and st.get("ok", True)
        if ok:
            mode = str(st.get("mode", "?"))
            if self.known_stopped():
                txt = (f" ■ STOPPED ({st.get('stop_why', '?')}, {float(st.get('stopped_ms', 0)) / 1000:.0f} s)"
                       f" — SPACE to resume ")
                self._line(scr, y, 1, txt, C(BAD) | curses.A_BOLD | curses.A_REVERSE)
            else:
                self._line(scr, y, 1, " ● RUNNING — SPACE stops every servo where it is ", C(OK) | curses.A_BOLD)
            if mode == "bench":
                self._line(scr, y, 60, "MODE bench — link loss ⇒ rescue pose", C(WARN))
            else:
                br = st.get("brain") or {}
                state = ("HOLDING (stream lost)" if br.get("holding")
                         else "streaming" if br.get("have_stream") else "silent")
                self._line(scr, y, 60, f"MODE {mode.upper()} — the brain drives, no deadman;"
                                       f" brain {state}, {br.get('applied', 0)} applied", C(BAD) | curses.A_BOLD)
        else:
            self._line(scr, y, 1, " ⚠ benchd unreachable — SPACE cannot stop the robot from here ", C(BAD) | curses.A_BOLD)
        y += 1
        if self.msg:
            self._line(scr, y, 1, self.msg, C(BAD if self.msg_bad else OK))
        y += 1

        # ---- bench ----
        self._line(scr, y, 1, "BENCH  ogma_benchd  ", C(HEAD))
        if not zmq:
            self._line(scr, y, 21, "no pyzmq — apt install python3-zmq", C(WARN))
        elif not ok:
            self._line(scr, y, 21, "unreachable", C(BAD))
        else:
            f = st
            self._line(scr, y, 21,
                       f"tick {float(f.get('tick_hz', 0)):5.2f}Hz   overruns {f.get('overruns', '?')}"
                       f"   bus_err {f.get('bus_errors', '?')}", C(OK))
            y += 1
            vb = float(f.get("vbat", 0.0))
            vcol = BAD if 0 < vb < 6.4 else OK
            self._line(scr, y, 3, f"Vbat  {vb:5.2f} V [{bar((vb - 6.0) / 2.4, 16)}]"
                                  f"   watchdog {f.get('watchdog_trips', '?')}"
                                  f"   throttled {f.get('pi_throttled', '?')}", C(vcol))
            y += 1
            # Whole-robot current (BOM 3): Pi + the 5 V regulator + all 12 servos.
            # Colours track the HAT's 3 A rail rating, which is a datasheet fact --
            # NOT the duty budget, which BOM 3.9 measured to be surface-dependent.
            ina = f.get("ina") or {}
            if ina.get("ok"):
                cur = float(ina.get("i_a", 0.0))
                charging = bool(ina.get("charging"))
                icol = DIM if charging else (BAD if cur > 2.7 else WARN if cur > 2.0 else OK)
                rail = f.get("rail") or {}
                rail_txt = (f"   rail {float(rail.get('v', 0.0)):4.2f} V (min1s {float(rail.get('min_1s', 0.0)):4.2f})"
                            if rail.get("v") else "")
                self._line(scr, y, 3,
                           f"power {cur:+6.3f} A [{bar(cur / 3.0, 16)}]"
                           f"   ina {float(ina.get('v', 0.0)):5.3f} V" + rail_txt
                           + ("   CHARGING — energy numbers are confounded" if charging else ""),
                           C(icol))
                y += 1
                # The SLOW metric: a 30 s mean, a 60 s decaying worst, and what the
                # robot has actually spent.  Instantaneous current says nothing about duty.
                self._line(scr, y, 3,
                           f"slow   ema30 {float(ina.get('i_ema', 0.0)):+6.3f} A"
                           f"   peak60 {float(ina.get('i_peak', 0.0)):5.3f}"
                           f"   max {float(ina.get('i_max', 0.0)):5.3f}"
                           f"   spent {float(ina.get('energy_j', 0.0)) / 1000.0:+7.3f} kJ"
                           f" / {float(ina.get('charge_as', 0.0)):+7.1f} A·s", C(DIM))
                y += 1
                # The same measure as the sim HUD's "joints" line, for the faster-than-sim check.
                if self.line.fresh():
                    self._line(scr, y, 3, f"joints line speed {self.line.speed:4.2f} rad/s mean, "
                                          f"{self.line.speed_max:4.2f} max (10 Hz, pulse on the line)", C(DIM))
                    y += 1
            elif ina:
                self._line(scr, y, 3, f"power  INA219 not reading (errors {ina.get('errors', '?')})", C(BAD))
                y += 1
            # Belly clearance (BOM 2 #4).  The status and the invalid rate sit ON the
            # same lines as the millimetres deliberately: a ToF reading that failed the
            # part's own checks is an arbitrary number, not a large or small one, and it
            # is indistinguishable from a good one if the distance is shown alone.
            tof = f.get("tof") or {}
            if tof.get("ok"):
                mm = float(tof.get("m", 0.0)) * 1000.0
                bad = float(tof.get("bad_frac", 0.0))
                age = int(tof.get("age_ms", -1))
                stale = age < 0 or age > 2000
                valid = bool(tof.get("valid"))
                # Red is for a channel that is not reporting the belly: stopped, or the
                # belly is actually down.  Amber is for one whose word is getting weaker.
                bcol = (BAD if stale or (valid and mm <= 5.0)
                        else WARN if not valid or bad > 0.25 else OK)
                self._line(scr, y, 3,
                           f"belly {mm:6.1f} mm [{bar(mm / 60.0, 16)}]"
                           f"   raw {int(tof.get('raw_mm', 0)):4d} - {float(tof.get('offset_mm', 0.0)):.0f} off"
                           f"   {str(tof.get('status', '?')):8s}"
                           + ("   STALE — ranging stopped" if stale else ""), C(bcol))
                y += 1
                # The SLOW metric.  worst60 is a MIN-hold, not a peak: on this channel
                # LOW is the dangerous end, so a peak-hold would report the safe extreme.
                self._line(scr, y, 3,
                           f"slow   ema30 {float(tof.get('m_ema', 0.0)) * 1000.0:6.1f} mm"
                           f"   worst60 {float(tof.get('m_min', 0.0)) * 1000.0:5.1f}"
                           f"   min {float(tof.get('m_min_all', 0.0)) * 1000.0:5.1f}"
                           f"   invalid {bad * 100.0:3.0f}%"
                           f"   sig {float(tof.get('signal_mcps', 0.0)):5.2f}"
                           f" / amb {float(tof.get('ambient_mcps', 0.0)):5.2f} Mcps", C(DIM))
                y += 1
            elif tof:
                self._line(scr, y, 3, f"belly  VL53L0X not reading (errors {tof.get('errors', '?')})", C(BAD))
                y += 1
            adc = f.get("adc", [])
            self._line(scr, y, 3, "adc   " + "  ".join(f"A{i} {v}" for i, v in enumerate(adc)), C(DIM))
            y += 1
            armed = f.get("armed_ch", -1)
            self._line(scr, y, 3, f"armed {'none' if armed in (-1, None) else 'P' + str(armed)}"
                                  f"   rescue {f.get('rescue_pose') or 'NONE'}"
                                  f"   body {f.get('body', '?')}",
                       C(WARN if armed not in (-1, None) else DIM))
        y += 2

        # ---- brain ----
        self._line(scr, y, 1, "BRAIN  ogma_host    ", C(HEAD))
        if self.brain is None:
            self._line(scr, y, 21, "unreachable — systemctl status ogma-host", C(BAD))
            y += 1
        else:
            self._line(scr, y, 21, f"ticks {self.brain.get('ticks', '?')}", C(OK))
            y += 1
            # WHAT IS RUNNING.  Named because "which config is loaded" is otherwise a
            # guess from whichever checkout the operator happens to be standing in, and
            # two checkouts can hold the same filename with different contents.
            nfo = self.info or {}
            cfgi = nfo.get("config", {})
            if cfgi:
                self._line(scr, y, 3,
                           f"cfg   {os.path.basename(cfgi.get('path', '?'))}"
                           f"   [{cfgi.get('phase_tag', '-')}]"
                           f"   {cfgi.get('stat', {}).get('mtime', '?')}", C(DIM))
                y += 1
                self._line(scr, y, 9, f"{cfgi.get('name', '')[:66]}", C(DIM))
                y += 1
            if nfo:
                ports = nfo.get("ports", {})
                sens = nfo.get("sensors", {})
                on = ",".join(k for k, v in sens.items() if v) or "none"
                self._line(scr, y, 3,
                           f"build {nfo.get('git_sha', '?')}"
                           f"  {nfo.get('binary', {}).get('stat', {}).get('mtime', '?')}"
                           f"   {float(nfo.get('hz', 0)):.0f}Hz"
                           f" {'SCHED_FIFO' if nfo.get('realtime') else 'SCHED_OTHER'}"
                           f"   ports {ports.get('control')}/{ports.get('diag')}/{ports.get('video')}"
                           f"   sensors {on}", C(DIM))
                y += 1
            s = self.sensors or {}
            rg, cam, mic = s.get("range", {}), s.get("camera", {}), s.get("mic", {})
            if rg:
                rv, up = rg.get("valid"), rg.get("up")
                miss = int(rg.get("timeouts", 0))
                tot = miss + int(rg.get("pings", 0))
                self._line(scr, y, 3,
                           f"range {float(rg.get('cm', 0)):6.1f}cm {'ok' if rv else 'NO ECHO':8s}"
                           f"{float(rg.get('hz', 0)):5.1f}Hz   no-echo {100.0 * miss / max(1, tot):4.0f}%",
                           C(BAD if not up else (OK if rv else DIM)))
                y += 1
            if cam:
                self._line(scr, y, 3,
                           f"cam   {float(cam.get('fps', 0)):5.1f}fps  mean {float(cam.get('mean_level', 0)):5.1f}"
                           f"  {cam.get('stride', '?')}px stride  {cam.get('frame_bytes', '?')}B/frame",
                           C(BAD if not cam.get("up") else DIM))
                y += 1
            if mic:
                self._line(scr, y, 3,
                           f"mic   peak {float(mic.get('peak', 0)):.4f}  {float(mic.get('hz', 0)):5.1f}Hz"
                           f"  {int(mic.get('rate', 0)) // 1000}kHz",
                           C(BAD if not mic.get("up") else DIM))
                y += 1
            # Every buffer over/underrun on one line, because "is anything being dropped"
            # is one question.  A no-echo ping is a sensor MISS and is deliberately not
            # summed in here: it means the world was empty, not that the host fell behind.
            drops = [("mic xrun", int(mic.get("xruns", 0))),
                     ("mic", int(mic.get("dropped", 0))),
                     ("cam", int(cam.get("dropped", 0))),
                     ("range", int(rg.get("dropped", 0))),
                     ("tick", int((self.brain or {}).get("overruns", 0)))]
            total = sum(v for _, v in drops)
            self._line(scr, y, 3,
                       "drops " + ("none" if total == 0 else
                                   "  ".join(f"{k} {v}" for k, v in drops if v)),
                       C(OK if total == 0 else WARN))
            y += 1
        y += 1

        # ---- the EPMs: the part the Godot dashboard does not show ----
        self._line(scr, y, 3,
                   f"{'EPM':<16}{'nodes':>6}{'baked':>7}{'frac':>7}{'ema_tle':>10}{'winner':>8}",
                   C(HEAD) | curses.A_BOLD)
        y += 1
        if not self.modules:
            self._line(scr, y, 3, "(no EPMs — brain unreachable or config has none)", C(DIM))
            y += 1
        for m in self.modules:
            sn = self.snaps.get(m["id"], {})
            g = sn.get("gng", {}) or {}
            n, baked, frac = gng_baked(g)
            tle = sn.get("ema_tle")
            # Grade on BAKED FRACTION, never node count (CLAUDE.md §0 rule 4).
            col = OK if isinstance(frac, float) and frac >= 0.30 else WARN
            self._line(scr, y, 3,
                       f"{m['id']:<16}{str(n):>6}{str(baked):>7}"
                       f"{(f'{frac * 100:.0f}%' if isinstance(frac, float) else '?'):>7}"
                       f"{(f'{tle:.4f}' if isinstance(tle, (int, float)) else '?'):>10}"
                       f"{str(sn.get('prev_winner_id_for_transitions', '')):>8}", C(col))
            y += 1

        self._line(scr, h - 3, 0, "─" * max(0, w - 1), C(DIM))
        # A colour with no key is worse than no colour.
        self._line(scr, h - 2, 1, "key ", C(DIM))
        self._line(scr, h - 2, 5, "green ok", C(OK))
        self._line(scr, h - 2, 14, "amber watch", C(WARN))
        self._line(scr, h - 2, 26, "red fault", C(BAD))
        self._line(scr, h - 2, 36,
                   "— EPM amber = under 30% baked (still earning its vocabulary)", C(DIM))
        self._line(scr, h - 1, 1,
                   f"SPACE stop/resume   C run config   q quit   r refresh   every {self.interval:.1f}s"
                   f"   baked = visits >= baking_threshold", C(DIM))
        scr.refresh()

    def run(self, scr) -> None:
        curses.curs_set(0)
        scr.nodelay(True)
        curses.use_default_colors()
        for i, c in ((DIM, curses.COLOR_WHITE), (OK, curses.COLOR_GREEN),
                     (WARN, curses.COLOR_YELLOW), (BAD, curses.COLOR_RED),
                     (HEAD, curses.COLOR_CYAN)):
            curses.init_pair(i, c, -1)
        # The poll (benchd status + every EPM snapshot) can take seconds when the brain is
        # slow or gone; on the key loop it would delay STOP by that much.  So it runs on its
        # own thread and the key loop only draws and reads keys.
        wake = threading.Event()
        quit_ = threading.Event()

        def poller() -> None:
            while not quit_.is_set():
                try:
                    self.poll()
                except Exception:
                    pass
                wake.wait(self.interval)
                wake.clear()

        threading.Thread(target=poller, daemon=True).start()

        def draw() -> None:
            # The poller mutates state while this reads it; a drawing glitch must never
            # take the STOP key down with it.
            try:
                self.draw(scr)
            except Exception:
                pass

        last_draw = 0.0
        while True:
            try:
                ch = scr.getch()
            except curses.error:
                ch = -1
            if ch != -1 and self.ui != "monitor":
                if self.handle_run_key(ch):
                    wake.set()
                draw()
            elif ch == ord(" "):
                self.toggle_stop()
                wake.set()                       # re-poll now so the banner catches up
                draw()
            elif ch in (ord("c"), ord("C")):
                self.open_picker()
                draw()
            elif ch in (ord("q"), ord("Q"), 27):
                quit_.set(); wake.set()
                return
            elif ch in (ord("r"), ord("R")):
                wake.set()
            elif ch == curses.KEY_RESIZE:
                draw()
            now = time.time()
            # E fallback: if the controller has not started the end sequence 2 s after E,
            # run it from here.  It is guarded to run exactly once.
            c = self.ctrl
            if c is not None and self.end_requested_at and now - self.end_requested_at > 2.0:
                self.end_requested_at = 0.0
                if not c.ending_started():
                    self.msg, self.msg_bad = "controller did not respond to E — ending the run from the dash", True
                    threading.Thread(target=c.end_now, daemon=True).start()
            if now - last_draw >= 0.25:
                draw()
                last_draw = now
            time.sleep(0.02)


def main() -> None:
    p = argparse.ArgumentParser(description="picrawler terminal dashboard")
    p.add_argument("--host", default="127.0.0.1", help="default: localhost, i.e. run it on the robot")
    p.add_argument("--control-port", type=int, default=7400)
    p.add_argument("--bench-port", type=int, default=5590)
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--once", action="store_true", help="print one frame and exit (scriptable)")
    a = p.parse_args()

    d = Dash(a.host, a.control_port, a.bench_port, a.interval)
    if a.once:
        d.poll()
        st, br = d.st, d.brain
        print(f"bench: {'ok' if st else 'unreachable'}   brain: {'ok' if br else 'unreachable'}")
        if d.info:
            i = d.info
            print(f"  cfg   {i['config']['path']}  [{i['config'].get('phase_tag', '-')}]")
            print(f"        \"{i['config'].get('name', '')}\"")
            print(f"  build {i.get('git_sha')}  binary {i['binary']['stat'].get('mtime')}"
                  f"  {i.get('hz')}Hz {'SCHED_FIFO' if i.get('realtime') else 'SCHED_OTHER'}")
        if st:
            _ina = st.get("ina") or {}
            if _ina.get("ok"):
                print(f"  power {float(_ina.get('i_a', 0)):+.3f} A  ema30 {float(_ina.get('i_ema', 0)):+.3f}"
                      f"  peak60 {float(_ina.get('i_peak', 0)):.3f}  spent {float(_ina.get('energy_j', 0)) / 1000:.3f} kJ"
                      + ("  CHARGING" if _ina.get("charging") else ""))
            _tof = st.get("tof") or {}
            if _tof.get("ok"):
                print(f"  belly {float(_tof.get('m', 0)) * 1000:.1f} mm  ema30 {float(_tof.get('m_ema', 0)) * 1000:.1f}"
                      f"  worst60 {float(_tof.get('m_min', 0)) * 1000:.1f}"
                      f"  {_tof.get('status', '?')}  invalid {float(_tof.get('bad_frac', 0)) * 100:.0f}%")
            print(f"  mode {st.get('mode', '?')}  {'STOPPED (' + str(st.get('stop_why')) + ')' if st.get('stopped') else 'running'}")
            print(f"  vbat {float(st.get('vbat', 0)):.2f} V  tick {float(st.get('tick_hz', 0)):.2f} Hz"
                  f"  overruns {st.get('overruns')}")
        if d.sensors:
            rg = d.sensors.get("range", {})
            print(f"  range {float(rg.get('cm', 0)):.1f} cm valid={rg.get('valid')}")
        for m in d.modules:
            g = (d.snaps.get(m["id"], {}) or {}).get("gng", {}) or {}
            n, bk, frac = gng_baked(g)
            pct = f"{frac * 100:.0f}%" if isinstance(frac, float) else "?"
            print(f"  {m['id']:<14} nodes {n}  baked {bk} ({pct})")
        return
    try:
        curses.wrapper(d.run)
    except KeyboardInterrupt:
        # Ctrl-C must not leave a brain driving the robot with no console: end the run.
        if d.ctrl is not None and d.ctrl.busy():
            print("Ctrl-C during a live run: ending it (stop, brain off, bench, rescue pose)…", flush=True)
            d.ctrl.end_now()
            print(f"ended. events: {d.ctrl.events_path}", flush=True)
    finally:
        d.control.close()
        d.bench.close()
        d.stopper.close()


if __name__ == "__main__":
    main()
