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

Read-only throughout: no verb here can move a servo or change a parameter.
"""
from __future__ import annotations

import argparse
import curses
import json
import os
import socket
import time
from typing import Any, Optional

try:
    import zmq
except ImportError:                      # bench metrics need it; the brain half does not
    zmq = None                           # degrade rather than refuse to start


# --------------------------------------------------------------------------- transports

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

    def status(self) -> Optional[dict]:
        if not zmq:
            return None
        try:
            self._connect()
            self._sock.send_string(json.dumps({"verb": "status"}))
            return json.loads(self._sock.recv_string())
        except Exception:
            # A REQ that missed its reply is stuck by protocol; recreate it.
            self.close()
            return None


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
        self.interval = interval
        self.host = host
        self.modules: list[dict] = []
        self.snaps: dict[str, dict] = {}
        self.sensors: Optional[dict] = None
        self.brain: Optional[dict] = None
        self.st: Optional[dict] = None
        self.info: Optional[dict] = None      # host_info: fetched once, it is static
        self.t0 = time.time()

    def poll(self) -> None:
        self.st = self.bench.status()
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

    def draw(self, scr) -> None:
        scr.erase()
        h, w = scr.getmaxyx()
        C = curses.color_pair
        y = 0
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

        # ---- bench ----
        st = self.st
        ok = st is not None and st.get("ok", True)
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
                   f"q quit   r refresh   every {self.interval:.1f}s"
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
        last = 0.0
        while True:
            now = time.time()
            if now - last >= self.interval:
                self.poll()
                last = now
                self.draw(scr)
            try:
                ch = scr.getch()
            except curses.error:
                ch = -1
            if ch in (ord("q"), ord("Q"), 27):
                return
            if ch in (ord("r"), ord("R")):
                last = 0.0
            elif ch == curses.KEY_RESIZE:
                self.draw(scr)
            time.sleep(0.05)


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
        pass
    finally:
        d.control.close()
        d.bench.close()


if __name__ == "__main__":
    main()
