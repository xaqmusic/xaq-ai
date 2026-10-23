#!/usr/bin/env python3
"""usr_button — the Robot HAT's USR button starts and ends a run, for demos (operator, 2026-10-23).

Power on, wait for the boot, press USR: the robot repeats the LAST RUN STARTED FROM picrawler-dash (the config,
the start pose, and the speed / LPG period / knobs the operator had set), through the same RunController the
dash uses, so every safety the dash has is here too (preflight, the tilt guard, HAT-reset recovery, the rescue
pose at the end).  No dash needed.

  press while idle        a 5 s countdown (LED blinking); press again inside it to abort
  press while running     end the run (rescue pose)
  LED                     off idle, blinking in the countdown, on while the brain drives, 3 fast blinks = refused

Refuses when benchd is not in bench mode (the dash or another client holds a run), when there is no last run,
or when its config has left the launcher's allowlist.  Every press and outcome goes to pi_host/log/usr_button.log.

The pins are the HAT revision's whose MCU reset is GPIO5 (benchd holds it as "MCURST"): USR = GPIO25,
LED = GPIO26.  Nothing on the HAT drives GPIO25 at rest (it follows either internal pull), so the button's pull
and active level are flags.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import threading
import time
from pathlib import Path

import gpiod
from gpiod.line import Bias, Direction, Edge, Value

import dash_run

LOG = dash_run.LOG_DIR / "usr_button.log"
COUNTDOWN_S = 5.0
DEBOUNCE_S = 0.05
MIN_PRESS_S = 0.08


def log(msg: str) -> None:
    line = f"{_dt.datetime.now():%Y-%m-%d %H:%M:%S} {msg}"
    print(line, flush=True)
    try:
        with open(LOG, "a") as f:
            f.write(line + "\n")
    except OSError:
        pass


def header_chip() -> str:
    """The gpiochip carrying the 40-pin header's line names (RP1 on a Pi 5, bcm2835 before)."""
    for p in sorted(Path("/dev").glob("gpiochip*")):
        try:
            with gpiod.Chip(str(p)) as c:
                c.line_offset_from_id("GPIO25")
                return str(p)
        except (OSError, ValueError):
            continue
    raise SystemExit("no gpiochip carries GPIO25")


class Led:
    def __init__(self, chip: str, name: str | None):
        self.req = None
        if name:
            try:
                self.req = gpiod.request_lines(chip, consumer="usr_button LED",
                                               config={name: gpiod.LineSettings(direction=Direction.OUTPUT,
                                                                                output_value=Value.INACTIVE)})
                self.name = name
            except OSError as e:
                log(f"LED {name} unavailable ({e}); running without it")
        self.mode = "off"
        threading.Thread(target=self._loop, daemon=True).start()

    def _set(self, on: bool) -> None:
        if self.req is not None:
            self.req.set_value(self.name, Value.ACTIVE if on else Value.INACTIVE)

    def _loop(self) -> None:
        t = 0
        while True:
            m = self.mode
            if m == "off":
                self._set(False)
            elif m == "on":
                self._set(True)
            elif m == "blink":
                self._set(t % 4 < 2)                 # 2.5 Hz
            elif m == "refused":
                for _ in range(3):
                    self._set(True); time.sleep(0.08); self._set(False); time.sleep(0.12)
                self.mode = "off"
            t += 1
            time.sleep(0.1)


class Button:
    def __init__(self, chip: str, name: str, pull: str, active_low: bool):
        bias = {"up": Bias.PULL_UP, "down": Bias.PULL_DOWN, "none": Bias.DISABLED}[pull]
        def req(debounce):
            kw = dict(direction=Direction.INPUT, bias=bias, active_low=active_low, edge_detection=Edge.BOTH)
            if debounce:
                kw["debounce_period"] = _dt.timedelta(seconds=DEBOUNCE_S)
            return gpiod.request_lines(chip, consumer="usr_button", config={name: gpiod.LineSettings(**kw)})
        try:
            self.req = req(True)
        except OSError:                              # no hardware debounce on this chip: MIN_PRESS_S still filters blips
            self.req = req(False)
        self.name = name

    def presses(self):
        """Yields once per press (on release), ignoring blips shorter than MIN_PRESS_S."""
        down_at = None
        while True:
            if not self.req.wait_edge_events(_dt.timedelta(seconds=1)):
                continue
            for ev in self.req.read_edge_events():
                t = ev.timestamp_ns / 1e9
                if ev.event_type == ev.Type.RISING_EDGE:           # active (pressed), after active_low
                    down_at = t
                elif down_at is not None:
                    held, down_at = t - down_at, None
                    if held >= MIN_PRESS_S:
                        yield held


class Demo:
    def __init__(self, led: Led):
        self.led = led
        self.ctrl: dash_run.RunController | None = None

    def busy(self) -> bool:
        return self.ctrl is not None and self.ctrl.st.phase not in ("done", "aborted", "idle")

    def refuse(self, why: str) -> None:
        log(f"refused: {why}")
        self.led.mode = "refused"

    def press(self) -> None:
        if self.busy():
            ph = self.ctrl.st.phase
            if ph == "countdown":
                log("press during the countdown: aborted, nothing was sent")
                self.ctrl.abort()
            elif ph in ("prepare", "running"):
                log("press during the run: ending it (rescue pose)")
                self.ctrl.end()
            return
        last = dash_run.load_last_run()
        if not last:
            return self.refuse("no last run (start one from picrawler-dash first)")
        cfg = next((c for c in dash_run.list_configs() if c.file == last["file"]), None)
        if cfg is None:
            return self.refuse(f"the last run's config left the launcher allowlist: {last['file']}")
        io = dash_run.RobotIo()
        st = io.bench.call("status")
        if not st:
            return self.refuse("benchd is not answering")
        if st.get("mode") != "bench":
            return self.refuse(f"benchd is in '{st.get('mode')}' mode: someone else holds a run")
        log(f"starting the last run: {cfg.name}  pose {last.get('pose')}  slew {last.get('slew_us')}"
            f"  period {last.get('lpg_period')}  knobs {last.get('tune')}  (saved {last.get('saved')})")
        self.ctrl = dash_run.RunController(io, cfg, last.get("pose") or "stand", countdown_s=COUNTDOWN_S,
                                           slew_us=last.get("slew_us"), lpg_period=last.get("lpg_period"),
                                           tune=last.get("tune") or {})
        self.ctrl.start()
        threading.Thread(target=self._watch, args=(self.ctrl,), daemon=True).start()

    def _watch(self, ctrl: dash_run.RunController) -> None:
        last_phase, n_ev = None, 0
        while True:
            ph = ctrl.st.phase
            self.led.mode = {"countdown": "blink", "prepare": "blink", "running": "on"}.get(ph, "off")
            if ph != last_phase:
                log(f"phase {ph}")
                last_phase = ph
            evs = ctrl.st.events
            for t, text in evs[n_ev:]:
                log(f"  {text}")
            n_ev = len(evs)
            if ph in ("done", "aborted"):
                self.led.mode = "off"
                return
            time.sleep(0.2)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--button", default="GPIO25")
    ap.add_argument("--led", default="GPIO26", help="'' for none")
    ap.add_argument("--pull", choices=("up", "down", "none"), default="up")
    ap.add_argument("--active-high", action="store_true", help="the button drives the line HIGH when pressed")
    a = ap.parse_args()
    chip = header_chip()
    led = Led(chip, a.led or None)
    btn = Button(chip, a.button, a.pull, active_low=not a.active_high)
    demo = Demo(led)
    log(f"ready: {a.button} on {chip} (pull {a.pull}, active-{'high' if a.active_high else 'low'}), LED {a.led or 'none'};"
        f" last run = {(dash_run.load_last_run() or {}).get('name', 'none yet')}")
    for held in btn.presses():
        log(f"USR press ({held:.2f} s)")
        try:
            demo.press()
        except Exception as e:                      # a bad press must never take the daemon down
            demo.refuse(f"{type(e).__name__}: {e}")


if __name__ == "__main__":
    main()
