#!/usr/bin/env python3
"""stall_probe.py — what a servo sounds like and draws, free and blocked.  ON THE PI, robot on its stand.

The data behind two things (port doc / ledger 2026-10-04):
  - the stall witnesses for the power-budget work: servo current (benchd's 50 Hz feed) and AUDIO
    (the robot's USB mic), free versus blocked, per joint type;
  - the sim's servo sounds: every commanded event is timestamped on the same monotonic clock as
    the audio, so clips can be cut per event (joint, size, free / blocked) and played in Godot
    as audio events that feed the sim's audio EPM (the STFT encoder, as on the robot).

Sequence, for ONE physical leg (one servo moves at a time):
  pose 'stand' -> 3 s silence -> per joint: free moves of each size x reps (out, hold, back)
  -> [operator blocks the joint] -> blocked moves (held at most --block-hold s, returned at once if
  the current passes --abort-a) -> [operator releases] -> 3 s silence -> rescue pose.
Records, in pi_host/log/stallprobe_<stamp>_<leg>/:
  audio.wav     48 kHz mono S16 from plughw:CARD=Device (the device ogma_host's AudioCapture uses)
  feed.jsonl    benchd's 50 Hz state feed (t = benchd CLOCK_MONOTONIC ms; us, out, i_a)
  events.jsonl  every commanded event on the same clock (time.monotonic, CLOCK_MONOTONIC)
  meta.json     leg, channels, sizes, the audio start time, the counts that prove it acted
The senses-only ogma-host service holds the mic, so it is stopped for the probe and restarted.

  python3 pi_host/tools/stall_probe.py --leg FR            # free + blocked, all three joints
  python3 pi_host/tools/stall_probe.py --leg FR --no-block # free moves only
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import zmq

REPO = Path(__file__).resolve().parents[2]
MIRROR = {"FL": "fr", "FR": "fl", "RL": "rr", "RR": "rl"}        # physical -> sim leg name
DEVICE = "plughw:CARD=Device,DEV=0"
RATE = 48000
ctx = zmq.Context.instance()


def mono_ms() -> float:
    return time.monotonic() * 1000.0


class Rpc:
    def __init__(self, port=5590):
        self.port, self.s = port, None

    def call(self, verb, **kw):
        for _ in range(2):
            try:
                if self.s is None:
                    self.s = ctx.socket(zmq.REQ)
                    self.s.setsockopt(zmq.RCVTIMEO, 1500); self.s.setsockopt(zmq.LINGER, 0)
                    self.s.connect(f"tcp://127.0.0.1:{self.port}")
                self.s.send_string(json.dumps({"verb": verb, **kw}))
                return json.loads(self.s.recv_string())
            except zmq.ZMQError:
                if self.s is not None:
                    self.s.close(0)
                self.s = None
        return {"ok": False, "error": "no reply from benchd"}


class Recorder:
    """benchd's 50 Hz state feed -> feed.jsonl, plus the latest current for the abort check."""

    def __init__(self, path):
        self.f = open(path, "w")
        self.i_a, self.n, self.run = 0.0, 0, True
        self.t = threading.Thread(target=self._loop, daemon=True)
        self.t.start()

    def _loop(self):
        s = ctx.socket(zmq.SUB); s.setsockopt(zmq.RCVTIMEO, 300); s.setsockopt_string(zmq.SUBSCRIBE, "state ")
        s.connect("tcp://127.0.0.1:5592")
        while self.run:
            try:
                d = json.loads(s.recv_string()[6:])
            except zmq.Again:
                continue
            self.i_a = d.get("i_a") or 0.0
            self.n += 1
            self.f.write(json.dumps({"t": d["t"], "seq": d["seq"], "us": d["us"], "out": d.get("out"),
                                     "i_a": d.get("i_a"), "fsr": d.get("fsr")}) + "\n")
        s.close(0)

    def stop(self):
        self.run = False
        self.t.join(2)
        self.f.close()


class Events:
    def __init__(self, path):
        self.f = open(path, "w")
        self.n = 0

    def __call__(self, kind, **kw):
        self.f.write(json.dumps({"mono_ms": round(mono_ms(), 2), "kind": kind, **kw}) + "\n")
        self.f.flush()
        self.n += 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--leg", required=True, choices=sorted(MIRROR), help="PHYSICAL leg")
    ap.add_argument("--joints", default="hip1,hip2,knee")
    ap.add_argument("--deltas", default="100,200,400", help="free move sizes, us")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--block-delta", type=int, default=300)
    ap.add_argument("--block-reps", type=int, default=2)
    ap.add_argument("--block-hold", type=float, default=1.0, help="max seconds pushing into the block")
    ap.add_argument("--abort-a", type=float, default=1.5, help="return at once above this current")
    ap.add_argument("--no-block", action="store_true")
    ap.add_argument("--pose", default="stand")
    ap.add_argument("--dry-run", action="store_true",
                    help="prove the recording pipeline (mic, feed, events, channels): 3 s, NOTHING MOVES")
    a = ap.parse_args()

    bench = Rpc()
    st = bench.call("status")
    if not st.get("ok", True) or st.get("mode") != "bench" or st.get("stopped"):
        sys.exit(f"benchd must be reachable, in bench mode and not STOPPED: {st.get('mode')} stopped={st.get('stopped')}")
    smap = json.load(open(REPO / "pi_host/calib/servo_map.json"))
    sim_leg = MIRROR[a.leg]
    chans = {}
    for sv in smap["servos"]:
        if sv.get("sim_leg") == sim_leg:
            chans[sv["joint"]] = sv
    joints = [j for j in a.joints.split(",") if j in chans]
    deltas = [int(x) for x in a.deltas.split(",")]
    pose = (bench.call("pose.get", name=a.pose) or {}).get("us")
    if not pose:
        sys.exit(f"no saved pose '{a.pose}'")

    out = REPO / "pi_host/log" / f"stallprobe_{time.strftime('%Y%m%d_%H%M%S')}_{a.leg}"
    out.mkdir(parents=True, exist_ok=True)
    ev = Events(out / "events.jsonl")
    print(f"stall_probe: leg {a.leg} (sim {sim_leg}), joints {joints}, records -> {out}")
    print("  the robot must be ON ITS STAND with the legs free. Ctrl-C stops safely.")

    # The mic belongs to the senses service: free it for the probe.
    svc_was = subprocess.run(["systemctl", "is-active", "--quiet", "ogma-host"]).returncode == 0
    if svc_was:
        subprocess.run(["sudo", "-n", "systemctl", "stop", "ogma-host"], check=False)
        time.sleep(1.0)

    stop_ping = threading.Event()

    def pinger():                           # a controlling client: the bench deadman stays quiet
        r = Rpc()
        while not stop_ping.is_set():
            r.call("ping")
            stop_ping.wait(0.2)
    threading.Thread(target=pinger, daemon=True).start()
    rec = Recorder(out / "feed.jsonl")

    t_rec0 = mono_ms()
    arec = subprocess.Popen(["arecord", "-q", "-D", DEVICE, "-c", "1", "-f", "S16_LE", "-r", str(RATE),
                             str(out / "audio.wav")], stderr=subprocess.PIPE)
    t_rec1 = mono_ms()
    ev("audio_start", popen_before_ms=round(t_rec0, 2), popen_after_ms=round(t_rec1, 2))

    current_ch, base_us = None, None
    moves = blocked_done = aborts = 0

    def status():
        return bench.call("status")

    def goto(ch, us, hold, trial, joint, delta, abortable=False):
        """Command one channel, wait for the slew, hold, report the peak current."""
        nonlocal aborts
        r = bench.call("servo.set", ch=ch, us=us)
        if not r.get("ok"):
            raise RuntimeError(f"servo.set ch{ch} {us}: {r.get('error')}")
        ev("move_start", ch=ch, joint=joint, leg=a.leg, to_us=us, trial=trial, delta=delta)
        t_end = time.monotonic() + abs(delta) / 40 * 0.02 + 0.1 + hold
        peak = 0.0
        while time.monotonic() < t_end:
            peak = max(peak, rec.i_a)
            if abortable and rec.i_a > a.abort_a:
                ev("abort", ch=ch, joint=joint, i_a=rec.i_a)
                aborts += 1
                print(f"    current {rec.i_a:.2f} A > {a.abort_a} — returning")
                return peak, True
            time.sleep(0.01)
        ev("hold_end", ch=ch, joint=joint, trial=trial, delta=delta, peak_i_a=round(peak, 3))
        return peak, False

    def silence(sec, label):
        ev("silence_start", label=label)
        time.sleep(sec)
        ev("silence_end", label=label)

    interrupted = False
    try:
        if a.dry_run:
            print(f"  DRY RUN — nothing moves. channels: { {j: chans[j]['ch'] for j in joints} }")
            silence(3.0, "dry_run")
            raise SystemExit
        r = bench.call("pose.set", us=pose)
        if not r.get("ok"):
            raise RuntimeError(f"pose.set {a.pose}: {r.get('error')}")
        ev("pose_start", pose=a.pose)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3 and not status().get("pose_move_active"):
            time.sleep(0.05)
        while status().get("pose_move_active"):
            time.sleep(0.1)
        ev("pose_landed", pose=a.pose)
        silence(3.0, "start")

        for joint in joints:
            sv = chans[joint]
            ch = sv["ch"]
            base = pose[ch]
            current_ch, base_us = ch, base
            lo, hi = sv.get("min_us", 500), sv.get("max_us", 2500)
            sign = 1 if base + max(deltas + [a.block_delta]) <= hi else -1   # stay inside the envelope
            print(f"  {joint} (ch {ch}, base {base} us, moving {'+' if sign > 0 else '-'})")
            for d in deltas:
                for k in range(a.reps):
                    goto(ch, base + sign * d, 0.5, "free", joint, sign * d)
                    goto(ch, base, 0.5, "free_return", joint, -sign * d)
                    moves += 1
                silence(1.0, f"{joint}_after_{d}")
            if not a.no_block:
                ev("prompt", text=f"block {a.leg} {joint}")
                input(f"  >> BLOCK the {a.leg} {joint}: hold the foot or put an object in the path it just "
                      f"moved along, then press Enter (it will push for at most {a.block_hold:.1f} s) ")
                for k in range(a.block_reps):
                    peak, aborted = goto(ch, base + sign * a.block_delta, a.block_hold, "blocked", joint,
                                         sign * a.block_delta, abortable=True)
                    goto(ch, base, 0.6, "blocked_return", joint, -sign * a.block_delta)
                    blocked_done += 1
                    print(f"    blocked push {k + 1}: peak {peak:.2f} A{' (aborted)' if aborted else ''}")
                    silence(1.0, f"{joint}_blocked_{k}")
                ev("prompt", text=f"release {a.leg} {joint}")
                input(f"  >> RELEASE the {a.leg} {joint} and press Enter ")
            current_ch = None
        silence(3.0, "end")
    except SystemExit:
        pass
    except KeyboardInterrupt:
        interrupted = True
        print("\n  interrupted — returning the joint and ending")
        if current_ch is not None:
            bench.call("servo.set", ch=current_ch, us=base_us)
            ev("interrupt_return", ch=current_ch, to_us=base_us)
            time.sleep(1.0)
    finally:
        ev("audio_stop")
        arec.send_signal(signal.SIGINT)                 # arecord finalises the WAV header on SIGINT
        try:
            arec.wait(5)
        except subprocess.TimeoutExpired:
            arec.kill()
        if not a.dry_run:
            r = bench.call("limp")                      # the rescue pose, deadman still fed
            ev("rescue", ok=r.get("ok"))
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15 and (status().get("rescue_active") or status().get("pose_move_active")):
            time.sleep(0.2)
        rec.stop()
        stop_ping.set()
        if svc_was:
            subprocess.run(["sudo", "-n", "systemctl", "start", "ogma-host"], check=False)
        wav = out / "audio.wav"
        audio_s = max(0.0, (wav.stat().st_size - 44) / (2 * RATE)) if wav.exists() else 0.0
        meta = {"leg": a.leg, "sim_leg": sim_leg, "joints": joints,
                "channels": {j: chans[j]["ch"] for j in joints}, "pose": a.pose, "pose_us": pose,
                "deltas": deltas, "reps": a.reps, "block_delta": a.block_delta, "block_hold": a.block_hold,
                "abort_a": a.abort_a, "device": DEVICE, "rate": RATE, "audio_popen_ms": [t_rec0, t_rec1],
                "counts": {"free_moves": moves, "blocked_pushes": blocked_done, "aborts": aborts,
                           "feed_frames": rec.n, "events": ev.n, "audio_seconds": round(audio_s, 1)},
                "interrupted": interrupted,
                "arecord_stderr": (arec.stderr.read().decode(errors="replace")[-300:] if arec.stderr else "")}
        json.dump(meta, open(out / "meta.json", "w"), indent=1)
        c = meta["counts"]
        print(f"stall_probe: {c['free_moves']} free moves, {c['blocked_pushes']} blocked pushes "
              f"({c['aborts']} aborted), {c['feed_frames']} feed frames, {c['audio_seconds']} s of audio -> {out}")
        if c["feed_frames"] == 0 or c["audio_seconds"] < 1:
            print("  ⚠ NOTHING RECORDED on one channel — this run is not evidence (see meta.json)")


if __name__ == "__main__":
    main()
