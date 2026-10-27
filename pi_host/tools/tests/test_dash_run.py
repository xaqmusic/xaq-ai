"""Tests for dash_run — the robot-run sequence behind picrawler_dash's "run config".

Run:  .venv/bin/python -m pytest pi_host/tools/tests/test_dash_run.py -q
A fake robot records every call, so the ORDER of what reaches the robot is checked, and the
property that matters most: nothing is sent before the countdown ends.
"""
import json
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
        self.hat_resets, self.hat = 0, {"outage": False, "recovering": False, "recover_resume": False, "outages_60s": 0}
        self.stop_why = None
        self.slew = 40
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
            if verb == "recover.pose":
                self.recover_pose = kw.get("name"); return {"ok": True}
            if verb == "pose.recall":
                if not self.stopped or self.mode == "bench":
                    return {"ok": False, "error": "STOP first"}
                self.pose_ticks = 3
                return {"ok": True, "eta_ms": 100}
        if verb == "status":
            moving = self.pose_ticks > 0
            if moving:
                self.pose_ticks -= 1
            return {"ok": True, "mode": self.mode, "stopped": self.stopped, "brain": {"frames": 0},
                    "hat_resets": self.hat_resets, "hat": dict(self.hat), "stop_why": self.stop_why,
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
        if verb == "slew.set":
            if not kw.get("confirm"):
                return {"ok": False, "error": "needs confirm"}
            if "slew_us" not in kw:
                return {"ok": True, "slew_us": self.slew, "min": 20, "max": 60}
            if not 20 <= kw["slew_us"] <= 60:
                return {"ok": False, "error": "slew.set: slew_us out of 20-60"}
            was, self.slew = self.slew, kw["slew_us"]
            self.log(f"slew={self.slew}")
            return {"ok": True, "slew_us": self.slew, "was": was}
        if verb == "limits.set":
            if self.mode != "bench":
                return {"ok": False, "error": "refused in a brain mode"}
            if "slew_us" in kw:
                self.slew = kw["slew_us"]
                self.log(f"slew={self.slew}")
            return {"ok": True, "slew_us": self.slew}
        return {"ok": True}

    # RobotIo surface
    def start_audio(self, path):
        self.log("audio_start")
        robot = self
        class H:
            def stop(self_inner):
                robot.log("audio_stop"); return 30.0
        return H()

    def start_brain_diag(self, path, module_ids, port=7400):
        self.log("bdiag_start")
        robot = self
        class H:
            def stop(self_inner):
                robot.log("bdiag_stop"); return 0
        return H()

    def start_feed(self, path):
        self.log("feed_start")
        robot = self
        class H:
            def stop(self_inner):
                robot.log("feed_stop"); return 1500
        return H()

    def new_ctl(self):
        return FakeRpc(self, "ctl")

    def brain_call(self, verb, port=7400, **kw):
        self.log(f"brain {verb} {kw.get('id')}.{kw.get('key')}={kw.get('value')}")
        if verb == "set_param" and kw.get("id") in ("load_phase", "motor_epm"):
            return {"status": "ok", "batch_id": 1}
        return {"status": "error", "message": "unknown module: " + str(kw.get("id"))}

    def systemctl(self, action, unit):
        self.log(f"systemctl {action} {unit}"); return True

    def unit_active(self, unit):
        return True

    def can_sudo(self):
        return True

    def host_exists(self):
        return True

    def spawn_host(self, cfg, log_path, extra=()):
        self.log("spawn")
        log_path = self.tmp / log_path.name
        self.spawned_log = log_path
        return FakeProc(log_path, die=self.host_dies)

    def now(self):
        return time.monotonic()

    def sleep(self, s):
        time.sleep(min(s, 0.01))


# The controller tests pick their configs from THIS list, not the launcher's: the dropdown is pruned
# by the operator (2026-10-10, 2026-10-23) and a pruned dropdown must not break the run controller.
_P = "the_picrawler_motor_epm_embed_corridor_v3base__ga__bodypose__m1auth__planpull__native_measured__tofboom__fsrleg__"
TEST_CONFIGS = [_P + "honest__nohomeo.json", _P + "honest__nohomeo__spd600.json",
                _P + "honest__nohomeo__room_senses__s3a_stuckinst__hip2p030__lpgn_r05_e0d0_ref300.json",
                _P + "ref_servo6.json",
                _P + "honest__nohomeo__room_senses__s3a_stuckinst__hip2p030__lpgn_r05_e0d0_ref300__robot_cand_tl.json",
                _P + "honest__nohomeo__room_senses__s3a_stuckinst__hip2p030__lpgn_r05_e0d0_ref300__robot_p1_swl.json"]


def configs_for_tests():
    import tempfile
    lg = Path(tempfile.mkdtemp()) / "launcher.gd"
    lg.write_text("const _PICRAWLER_CONFIG_ALLOWLIST: Array = [\n" + "".join(f'\t"{f}",\n' for f in TEST_CONFIGS) + "]\n")
    return dash_run.list_configs(launcher_gd=lg)


def cfg():
    return next(c for c in configs_for_tests() if c.file.endswith("__fsrleg__honest__nohomeo.json"))


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
    assert len(dash_run.list_configs()) >= 1          # the real launcher list parses
    cs = configs_for_tests()
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


def test_the_brain_starts_only_after_the_brain_mode_and_the_end_sequence_is_ordered(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.start()
    assert wait_phase(ctrl, {"running"}), ctrl.st.events
    ctrl.end()
    ctrl.join(10)
    c = robot.calls
    order = ["systemctl stop ogma-host", "pose.set", "mode.set=autonomous", "recover.pose", "spawn", "resume"]
    idx = [c.index(x) for x in order]
    assert idx == sorted(idx), c
    # No bench pre-roll: the brain process exists only once benchd is in the brain mode (STOPPED).
    assert c.index("spawn") > c.index("mode.set=autonomous")
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


def test_reset_freezes_first_then_recalls_the_start_pose_and_stays_stopped(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.start()
    assert wait_phase(ctrl, {"running"})
    n = len(robot.calls)
    assert ctrl.reset()
    after = robot.calls[n:]
    assert after[:2] == ["stop", "pose.recall"], after     # freeze BEFORE the pose move
    assert robot.stopped                                   # the brain stays paused
    assert "resume" not in after                           # only the operator resumes
    assert ctrl.st.phase == "running"                      # the run (and the brain) continue
    ctrl.end(); ctrl.join(10)


def test_reset_before_the_brain_has_the_servos_is_refused(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.countdown_s = 5.0
    ctrl.start()
    assert wait_phase(ctrl, {"countdown"})
    assert not ctrl.reset()
    assert robot.calls == []                               # still nothing sent
    ctrl.abort(); ctrl.join(5)


def test_the_end_sequence_runs_once_when_E_and_the_fallback_race(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.start()
    assert wait_phase(ctrl, {"running"})
    ctrl.end()
    t = threading.Thread(target=ctrl.end_now); t.start()      # the dashboard's fallback
    t.join(15); ctrl.join(15)
    assert robot.calls.count("limp") == 1
    assert robot.calls.count("mode.set=bench") == 1
    assert robot.calls.count("systemctl start ogma-host") == 1
    assert ctrl.st.phase == "done"


def test_every_step_is_written_to_the_events_file(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.start()
    assert wait_phase(ctrl, {"running"})
    ctrl.end(); ctrl.join(10)
    text = ctrl.events_path.read_text()
    for needle in ("moving to 'stand'", "RUNNING", "E pressed", "rescue pose commanded", "done"):
        assert needle in text, needle


def test_a_hat_reset_mid_run_is_announced_then_its_recovery(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.start()
    assert wait_phase(ctrl, {"running"})
    time.sleep(0.3)
    # benchd detects an outage: disarmed, STOPPED by the reset, auto-recovery armed
    robot.stopped, robot.stop_why = True, "HAT reset"
    robot.hat.update(outage=True, recover_resume=True, outages_60s=1)
    robot.hat_resets = 1
    t_end = time.time() + 3
    while time.time() < t_end and "HAT RESET" not in ctrl.st.hat_warning:
        time.sleep(0.01)
    assert "auto-recovering" in ctrl.st.hat_warning, ctrl.st.hat_warning
    assert "'stand'" in ctrl.st.hat_warning
    assert robot.recover_pose == "stand"                   # benchd was told to recover to the start pose
    # the HAT is back, servos re-arm one at a time...
    robot.hat.update(outage=False, recovering=True)
    time.sleep(0.4)
    assert ctrl.st.recovering
    # ...and benchd resumes the brain
    robot.hat.update(recovering=False, recover_resume=False); robot.stopped, robot.stop_why = False, None
    t_end = time.time() + 3
    while time.time() < t_end and "recovered" not in ctrl.st.detail:
        time.sleep(0.01)
    assert "recovered" in ctrl.st.detail
    assert ctrl.st.phase == "running"
    ctrl.end(); ctrl.join(10)


def test_an_operator_paused_outage_waits_for_space(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.start()
    assert wait_phase(ctrl, {"running"})
    time.sleep(0.3)
    robot.stopped, robot.stop_why = True, "operator"          # SPACE, then the HAT switched off
    robot.hat.update(outage=True, recover_resume=False, outages_60s=1)
    robot.hat_resets = 1
    t_end = time.time() + 3
    while time.time() < t_end and "HAT RESET" not in ctrl.st.hat_warning:
        time.sleep(0.01)
    assert "SPACE returns to 'stand'" in ctrl.st.hat_warning
    assert "resume" not in robot.calls[robot.calls.index("resume") + 1:]   # the controller never resumes it
    ctrl.end(); ctrl.join(10)


def test_ending_during_an_outage_waits_for_the_hat_before_the_rescue_pose(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.start()
    assert wait_phase(ctrl, {"running"})
    robot.hat.update(outage=True)
    ctrl.end()
    time.sleep(0.6)
    assert "limp" not in robot.calls                        # no pose into a dead HAT
    robot.hat.update(outage=False)
    ctrl.join(10)
    assert "limp" in robot.calls and ctrl.st.phase == "done"


def test_the_run_is_recorded_from_before_the_first_motion_until_after_the_rescue(tmp_path, monkeypatch):
    # Audio is off by default (2026-10-26): the 50 Hz feed is the recording that must span the run.
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.start()
    assert wait_phase(ctrl, {"running"})
    ctrl.end(); ctrl.join(10)
    c = robot.calls
    assert "audio_start" not in c
    assert c.index("feed_start") < c.index("pose.set")               # the pose move is on the recording
    assert c.index("limp") < c.index("feed_stop")                     # the rescue is on the recording
    meta = json.loads((ctrl.rec_dir / "meta.json").read_text())
    assert meta["audio"] is None and meta["feed_frames"] == 1500
    assert ctrl.st.synced is True                                     # flushed to the card: safe to power off
    assert any("SAFE TO POWER OFF" in t for _, t in ctrl.st.events)


def test_a_fast_config_sets_the_slew_in_bench_mode_and_restores_it(tmp_path, monkeypatch):
    monkeypatch.setattr(dash_run, "LOG_DIR", tmp_path)
    fast = next(c for c in configs_for_tests() if c.file.endswith("__nohomeo__spd600.json"))
    assert fast.slew_us == 65 and abs(fast.speed_rad_s - 6.0) < 1e-9
    robot = FakeRobot(tmp_path)
    ctrl = dash_run.RunController(robot, fast, "stand", countdown_s=0.3, ready_extra_s=0.0)
    ctrl.start()
    assert wait_phase(ctrl, {"running"}, 8.0)
    assert robot.slew == 65
    calls = list(robot.calls)
    assert calls.index("slew=65") < calls.index("mode.set=autonomous")
    ctrl.end()
    ctrl.join(10)
    calls = list(robot.calls)
    assert robot.slew == 40
    assert calls.index("mode.set=bench") < calls.index("slew=40") < calls.index("limp")


def test_the_deployed_speed_config_leaves_the_slew_alone(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    assert ctrl.cfg.slew_us == 40
    ctrl.start()
    assert wait_phase(ctrl, {"running"}, 8.0)
    ctrl.end()
    ctrl.join(10)
    assert not any(c.startswith("slew=") for c in robot.calls)


def lpg_cfg():
    return next(c for c in configs_for_tests() if c.file.endswith("__lpgn_r05_e0d0_ref300.json"))


def test_live_speed_keys_ride_slew_set_in_the_brain_mode_and_the_slew_is_restored(tmp_path, monkeypatch):
    monkeypatch.setattr(dash_run, "LOG_DIR", tmp_path)
    robot = FakeRobot(tmp_path)
    ctrl = dash_run.RunController(robot, lpg_cfg(), "stand", countdown_s=0.3, ready_extra_s=0.0)
    ctrl.start()
    assert wait_phase(ctrl, {"running"}, 8.0)
    assert robot.mode == "autonomous"
    msg = ctrl.nudge_speed(+3)                          # 3.67 + 0.75 = 4.42 rad/s -> 49 us/tick
    assert robot.slew == int(round((40 * 50 / dash_run.US_PER_RAD + 0.75) * dash_run.US_PER_RAD / 50)), msg
    assert ctrl.st.slew_us == robot.slew
    for _ in range(10):
        ctrl.nudge_speed(+1)                            # benchd's band stops it at 60
    assert robot.slew <= 60
    ctrl.end()
    ctrl.join(10)
    assert robot.slew == 40                             # the pre-run slew is back
    calls = list(robot.calls)
    assert calls.index("mode.set=bench") < calls.index("slew=40")


def test_live_period_keys_hot_patch_the_lpg_and_are_bounded(tmp_path, monkeypatch):
    monkeypatch.setattr(dash_run, "LOG_DIR", tmp_path)
    robot = FakeRobot(tmp_path)
    c = lpg_cfg()
    assert c.lpg_id == "load_phase" and c.lpg_period == 70.0
    ctrl = dash_run.RunController(robot, c, "stand", countdown_s=0.3, ready_extra_s=0.0)
    ctrl.nudge_period(-2)                               # before the brain takes over: queued, not sent
    assert ctrl.st.lpg_period == 60.0 and ctrl.period_target == 60.0
    assert not any(x.startswith("brain set_param") for x in robot.calls)
    ctrl.start()
    assert wait_phase(ctrl, {"running"}, 8.0)
    assert "brain set_param load_phase.period_ticks=60.0" in robot.calls   # sent at the start, before RESUME
    assert ctrl.st.lpg_period == 60.0                    # and not overwritten by the config's 70
    ctrl.nudge_period(-1)
    assert "brain set_param load_phase.period_ticks=55.0" in robot.calls   # live once running
    for _ in range(30):
        ctrl.nudge_period(-1)
    assert ctrl.st.lpg_period == dash_run.PERIOD_MIN
    ctrl.end()
    ctrl.join(10)


def test_the_period_keys_say_so_when_the_config_has_no_lpg(tmp_path, monkeypatch):
    robot, ctrl = make(tmp_path, monkeypatch)
    assert "no LoadPhaseGait" in ctrl.nudge_period(1)
    assert not any(c.startswith("brain set_param") for c in robot.calls)


def test_a_run_starts_from_the_operators_speed_and_period(tmp_path, monkeypatch):
    monkeypatch.setattr(dash_run, "LOG_DIR", tmp_path)
    robot = FakeRobot(tmp_path)
    c = lpg_cfg()
    ctrl = dash_run.RunController(robot, c, "stand", countdown_s=0.3, ready_extra_s=0.0, slew_us=45, lpg_period=60.0)
    ctrl.start()
    assert wait_phase(ctrl, {"running"}, 8.0)
    calls = list(robot.calls)
    assert robot.slew == 45 and calls.index("slew=45") < calls.index("mode.set=autonomous")
    assert "brain set_param load_phase.period_ticks=60.0" in calls
    assert calls.index("brain set_param load_phase.period_ticks=60.0") < calls.index("resume")   # while frozen
    assert ctrl.st.lpg_period == 60.0 and ctrl.st.slew_us == 45
    ctrl.end()
    ctrl.join(10)
    assert robot.slew == 40


def test_the_configs_own_period_sends_no_patch(tmp_path, monkeypatch):
    monkeypatch.setattr(dash_run, "LOG_DIR", tmp_path)
    robot = FakeRobot(tmp_path)
    ctrl = dash_run.RunController(robot, lpg_cfg(), "stand", countdown_s=0.3, ready_extra_s=0.0)
    ctrl.start()
    assert wait_phase(ctrl, {"running"}, 8.0)
    assert not any(x.startswith("brain set_param") for x in robot.calls)
    assert ctrl.st.lpg_period == 70.0
    ctrl.end()
    ctrl.join(10)


class _Scr:
    def __init__(self, h=50, w=160):
        self.h, self.w, self.lines = h, w, {}
    def getmaxyx(self):
        return self.h, self.w
    def addnstr(self, y, x, s, n, attr=0):
        assert 0 <= y < self.h, (y, s)
        self.lines[y] = self.lines.get(y, "") + s
    def addstr(self, y, x, s, attr=0):
        self.addnstr(y, x, s, len(s), attr)
    def erase(self): self.lines = {}
    def refresh(self): pass
    def clrtoeol(self): pass
    def move(self, y, x): pass


def test_the_dash_shows_the_tuning_readout_and_the_keys(monkeypatch):
    import picrawler_dash as pd
    monkeypatch.setattr(pd.curses, "color_pair", lambda n: 0)
    d = pd.Dash("127.0.0.1", 1, 2, 1.0)
    d.bench_slew = 40
    d.tune_between_runs(ord("]"))
    d.tune_between_runs(ord("-"))
    assert d.tune_slew == int(round((40 * 50 / dash_run.US_PER_RAD + 0.25) * dash_run.US_PER_RAD / 50))
    assert d.tune_period == 45.0
    for _ in range(40):
        d.tune_between_runs(ord("]"))
    assert d.tune_slew == dash_run.LIVE_SLEW_MAX
    scr = _Scr()
    d.draw(scr)
    text = "\n".join(scr.lines.values())
    assert "TUNE" in text and "next run: servo" in text and "45 ticks" in text
    assert "[ ] speed" in scr.lines[scr.h - 1] and "- = period" in scr.lines[scr.h - 1]
    assert "S restart svcs" in scr.lines[scr.h - 1] and "; ' swing lift" in scr.lines[scr.h - 1]
    assert ", . belly target" in scr.lines[scr.h - 1]
    d.tune_between_runs(ord("."))
    assert abs(d.tune_other["height"] - 0.070) < 1e-9
    d.draw(scr)
    assert "belly target 70 mm" in "\n".join(scr.lines.values())


def tl_cfg():
    return next(c for c in configs_for_tests() if c.file.endswith("__robot_cand_tl.json"))


def test_the_belly_target_exists_only_where_the_learner_is_on():
    assert tl_cfg().tunables["height"] == ("motor_epm", 0.065)
    assert "height" not in lpg_cfg().tunables              # no learner: no knob


def test_the_belly_target_is_live_bounded_and_carried_into_the_next_run(tmp_path, monkeypatch):
    monkeypatch.setattr(dash_run, "LOG_DIR", tmp_path)
    robot = FakeRobot(tmp_path)
    ctrl = dash_run.RunController(robot, tl_cfg(), "stand", countdown_s=0.3, ready_extra_s=0.0, tune={"height": 0.055})
    ctrl.start()
    assert wait_phase(ctrl, {"running"}, 8.0)
    calls = list(robot.calls)
    assert "brain set_param motor_epm.tuck_learn_target_m=0.055" in calls
    assert calls.index("brain set_param motor_epm.tuck_learn_target_m=0.055") < calls.index("resume")
    ctrl.nudge("height", +1)
    assert abs(ctrl.st.tuned["height"] - 0.060) < 1e-9
    for _ in range(30):
        ctrl.nudge("height", +1)
    assert abs(ctrl.st.tuned["height"] - dash_run.TUNABLES["height"].hi) < 1e-9
    ctrl.end()
    ctrl.join(10)
    r = dash_run.RunController(robot, lpg_cfg(), "stand").nudge("height", 1)
    assert "not in this config" in r


def test_the_last_run_is_saved_for_the_usr_button_and_read_back(tmp_path, monkeypatch):
    monkeypatch.setattr(dash_run, "LOG_DIR", tmp_path)
    assert dash_run.load_last_run() is None
    dash_run.save_last_run(tl_cfg(), "stand", 46, 50.0, {"height": 0.07})
    d = dash_run.load_last_run()
    assert d["file"] == tl_cfg().file and d["pose"] == "stand" and d["slew_us"] == 46
    assert d["lpg_period"] == 50.0 and d["tune"] == {"height": 0.07}


def test_the_restart_key_needs_a_second_press_and_refuses_during_a_run(monkeypatch):
    import picrawler_dash as pd
    d = pd.Dash("127.0.0.1", 1, 2, 1.0)
    started = []
    monkeypatch.setattr(d, "_restart_services", lambda: started.append(1))
    monkeypatch.setattr(pd.os.path, "exists", lambda p: True)
    monkeypatch.setattr(pd.threading, "Thread", lambda target, daemon: type("T", (), {"start": lambda self: target()})())
    d.st = {"mode": "autonomous"}
    d.request_restart()
    assert not started and "not restarting" in d.msg
    d.st = {"mode": "bench"}
    d.request_restart()
    assert not started and "again" in d.msg              # armed, nothing yet
    d.request_restart()
    assert started == [1]
    monkeypatch.setattr(pd.os.path, "exists", lambda p: False)
    d.restarting = False
    d.request_restart(); d.request_restart()
    assert started == [1] and "only on the robot" in d.msg


def test_picking_a_config_resets_the_tuning(monkeypatch):
    import picrawler_dash as pd
    d = pd.Dash("127.0.0.1", 1, 2, 1.0)
    d.tune_slew, d.tune_period, d.tune_other = 34, 30.0, {"height": 0.03}
    class IO:
        bench = type("B", (), {"call": lambda self, *a, **k: {"poses": ["stand"]}, "close": lambda self: None})()
        ctl = type("C", (), {"close": lambda self: None})()
    monkeypatch.setattr(pd.dash_run, "RobotIo", lambda: IO())
    monkeypatch.setattr(pd.dash_run, "preflight", lambda io, pose: [])
    d.open_confirm()
    assert d.tune_slew is None and d.tune_period is None and d.tune_other == {}


def test_the_swing_lift_moves_femur_and_knee_together_live_and_for_the_next_run(tmp_path, monkeypatch):
    monkeypatch.setattr(dash_run, "LOG_DIR", tmp_path)
    c = tl_cfg()
    assert c.tunables["lift"][:2] == ("motor_epm", 0.3)
    robot = FakeRobot(tmp_path)
    ctrl = dash_run.RunController(robot, c, "stand", countdown_s=0.3, ready_extra_s=0.0, tune={"lift": 0.4})
    ctrl.start()
    assert wait_phase(ctrl, {"running"}, 8.0)
    calls = list(robot.calls)
    hip = [x for x in calls if "leg_phase_lift_hip2" in x]; knee = [x for x in calls if "leg_phase_lift_knee" in x]
    assert hip and abs(float(hip[-1].split("=")[-1]) + 0.4) < 1e-9          # femur up = negative
    assert knee and abs(float(knee[-1].split("=")[-1]) - 0.4 * 0.2 / 0.3) < 1e-9
    ctrl.nudge("lift", +1)
    assert abs(ctrl.st.tuned["lift"] - 0.45) < 1e-9
    assert abs(float([x for x in robot.calls if "leg_phase_lift_hip2" in x][-1].split("=")[-1]) + 0.45) < 1e-9
    for _ in range(30):
        ctrl.nudge("lift", -1)
    assert ctrl.st.tuned["lift"] == 0.0
    ctrl.end()
    ctrl.join(10)


def test_a_knob_turned_before_the_brain_takes_over_is_kept_not_reset(tmp_path, monkeypatch):
    monkeypatch.setattr(dash_run, "LOG_DIR", tmp_path)
    robot = FakeRobot(tmp_path)
    ctrl = dash_run.RunController(robot, tl_cfg(), "stand", countdown_s=0.3, ready_extra_s=0.0)
    ctrl.nudge("lift", +2)                               # 0.30 -> 0.40 during the countdown
    ctrl.start()
    assert wait_phase(ctrl, {"running"}, 8.0)
    assert abs(ctrl.st.tuned["lift"] - 0.40) < 1e-9
    assert any("leg_phase_lift_hip2=-0.4" in x for x in robot.calls)
    ctrl.end()
    ctrl.join(10)


def test_the_lift_learner_knob_exists_only_with_the_rising_window_and_turns_off(tmp_path, monkeypatch):
    monkeypatch.setattr(dash_run, "LOG_DIR", tmp_path)
    c = next(x for x in configs_for_tests() if x.file.endswith("__robot_p1_swl.json"))
    assert c.tunables["slearn"] == ("motor_epm", 0.01)
    assert "slearn" not in tl_cfg().tunables
    robot = FakeRobot(tmp_path)
    ctrl = dash_run.RunController(robot, c, "stand", countdown_s=0.3, ready_extra_s=0.0)
    ctrl.start()
    assert wait_phase(ctrl, {"running"}, 8.0)
    ctrl.nudge("slearn", -1); ctrl.nudge("slearn", -1); ctrl.nudge("slearn", -1)
    assert ctrl.st.tuned["slearn"] == 0.0
    assert "brain set_param motor_epm.swing_learn_rate=0.0" in robot.calls
    ctrl.end()
    ctrl.join(10)


def test_a_mark_key_stamps_the_operators_label_on_the_recording(tmp_path, monkeypatch):
    # 2026-10-27: the stride odometry cannot see a slipping body, so the operator labels the episodes.
    robot, ctrl = make(tmp_path, monkeypatch)
    ctrl.start()
    assert wait_phase(ctrl, {"running"})
    assert ctrl.mark("swim") == "marked swim"
    assert ctrl.mark("good") == "marked good"
    ctrl.end(); ctrl.join(10)
    rows = [json.loads(l) for l in (ctrl.rec_dir / "events.jsonl").read_text().splitlines()]
    marks = [r for r in rows if r["kind"] == "mark"]
    assert [m["label"] for m in marks] == ["swim", "good"]
    assert all("mono_ms" in m for m in marks)                        # on the feed's clock
    assert any("mark: swim" in t for _, t in ctrl.st.events)
