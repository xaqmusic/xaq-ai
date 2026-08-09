#!/usr/bin/env python3
"""Seed-averaging harness for the picrawler corridor.
Runs a config across N seeds (concurrently), parses each body log for the honest
corridor metrics, and prints mean ± std so we can tell a ROBUST effect from
trajectory noise (the seed override in OgmaBrain was fixed 2026-07-23 so OGMA_SEED
actually varies the MotorEPM RNG).

Usage:  python3 seedavg.py <config_basename.json> [n_seeds] [max_steps] [difficulty] [extra_env=...]
"""
import hashlib, json, math, os, subprocess, sys, statistics, concurrent.futures as cf
import pathlib

# Derived from this script's own location so a fresh clone works anywhere
# (was a hardcoded home directory, which broke every non-author checkout).
PROJ = str(pathlib.Path(__file__).resolve().parents[1])
# Scratch dir for the per-seed body logs. Defaults to a stable tmp dir; override with
# SEEDAVG_OUT=<dir> (e.g. an agent session scratchpad). Must NOT be a hardcoded
# session path — those go stale and the runs then fail to write.
SP   = os.environ.get("SEEDAVG_OUT", "/tmp/xaq_seedavg")
os.makedirs(SP, exist_ok=True)

def run_one(cfg, seed, max_steps, difficulty, extra):
    # 2026-08-03 — arms differentiated ONLY by extra env vars (e.g. two damping levels on
    # one config) previously collided on this filename and silently OVERWROTE each other's
    # logs.  The printed summaries stayed correct (computed sequentially) but every earlier
    # arm's per-tick log was destroyed, so windowavg/instrument reads could only ever see
    # the LAST arm.  Suffix the log with a short hash of the extra env to keep them apart.
    tag = ""
    if extra:
        tag = "_" + hashlib.sha1("|".join(sorted(extra)).encode()).hexdigest()[:6]
    out = f"{SP}/sa_{os.path.splitext(cfg)[0]}{tag}_s{seed}.log"
    env = dict(os.environ, OGMA_PICRAWLER_GYM="corridor", OGMA_SEED=str(seed),
               OGMA_INSPECTOR_PORT=str(7400+seed),
               OGMA_PICRAWLER_GYM_DIFFICULTY=str(difficulty),
               OGMA_PICRAWLER_CONFIG=f"res://addons/ami_ogma/configs/{cfg}",
               OGMA_RESET_MODE="continuous", OGMA_PICRAWLER_MAX_STEPS=str(max_steps))
    for kv in extra:
        k,_,v = kv.partition("="); env[k]=v
    with open(out,"w") as f:
        subprocess.run(["godot4","--headless","--fixed-fps","60","--quit-after","4000000",
                        "--path",".","res://scenes/the_picrawler.tscn"], cwd=PROJ, env=env,
                       stdout=f, stderr=subprocess.STDOUT)
    return parse(out)

# Foot is PLANTED when feet_y < stance_y_threshold (same test the body uses at
# picrawler_body.gd:4556).  Override with OGMA_PICRAWLER_STANCE_Y_THRESHOLD to match
# a run that moved it, or the planted/unstable columns silently measure the wrong body.
STANCE_TH = float(os.environ.get("OGMA_PICRAWLER_STANCE_Y_THRESHOLD", 0.04))
# The robot SPAWNS tucked (belly ~9 mm off the deck) and settles over the first
# seconds.  Postural metrics (clearance, planted, tilt, scrub) must skip that
# transient or bellyc_min just reports the spawn pose on every seed and is blind to
# what walking actually does.  Displacement metrics still use the WHOLE run.
WARMUP_TICKS = int(os.environ.get("SEEDAVG_WARMUP", 900))
# Past this z the robot is close enough to the end of the corridor floor that `falls` and
# `chassis_y` stop measuring the gait.  See the guard at the bottom of __main__.
CORRIDOR_SAFE_Z = float(os.environ.get("SEEDAVG_SAFE_Z", 8.5))

def parse(path):
    zs=[];xs=[];hy=[];ar=[];fv=[];ts=[]
    gc=[];cy=[];planted=[];tilt=[];lat=[];lifts=None;lifts0=None
    # ---- 2026-08-04 · COORDINATION + PER-LEG ERROR.  These have been in the JSONL since
    # 2026-08-03 (MotorEPM deliberately mirrored them out of diag_snapshot so a seedavg arm
    # could see them) and this parser never read them, so every A/B in the campaign scored
    # distance and steps while the operator's actual complaint -- "each leg has its own
    # directive" -- went unmeasured.  A metric that exists but is unparsed is exactly as
    # invisible as one that was never emitted.
    plv=0.0;plv_n=0;coh=[];step_cv=0.0;mtle=[];resp=[]
    tleg=[[] for _ in range(4)];ameg=[[] for _ in range(4)]
    tspr=[];plvw=[];plvwn=[];panic=[];cwspr=0.0;cwmean=0.0
    for line in open(path):
        line=line.strip()
        if '"x":' not in line or not line.startswith('{'): continue
        try: d=json.loads(line)
        except: continue
        if 'x' in d:
            xs.append(d['x']);zs.append(d['z']);hy.append(d.get('heading_yaw',0))
            ar.append(d.get('auto_reset_count',0));ts.append(d.get('t',0))
            if d.get('t', 0) < WARMUP_TICKS:   # postural metrics: steady state only
                continue
            fv.append(d.get('fwd_v',0.0))
            # BELLY-UP: gc_raw = belly ToF rangefinder in METRES (the Markov-compliant
            # sensor).  y = chassis height.  Both needed: chassis height is BLIND to
            # belly-drag on raised terrain, clearance is what would grind a real chassis.
            if 'gc_raw' in d: gc.append(d['gc_raw'])
            cy.append(d.get('y',0.0))
            tilt.append(d.get('tilt',0.0))
            lat.append(abs(d.get('lateral_v',0.0)))
            # STATIC STABILITY: how many feet are actually on the ground.  A trot
            # targets 2, a crawl 3+.  Belly clearance is blind to HOW the support is
            # achieved — this is its complement.
            fy=d.get('feet_y')
            if isinstance(fy,list) and fy: planted.append(sum(1 for v in fy if v < STANCE_TH))
            # RHYTHM COALESCENCE: cumulative per-leg lift events.  Total = steps taken;
            # min/max across legs = whether all four participate or one leg is dragged
            # (the tripod-skid signature).  fwd_v is oscillation-dominated and can't see this.
            ll=d.get('leg_lifted_counts')
            if isinstance(ll,list) and ll:
                if lifts0 is None: lifts0=list(ll)   # subtract the warmup's lifts
                lifts=ll
            # -- coordination.  plv/step_cv are whole-run accumulators inside MotorEPM, so
            # the LAST sample is the run's value; coh/plv_w/motor_tle are instantaneous and
            # must be TIME-AVERAGED (single-instant coherence is what produced the
            # 2026-08-03 retraction -- its random-phase null is 0.450 +- 0.219).
            if 'plv'    in d: plv=d['plv']
            if 'plv_n'  in d: plv_n=d['plv_n']
            if 'step_cv'in d: step_cv=d['step_cv']
            if 'coh'    in d: coh.append(d['coh'])
            if 'plv_w'  in d: plvw.append(d['plv_w'])
            if 'plv_wn' in d: plvwn.append(d['plv_wn'])
            if 'motor_tle' in d: mtle.append(d['motor_tle'])
            # Raw egocentric responsiveness |dx|/|du| (2026-08-09).  Numerator of the
            # actuator-search criterion; 0.0 = not yet computed this run, skip.
            if d.get('sup_resp', 0.0) > 0.0: resp.append(d['sup_resp'])
            if 'panic_eff' in d: panic.append(d['panic_eff'])
            if 'cw_spr'  in d: cwspr=d['cw_spr']       # whole-run accumulator: last = value
            if 'cw_mean' in d: cwmean=d['cw_mean']
            # -- per-leg prediction error.  The inferential-gain direction turns on whether
            # the four legs DIFFER in how well they predict themselves; motor_tle collapses
            # exactly that.  Spread is computed PER SAMPLE and then time-averaged, because a
            # precision weighting is computed per tick -- legs that swap rank over the run
            # would show a near-zero spread if you averaged first and differenced after.
            tl=d.get('tle_leg'); al=d.get('amp_leg')
            if isinstance(tl,list) and tl:
                live=[]
                for i,v in enumerate(tl[:4]):
                    if v is None or v < 0.0: continue     # -1 = leg not initialised
                    tleg[i].append(v); live.append(v)
                if len(live)>1:
                    mu=sum(live)/len(live)
                    if mu>1e-9: tspr.append((max(live)-min(live))/mu)
            if isinstance(al,list) and al:
                for i,v in enumerate(al[:4]):
                    if v is not None and v >= 0.0: ameg[i].append(v)
    if len(zs)<5: return None
    acc=hy[0];prev=hy[0]
    for h in hy[1:]:
        dd=h-prev
        while dd>math.pi:dd-=2*math.pi
        while dd<-math.pi:dd+=2*math.pi
        acc+=dd;prev=h
    # STRAIGHTNESS (the honest heading metric — net-rotation `turns` is BLIND to a body
    # that swings its heading and nets ~0).  path_len = total distance walked; net_disp =
    # straight-line distance spawn→end (any direction).  straight = net_disp/path_len:
    # 1.0 = walked a dead-straight line (held a heading); ~0 = wandered/circled in place.
    path=0.0
    for i in range(1,len(xs)):
        path+=math.hypot(xs[i]-xs[i-1], zs[i]-zs[i-1])
    net_disp=math.hypot(xs[-1]-xs[0], zs[-1]-zs[0])
    straight = net_disp/path if path>1e-6 else 0.0
    # fwd_v = body-frame forward speed (egocentric). Mean over the run is a DIRECT
    # propulsion / anti-scrub proxy: more effective thrust → higher forward speed.
    # Read WITH net_disp — high fwd_v + low net_disp = fast but circling.
    fwd_v_mean = statistics.mean(fv) if fv else 0.0
    # FLAT speed, isolated from the hump.  The corridor hump's base starts at z=2, so
    # progress up to z=1.8 is pure flat traversal — the operator's stated deficit.
    # Reporting only whole-corridor net_z would let a good climb mask a slow walk.
    HUMP_Z = 1.8
    flat_v = 0.0; t_flat = -1.0
    for t, z in zip(ts, zs):
        if z - zs[0] >= HUMP_Z:
            t_flat = t - ts[0]
            flat_v = (z - zs[0]) / t_flat * 60.0 if t_flat > 0 else 0.0
            break
    else:                                   # never got there — honest partial rate
        span = (ts[-1] - ts[0]) or 1
        flat_v = (zs[-1] - zs[0]) / span * 60.0
    lp = [a-b for a,b in zip(lifts, lifts0)] if (lifts and lifts0) else (lifts or [])
    steps = sum(lp)
    # step_bal = least-stepping leg / most-stepping leg.  1.0 = all four share the
    # gait; 0.0 = a leg is being DRAGGED (the tripod-skid).  This is the metric that
    # sees "wobbly, not coalesced into a rhythm" — fwd_v cannot.
    step_bal = (min(lp)/max(lp)) if (lp and max(lp) > 0) else 0.0
    # Per-leg time-means, and the weakest leg's oscillation amplitude.  amp_min is the
    # mandatory companion to any per-leg TLE read: a frozen leg is trivially predictable
    # (tle -> 0) and would otherwise look like the best-modelled limb in the body.  It is
    # also what gates PLV (kPlvAmpFloor = 0.02), so amp_min below that floor means the
    # coordination number lost its support rather than the legs losing their coordination.
    tl_mu=[statistics.mean(v) if v else 0.0 for v in tleg]
    am_mu=[statistics.mean(v) if v else 0.0 for v in ameg]
    live_am=[m for m,v in zip(am_mu,ameg) if v]
    return dict(net_z=zs[-1]-zs[0], max_z=max(zs), net_disp=net_disp, straight=straight,
                fwd_v=fwd_v_mean, turns=(acc-hy[0])/(2*math.pi), falls=max(ar),
                flat_v=flat_v, t_flat=t_flat,
                bellyc=statistics.mean(gc) if gc else 0.0,
                bellyc_min=min(gc) if gc else 0.0,
                chassis_y=statistics.mean(cy) if cy else 0.0,
                planted=statistics.mean(planted) if planted else 0.0,
                unstable=(sum(1 for p in planted if p < 3)/len(planted)) if planted else 0.0,
                steps=steps, step_bal=step_bal,
                tilt_sd=statistics.pstdev(tilt) if len(tilt) > 1 else 0.0,
                scrub=statistics.mean(lat) if lat else 0.0,
                plv=plv, plv_n=plv_n, step_cv=step_cv,
                coh=statistics.mean(coh) if coh else 0.0,
                plv_w=statistics.mean(plvw) if plvw else 0.0,
                plv_wn=statistics.mean(plvwn) if plvwn else 0.0,
                motor_tle=statistics.mean(mtle) if mtle else 0.0,
                # THE CRITERION (ledger ★ open problem): value = responsiveness/(motor_tle+ε).
                # ε matches the selector's in-code 1e-3.  Run-level = ratio of run means, the
                # same read that scored the 2026-08-07 amp sweep at corr +0.996 with net_z.
                resp=statistics.mean(resp) if resp else 0.0,
                hk_value=(statistics.mean(resp)/(statistics.mean(mtle)+1e-3))
                         if (resp and mtle) else 0.0,
                tle_spr=statistics.mean(tspr) if tspr else 0.0,
                tle_legs=tl_mu, amp_legs=am_mu,
                amp_min=min(live_am) if live_am else 0.0,
                panic_duty=(sum(1 for p in panic if p > 0.001)/len(panic)) if panic else 0.0,
                cw_spr=cwspr, cw_mean=cwmean)

def ms(vals): return (statistics.mean(vals), statistics.pstdev(vals))

if __name__=="__main__":
    cfg=sys.argv[1]; n=int(sys.argv[2]) if len(sys.argv)>2 else 6
    steps=int(sys.argv[3]) if len(sys.argv)>3 else 12000
    diff=sys.argv[4] if len(sys.argv)>4 else "0.3"
    extra=[a for a in sys.argv[5:]]
    seeds=list(range(1,n+1))
    with cf.ThreadPoolExecutor(max_workers=min(8,n)) as ex:
        res=list(ex.map(lambda s: run_one(cfg,s,steps,diff,extra), seeds))
    ok=[r for r in res if r]
    print(f"\n{cfg}  (n={len(ok)}/{n} seeds, {steps} ticks, diff {diff})")
    if not ok: print("  no valid runs"); sys.exit()
    # Grouped so no single number can carry a promote decision (CLAUDE.md §3 rule 4).
    GROUPS = (("FLAT SPEED", ("flat_v","t_flat")),
              ("PROGRESS", ("net_z","max_z","net_disp","straight","fwd_v")),
              ("BELLY-UP", ("bellyc","bellyc_min","chassis_y")),
              ("STABILITY",("planted","unstable","tilt_sd","falls","turns")),
              ("RHYTHM",   ("steps","step_bal","scrub","step_cv")),
              # plv is the honest coordination read; coh's random-phase null is 0.450+-0.219,
              # so it is printed only as a check that we are sitting on that null.  plv_w is
              # the trailing-window twin (the only one that can score a perturbation).
              # ALWAYS read plv beside plv_n and plv_w beside plv_wn: a frozen or fallen body
              # scores high PLV trivially, and low support means the number is unmeasured.
              ("COORDINATION", ("plv","plv_n","plv_w","plv_wn","coh")),
              # The inferential-gain direction: does the agent's own prediction error DIFFER
              # across legs?  tle_spr = (max-min)/mean over the four legs, per sample, time
              # averaged.  ~0 means there is nothing for a precision weighting to weight.
              # amp_min guards the freeze trap (a still leg is trivially predictable) and is
              # also PLV's support floor (kPlvAmpFloor = 0.02).
              ("INFERENTIAL", ("motor_tle","tle_spr","amp_min","panic_duty",
                               "cw_spr","cw_mean")),
              # The actuator-search criterion (ledger ★ open problem).  resp is raw
              # |dx|/|du|; hk_value divides by motor_tle.  Read hk_value WITH net_z:
              # a knob moving both the same way is an actuator candidate.
              ("CRITERION", ("resp","hk_value")))
    for label, keys in GROUPS:
        print(f"  -- {label}")
        for k in keys:
            if k not in ok[0]: continue
            m,s=ms([r[k] for r in ok])
            fmt = (".3f" if k in ("bellyc","bellyc_min","chassis_y","scrub","tilt_sd")
                   else ".0f" if k == "plv_n"
                   else ".4f" if k in ("motor_tle","amp_min","resp") else ".2f")
            per = '  '.join(f"{r[k]:{fmt}}" for r in ok)
            print(f"     {k:<10} mean={m:+{fmt}}  std={s:{fmt}}   [{per}]")
    # Per-leg breakdown, printed OUTSIDE the groups because it is a vector per seed.  This is
    # the diagnosis behind tle_spr: which leg is the outlier, and is any leg frozen.
    if ok and ok[0].get("tle_legs"):
        print("  -- PER-LEG  (FL FR RL RR;  tle = own forward-model residual, amp = oscillation)")
        for nm, key in (("tle", "tle_legs"), ("amp", "amp_legs")):
            mu = [statistics.mean([r[key][i] for r in ok]) for i in range(4)]
            print(f"     {nm:<10} " + "  ".join(f"{v:.4f}" for v in mu)
                  + ("   << 0.0000 = never moved, NOT a perfect model" if min(mu) < 1e-9 else ""))
    # GYM-BOUNDARY GUARD (2026-07-27).  `_build_corridor()` lays a 9.5 m curriculum on a
    # 20x20 floor, so the walkable strip ends near z=9.5.
    #
    # ORIGINALLY this guard existed because the +Z end simply DROPPED OFF: a run long
    # enough for a fast arm to reach the edge charged it a `fall` for walking off the map
    # and poisoned chassis_y with the drop, which reads as "this lever destabilizes the
    # gait" when it means "this lever ran out of gym".  It bit the fastest arm first --
    # exactly the one a speed lever exists to demonstrate -- so the bias ran toward
    # REJECTING REAL WINS (CLAUDE.md 3.2 rule 7: you measured your harness, not your idea).
    #
    # THAT FAILURE MODE IS FIXED (same day): both corridor ends now carry 30 deg
    # self-centering walls, so a fast arm is contained instead of dropped, and the -Z end
    # no longer traps a turned-around robot against a vertical face.  What remains is
    # SATURATION rather than corruption -- distance simply stops accumulating once the
    # body is against the far wall, so an arm that reaches it is under-reported and two
    # such arms are indistinguishable.  Still a reason to re-run shorter; no longer a
    # reason to distrust `falls`/`chassis_y`.
    near = [(i + 1, r["max_z"]) for i, r in enumerate(ok) if r["max_z"] > CORRIDOR_SAFE_Z]
    if near:
        print(f"\n  !! GYM-BOUNDARY WARNING: {len(near)}/{len(ok)} seeds passed z={CORRIDOR_SAFE_Z}"
              f" (corridor curriculum ends ~9.5, far wall just past it)")
        for sd, mz in near:
            print(f"       seed {sd}: max_z={mz:.2f}")
        print("     Distance SATURATES against the far wall for those seeds, so net_z/max_z"
              "\n     under-report and two fast arms can tie artificially.  Re-run at 6000"
              "\n     ticks, or move the comparison to the arena, before ranking on distance.")
