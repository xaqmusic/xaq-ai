#!/usr/bin/env python3
"""Seed-averaging harness for the picrawler corridor.
Runs a config across N seeds (concurrently), parses each body log for the honest
corridor metrics, and prints mean ± std so we can tell a ROBUST effect from
trajectory noise (the seed override in OgmaBrain was fixed 2026-07-23 so OGMA_SEED
actually varies the MotorEPM RNG).

Usage:  python3 seedavg.py <config_basename.json> [n_seeds] [max_steps] [difficulty] [extra_env=...]
"""
import json, math, os, subprocess, sys, statistics, concurrent.futures as cf

PROJ = "/home/xaqmusic/xaq-ai/godot_host/project"
# Scratch dir for the per-seed body logs. Defaults to a stable tmp dir; override with
# SEEDAVG_OUT=<dir> (e.g. an agent session scratchpad). Must NOT be a hardcoded
# session path — those go stale and the runs then fail to write.
SP   = os.environ.get("SEEDAVG_OUT", "/tmp/xaq_seedavg")
os.makedirs(SP, exist_ok=True)

def run_one(cfg, seed, max_steps, difficulty, extra):
    out = f"{SP}/sa_{os.path.splitext(cfg)[0]}_s{seed}.log"
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

def parse(path):
    zs=[];xs=[];hy=[];ar=[];fv=[];ts=[]
    gc=[];cy=[];planted=[];tilt=[];lat=[];lifts=None;lifts0=None
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
                scrub=statistics.mean(lat) if lat else 0.0)

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
              ("RHYTHM",   ("steps","step_bal","scrub")))
    for label, keys in GROUPS:
        print(f"  -- {label}")
        for k in keys:
            m,s=ms([r[k] for r in ok])
            fmt = ".3f" if k in ("bellyc","bellyc_min","chassis_y","scrub","tilt_sd") else ".2f"
            per = '  '.join(f"{r[k]:{fmt}}" for r in ok)
            print(f"     {k:<10} mean={m:+{fmt}}  std={s:{fmt}}   [{per}]")
