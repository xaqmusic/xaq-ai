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

def parse(path):
    zs=[];xs=[];hy=[];ar=[];fv=[]
    for line in open(path):
        line=line.strip()
        if '"x":' not in line or not line.startswith('{'): continue
        try: d=json.loads(line)
        except: continue
        if 'x' in d:
            xs.append(d['x']);zs.append(d['z']);hy.append(d.get('heading_yaw',0))
            ar.append(d.get('auto_reset_count',0));fv.append(d.get('fwd_v',0.0))
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
    return dict(net_z=zs[-1]-zs[0], max_z=max(zs), net_disp=net_disp, straight=straight,
                fwd_v=fwd_v_mean, turns=(acc-hy[0])/(2*math.pi), falls=max(ar))

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
    for k in ("net_z","max_z","net_disp","straight","fwd_v","turns","falls"):
        m,s=ms([r[k] for r in ok])
        print(f"  {k:<8} mean={m:+.2f}  std={s:.2f}   [{'  '.join(f'{r[k]:+.1f}' for r in ok)}]")
