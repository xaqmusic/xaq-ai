# bench_seq.py — score sequencer runs written by run_gain0.sh: distance, the stairs level reached and when, steps per leg, seconds per
# step, missed landings, phase timeouts, obstacle hits, frozen windows (300 ticks with no body motion and no landing), re-seeds,
# hang counts, multistart counts, falls, max tilt, IK residual.  usage: bench_seq.py [--dir DIR] <run names without .jsonl>
import json, statistics, sys
S='./'
if len(sys.argv) > 2 and sys.argv[1] == '--dir': S = sys.argv[2].rstrip('/') + '/'; sys.argv = [sys.argv[0]] + sys.argv[3:]
def rows(f): return [json.loads(l) for l in open(S+f+'.jsonl') if l.startswith('{') and '"fwd_v"' in l]
def frozen(rs, win=60):
    n=0
    for i in range(0, len(rs)-win, 10):
        a,b=rs[i],rs[i+win]
        if a.get('ps_eng',0)>=1 and abs(b['z']-a['z'])<0.01 and sum(b['ps_steps'])==sum(a['ps_steps']): n+=1
    return n
print(f"{'run':8s} {'z':>5s} {'level':>5s} {'t@lvl':>6s} {'steps':>16s} {'s/step':>6s} {'miss':>4s} {'tmo':>4s} {'obst':>4s} {'frz':>4s} {'reseed':>6s} {'hang':>14s} {'ms[n,sw]':>12s} {'falls':>5s} {'tilt':>5s} {'ik_err':>6s}")
for f in sys.argv[1:]:
    try: rs=rows(f)
    except FileNotFoundError: print(f,'missing'); continue
    eng=[x for x in rs if x.get('ps_eng',0)>=1]; r=rs[-1]; steps=sum(r['ps_steps'])
    lvl=max(x.get('stairs_level',0) for x in rs); tl=next((str(x['t']) for x in rs if x.get('stairs_level',0)>=lvl-1e-4 and lvl>0), '-')
    print(f"{f:8s} {r['z']:5.2f} {lvl:5.3f} {tl:>6s} {str(r['ps_steps']):>16s} {len(eng)*5/50/max(1,steps):6.2f} {r['ps_miss']:4d} {r.get('ps_tmo',0):4d} {r['ps_obst']:4d} {frozen(rs):4d} {r.get('ps_reseed',0):6d} {str(r.get('ps_hang')):>14s} {str(r.get('ps_ms')):>12s} {r['auto_reset_count']:5d} {max(abs(x.get('tilt',0) or 0) for x in eng):5.2f} {statistics.mean(x.get('ps_ik',0) or 0 for x in eng):6.4f}")
