#!/usr/bin/env python3
"""(mj_host/tools/head_modes.py, 2026-10-01, §17.88) The head's two fore-aft modes on the walk: TRANSLATION T = (neck + head)/2 (the view unchanged, the head slides
fore-aft; - = forward) and PITCH P = (neck - head)/2 (the view tilts; + = up). Joints relative to their median.
Per glob: each mode's spread (sd), its stride-band part (the 0.5 s box removed: what nods), its speed, and the
correlation of each with forward speed.
Usage: head_modes.py "glob" [...]   (the host's JSONL; walking ticks after 30 s)"""
import json, glob, sys, math
def sd(x):
    m=sum(x)/len(x); return math.sqrt(sum((a-m)**2 for a in x)/len(x))
def corr(x,y):
    mx,my=sum(x)/len(x),sum(y)/len(y); sx,sy=sd(x),sd(y)
    return sum((a-mx)*(b-my) for a,b in zip(x,y))/(len(x)*sx*sy) if sx>0 and sy>0 else float('nan')
for g in sys.argv[1:]:
    T=[];P=[];V=[];Tf=[];Pf=[];dT=[];dP=[]
    for path in sorted(glob.glob(g)):
        seg=[]
        def flush():
            if len(seg)<60: return
            for i in range(25,len(seg)):
                t,p,v=seg[i]; T.append(t);P.append(p);V.append(v)
                bt=sum(s[0] for s in seg[i-24:i+1])/25; bp=sum(s[1] for s in seg[i-24:i+1])/25
                Tf.append(t-bt); Pf.append(p-bp)
                dT.append(abs(t-seg[i-1][0])/0.02); dP.append(abs(p-seg[i-1][1])/0.02)
        for line in open(path):
            if not line.startswith('{'): continue
            r=json.loads(line)
            if r['t']<30 or r['drive']!='walk' or r.get('stop',0)!=0:
                flush(); seg=[]; continue
            n,h=r['q'][5],r['q'][6]; seg.append(((n+h)/2,(n-h)/2,r['sensed'][0]*0.4))
        flush()
    dT.sort(); dP.sort()
    print(f"{g}: TRANSLATION sd {sd(T):.3f} rad (stride band {sd(Tf):.3f}), speed p50 {dT[len(dT)//2]:.2f} p90 {dT[9*len(dT)//10]:.2f} rad/s, r with v {corr(T,V):+.2f} | "
          f"PITCH sd {sd(P):.3f} (stride band {sd(Pf):.3f}), speed p50 {dP[len(dP)//2]:.2f} p90 {dP[9*len(dP)//10]:.2f}, r with v {corr(P,V):+.2f}")
