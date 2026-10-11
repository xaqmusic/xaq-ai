#!/usr/bin/env python3
"""Reset-to-reset repeatability of the belly-flat accel reference.

Each cycle issues a full DEVICE_RESET and re-configures, so it measures what the
chip does across re-initialisation -- the same thing a daemon restart would do.
The robot must NOT move between cycles.
"""
import spidev, time, struct, statistics as st, sys, json

CYCLES = int(sys.argv[1]) if len(sys.argv) > 1 else 4
DUR    = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
A_LSB, G_LSB = 8192.0, 65.5

def cycle():
    s = spidev.SpiDev(); s.open(0, 0); s.mode = 0; s.max_speed_hz = 4000000
    def wr(r, v): s.xfer2([r & 0x7F, v]); time.sleep(0.001)
    def rd(r, n=1): return s.xfer2([r | 0x80] + [0]*n)[1:]
    def bank(b): wr(0x7F, (b & 3) << 4)
    bank(0); wr(0x06, 0x80); time.sleep(0.20)         # DEVICE_RESET
    bank(0); wr(0x06, 0x01); time.sleep(0.05)
    wr(0x07, 0x00); wr(0x03, 0x10); time.sleep(0.02)
    bank(2)
    wr(0x00, 0x04); wr(0x01, 0x0B); wr(0x10, 0x00); wr(0x11, 0x04); wr(0x14, 0x0B)
    bank(0)
    time.sleep(2.0)                                   # settle, discarded
    ax=[];ay=[];az=[];gz=[];tc=[]
    t0=time.monotonic()
    while time.monotonic()-t0 < DUR:
        v = struct.unpack(">hhhhhhh", bytes(rd(0x2D, 14)))
        ax.append(v[0]/A_LSB); ay.append(v[1]/A_LSB); az.append(v[2]/A_LSB)
        gz.append(v[5]/G_LSB); tc.append(v[6]/333.87 + 21.0)
        time.sleep(0.02)
    s.close()
    return dict(n=len(ax), ax=st.mean(ax), ay=st.mean(ay), az=st.mean(az),
                norm=st.mean([ (x*x+y*y+z*z)**0.5 for x,y,z in zip(ax,ay,az) ]),
                gz=st.mean(gz), tc=st.mean(tc),
                sd=st.pstdev(ax))

print("  cyc     n      ax        ay        az      |a|      gz(dps)   dieC")
R=[]
for i in range(CYCLES):
    r = cycle(); R.append(r)
    print("   %d   %5d  %+8.5f %+8.5f %+8.5f  %.5f  %+7.4f  %5.2f"
          % (i, r["n"], r["ax"], r["ay"], r["az"], r["norm"], r["gz"], r["tc"]))
import math
for k in ("ax","ay","az","gz"):
    v=[r[k] for r in R]
    print("  %s: spread %.5f   (min %+.5f  max %+.5f)" % (k, max(v)-min(v), min(v), max(v)))
hz=[(r["ax"],r["ay"]) for r in R]
sp=math.hypot(max(a for a,_ in hz)-min(a for a,_ in hz), max(b for _,b in hz)-min(b for _,b in hz))
print("  horizontal tilt spread across resets: %.5f g = %.3f deg" % (sp, math.degrees(math.asin(sp))))
