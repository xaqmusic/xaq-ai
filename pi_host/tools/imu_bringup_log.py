#!/usr/bin/env python3
"""ICM-20948 bring-up logger: raw accel/gyro/die-temp to JSONL.

Deliberately writes under pi_host/log/, not /tmp -- tmpfs loses the capture on
the reboot that a crash causes, which has already cost this project one dataset.
"""
import spidev, time, struct, json, sys, datetime

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 900.0
A_LSB, G_LSB = 8192.0, 65.5          # +-4 g, +-500 dps

s = spidev.SpiDev(); s.open(0, 0); s.mode = 0; s.max_speed_hz = 4000000
def wr(r, v): s.xfer2([r & 0x7F, v]); time.sleep(0.001)
def rd(r, n=1): return s.xfer2([r | 0x80] + [0]*n)[1:]
def bank(b): wr(0x7F, (b & 3) << 4)

bank(0); wr(0x06, 0x80); time.sleep(0.15)
bank(0); wr(0x06, 0x01); time.sleep(0.05)
wr(0x07, 0x00); wr(0x03, 0x10); time.sleep(0.02)
bank(2)
wr(0x00, 0x04); wr(0x01, 0x0B); wr(0x10, 0x00); wr(0x11, 0x04); wr(0x14, 0x0B)
bank(0); time.sleep(0.1)
assert rd(0x00)[0] == 0xEA, "WHO_AM_I lost after config"

stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
path = "/home/xaqmusic/xaq-ai/pi_host/log/imu_bringup_%s.jsonl" % stamp
f = open(path, "w")
print(path, flush=True)

t0 = time.monotonic()
n = 0
while time.monotonic() - t0 < DUR:
    v = struct.unpack(">hhhhhhh", bytes(rd(0x2D, 14)))
    f.write(json.dumps({
        "t": round(time.monotonic() - t0, 4),
        "a": [round(v[0]/A_LSB, 5), round(v[1]/A_LSB, 5), round(v[2]/A_LSB, 5)],
        "g": [round(v[3]/G_LSB, 4), round(v[4]/G_LSB, 4), round(v[5]/G_LSB, 4)],
        "tc": round(v[6]/333.87 + 21.0, 3),
    }) + "\n")
    n += 1
    if n % 500 == 0: f.flush()
    time.sleep(0.02)
f.close()
