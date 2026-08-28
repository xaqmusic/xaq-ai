#!/usr/bin/env python3
"""Faithful Python port of the dataviz skill's validate_palette.js
(no node runtime available here).  Same thresholds, same Machado-2009
severity-1.0 CVD matrices, same OKLab dE*100 metric."""
import math, sys

BAND = {"light": (0.43, 0.77), "dark": (0.48, 0.67)}
CHROMA_FLOOR = 0.10
CVD_TARGET, CVD_FLOOR = 8.0, 6.0
NORMAL_FLOOR = 15.0
CONTRAST_MIN = 3.0
DEFAULT_SURFACE = {"light": "#fcfcfb", "dark": "#1a1a19"}
MACHADO = {
    "protan": [[0.152286, 1.052583, -0.204868],
               [0.114503, 0.786281, 0.099216],
               [-0.003882, -0.048116, 1.051998]],
    "deutan": [[0.367322, 0.860646, -0.227968],
               [0.280085, 0.672501, 0.047413],
               [-0.011820, 0.042940, 0.968881]],
    "tritan": [[1.255528, -0.076749, -0.178779],
               [-0.078411, 0.930809, 0.147602],
               [0.004733, 0.691367, 0.303900]],
}

def hex2srgb(h):
    h = h.strip().lstrip("#")
    return [int(h[i:i+2], 16) / 255 for i in (0, 2, 4)]

def s2lin(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4

def lin(h):
    return [s2lin(c) for c in hex2srgb(h)]

def rel_lum(h):
    r, g, b = lin(h)
    return 0.2126 * r + 0.7152 * g + 0.0722 * b

def contrast(a, b):
    hi, lo = sorted([rel_lum(a), rel_lum(b)], reverse=True)
    return (hi + 0.05) / (lo + 0.05)

def oklab_from_lin(rgb):
    r, g, b = rgb
    l = (0.4122214708*r + 0.5363325363*g + 0.0514459929*b) ** (1/3)
    m = (0.2119034982*r + 0.6806995451*g + 0.1073969566*b) ** (1/3)
    s = (0.0883024619*r + 0.2817188376*g + 0.6299787005*b) ** (1/3)
    return [0.2104542553*l + 0.7936177850*m - 0.0040720468*s,
            1.9779984951*l - 2.4285922050*m + 0.4505937099*s,
            0.0259040371*l + 0.7827717662*m - 0.8086757660*s]

def oklch(h):
    L, a, b = oklab_from_lin(lin(h))
    return L, math.hypot(a, b)

def simulate(h, kind):
    r, g, b = lin(h); M = MACHADO[kind]
    return [min(1, max(0, M[i][0]*r + M[i][1]*g + M[i][2]*b)) for i in range(3)]

def delta_e(h1, h2, kind=None):
    a = oklab_from_lin(simulate(h1, kind) if kind else lin(h1))
    b = oklab_from_lin(simulate(h2, kind) if kind else lin(h2))
    return 100 * math.dist(a, b)

def validate(palette, mode="dark", surface=None, pairs="adjacent"):
    surface = surface or DEFAULT_SURFACE[mode]
    lo, hi = BAND[mode]; ok = True
    print(f"palette n={len(palette)}  mode={mode}  surface={surface}  pairs={pairs}\n")

    off = [(c, round(oklch(c)[0], 3)) for c in palette if not (lo <= oklch(c)[0] <= hi)]
    ok &= not off
    print(f"[{'PASS' if not off else 'FAIL'}] Lightness band  " +
          (f"outside L {lo}-{hi}: {off}" if off else f"all inside L {lo}-{hi}"))

    lowc = [(c, round(oklch(c)[1], 3)) for c in palette if oklch(c)[1] < CHROMA_FLOOR]
    ok &= not lowc
    print(f"[{'PASS' if not lowc else 'FAIL'}] Chroma floor    " +
          (f"reads gray: {lowc}" if lowc else f"all >= {CHROMA_FLOOR}"))

    n = len(palette)
    pl = ([(i, j) for i in range(n) for j in range(i+1, n)] if pairs == "all"
          else [(i, i+1) for i in range(n-1)])
    worst = None
    for kind in ("protan", "deutan"):
        for i, j in pl:
            d = delta_e(palette[i], palette[j], kind)
            if worst is None or d < worst[0]:
                worst = (d, kind, palette[i], palette[j])
    tri = min(delta_e(palette[i], palette[j], "tritan") for i, j in pl)
    wd = worst[0]
    state = "PASS" if wd >= CVD_TARGET else ("WARN" if wd >= CVD_FLOOR else "FAIL")
    ok &= state != "FAIL"
    print(f"[{state}] CVD separation  worst {pairs} {worst[3]}<->{worst[2]} "
          f"dE {wd:.1f} ({worst[1]}) · tritan {tri:.1f}  (target>={CVD_TARGET})")

    wn = min((delta_e(palette[i], palette[j]), palette[i], palette[j]) for i, j in pl)
    ok &= wn[0] >= NORMAL_FLOOR
    print(f"[{'PASS' if wn[0] >= NORMAL_FLOOR else 'FAIL'}] Normal-vision   "
          f"worst {wn[1]}<->{wn[2]} dE {wn[0]:.1f}  (floor {NORMAL_FLOOR})")

    low = [(c, round(contrast(c, surface), 2)) for c in palette
           if contrast(c, surface) < CONTRAST_MIN]
    print(f"[{'PASS' if not low else 'WARN'}] Contrast        " +
          (f"sub-{CONTRAST_MIN}:1 (needs labels/table): {low}" if low
           else f"all >= {CONTRAST_MIN}:1 vs surface"))
    print("\n" + ("ALL HARD GATES PASS" if ok else "*** HARD FAIL ***"))
    return ok

if __name__ == "__main__":
    pal = [c.strip() for c in sys.argv[1].split(",") if c.strip()]
    mode = "dark"; surface = None; pairs = "adjacent"
    a = sys.argv[2:]
    for i, t in enumerate(a):
        if t == "--mode": mode = a[i+1]
        if t == "--surface": surface = a[i+1]
        if t == "--pairs": pairs = a[i+1]
    sys.exit(0 if validate(pal, mode, surface, pairs) else 1)
