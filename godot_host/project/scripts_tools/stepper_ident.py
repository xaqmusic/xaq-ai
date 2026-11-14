#!/usr/bin/env python3
"""stepper_ident.py — golden-trace identity for the learned stepper plan (docs/plans-and-designs/learned_stepper_plan.md).

    stepper_ident.py <golden dir> <dir>              compare every run in <dir> with the golden run of the same bench
    stepper_ident.py --hash <dir> [--write F --label L]   sha256 of each run's diag stream (optionally stored in F under L)
    stepper_ident.py --check F --label L <dir>       compare <dir>'s hashes with the stored ones

Runs are matched by bench (the file name after the arm label: c_b38_1.jsonl.gz and golden_b38_1.jsonl.gz are both b38_1). The
diag stream is the body's JSON rows (the lines carrying "fwd_v"); Godot's start-up chatter is not compared. Two checks:
  bytes   the diag rows are the same text, row for row (Phase 0's gate: the battery reproduces the golden traces byte for byte)
  fields  every field the golden row carries has the same value in the new row; keys only the new run publishes are listed, not
          counted (the gate for a refactor that adds an instrument: Phase 2's step.outcome stream and room measure)
Exit status 1 when any run differs in its shared fields or has a different row count, so a script can gate on it. Byte-identity
proves only the paths the battery exercises (memory: the gate cannot see what it does not run)."""
import glob, hashlib, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _logio import open_log


def bench_of(path):
    n = os.path.basename(path).split('.jsonl')[0]
    return n.split('_', 1)[1] if '_' in n else n


def runs(d):
    out = {}
    for f in sorted(glob.glob(os.path.join(d, '*.jsonl')) + glob.glob(os.path.join(d, '*.jsonl.gz'))):
        out[bench_of(f)] = f
    return out


def diag_lines(f):
    return [l.rstrip('\n') for l in open_log(f) if l.startswith('{') and '"fwd_v"' in l]


def digest(f):
    h = hashlib.sha256()
    n = 0
    for l in diag_lines(f):
        h.update(l.encode() + b'\n'); n += 1
    return h.hexdigest()[:24], n


def compare(gf, nf):
    a, b = diag_lines(gf), diag_lines(nf)
    same_bytes = a == b
    tot = diff = 0; first = None; new = set()
    for i, (x, y) in enumerate(zip(a, b)):
        if x == y: tot += x.count('":'); continue
        try: x, y = json.loads(x), json.loads(y)
        except Exception: diff += 1; first = first or (i, 'unparseable', None, None); continue
        for k in x:
            tot += 1
            if k not in y or x[k] != y[k]:
                diff += 1
                if first is None: first = (i, k, x[k], y.get(k, '<missing>'))
        new |= set(y) - set(x)
    return dict(rows=(len(a), len(b)), bytes=same_bytes, fields=tot, differ=diff, first=first, new=sorted(new))


def main(argv):
    if argv and argv[0] == '--hash':
        d = argv[1]; hs = {b: digest(f) for b, f in runs(d).items()}
        for b, (h, n) in hs.items(): print(f'{b:8s} {h} {n} rows')
        if '--write' in argv:
            F = argv[argv.index('--write') + 1]; L = argv[argv.index('--label') + 1]
            store = json.load(open(F)) if os.path.exists(F) else {}
            store[L] = {b: {'sha256_24': h, 'rows': n} for b, (h, n) in hs.items()}
            json.dump(store, open(F, 'w'), indent=1, sort_keys=True); print(f'wrote {L} to {F}')
        return 0
    if argv and argv[0] == '--check':
        F, L, d = argv[1], argv[3], argv[4]; ref = json.load(open(F))[L]; bad = 0
        for b, f in runs(d).items():
            if b not in ref: print(f'{b:8s} not in {L}'); continue
            h, n = digest(f); ok = h == ref[b]['sha256_24'] and n == ref[b]['rows']; bad += not ok
            print(f'{b:8s} {"IDENTICAL" if ok else "DIFFERS"} {h} {n} rows (golden {ref[b]["sha256_24"]} {ref[b]["rows"]})')
        missing = sorted(set(ref) - set(runs(d)))
        if missing: print('missing runs:', ' '.join(missing))
        return 1 if bad or missing else 0
    g, d = runs(argv[0]), runs(argv[1]); bad = 0
    for b, f in d.items():
        if b not in g: print(f'{b:8s} no golden run'); continue
        r = compare(g[b], f); fail = r['differ'] > 0 or r['rows'][0] != r['rows'][1]; bad += fail
        print(f"{b:8s} rows {r['rows'][0]}/{r['rows'][1]} bytes {'same' if r['bytes'] else 'DIFFER'} fields {r['differ']} of {r['fields']} differ"
              + (f" first {r['first']}" if r['first'] else '') + (f" new keys {r['new']}" if r['new'] else ''))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
