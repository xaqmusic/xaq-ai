#!/usr/bin/env python3
"""stepper_expect.py <log> module.key=value [...] — assert an arm's levers LANDED (CLAUDE.md §3.2 rule 7): read the run's
OGMA_DUMP_PARAMS lines (the params each module reports as effective) and compare. Values are parsed as JSON; numbers compare
with a 1e-9 tolerance. Prints one line; exit 1 if any lever is missing or different. Used by stepper_battery.sh (EXPECT=...)."""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _logio import open_log
log, wants = sys.argv[1], sys.argv[2:]
params = {}
for l in open_log(log):
    if '"event":"PARAMS"' in l.replace(' ', '') and l.lstrip().startswith('{'):
        try: d = json.loads(l)
        except Exception: continue
        params[d.get('module')] = d.get('params', {})
bad = []
for w in wants:
    k, v = w.split('=', 1); mod, key = k.split('.', 1); want = json.loads(v)
    got = params.get(mod, {}).get(key, '<missing>')
    ok = (abs(float(got) - float(want)) < 1e-9) if isinstance(want, (int, float)) and isinstance(got, (int, float)) else got == want
    if not ok: bad.append(f'{k} = {got!r}, wanted {want!r}')
print('levers landed' if not bad else 'LEVER NOT LANDED: ' + '; '.join(bad))
sys.exit(1 if bad else 0)
