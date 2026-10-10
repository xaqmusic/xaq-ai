# local_env — the values that are yours, not the project's

**Copy to `docs/operational/local_env.md`, which is gitignored.** Fill in your own
values and keep them here rather than inline in a doc or a report.

## Why this file exists

On 2026-09-05 a real SSID and two router BSSIDs reached a public repo through
`picrawler_sim2real_port.md` and had to be removed by rewriting history. They were never
in code or config — **nothing in this project has ever needed them.** They got in
because a debugging session was written up honestly and the identifiers *were* the
evidence, and there was nowhere else to put them. This is that somewhere else.

When you write up a network session, put the real values here and use the placeholders
in the tracked prose. The measurement is the finding; the identifier is not.

## Placeholders to use in tracked files

| real thing | write this instead |
|---|---|
| your wifi SSID | `<your-ssid>` |
| a BSSID / MAC | `AA:BB:CC:DD:EE:F0` (`:F1` for a second radio) |
| a public/static IP | `<your-public-ip>` |

RFC1918 addresses (`10.0.0.114`, `192.168.x.x`) are **fine to write** and appear
throughout these docs — they are meaningless outside your LAN, and redacting them would
make the network notes unreadable for no gain.

## Your values

```
SSID                  =
BSSID 2.4 GHz         =
BSSID 5 GHz           =
robot hostname        = picrawler.local     # the project addresses it by name, not IP
robot user            =
```

## Robot access

The project addresses the robot as `picrawler.local` everywhere (mDNS), so its IP is not
configuration and does not belong in a file. If mDNS is not resolving, that is the thing
to fix — hardcoding the address is what §7.5 of the port doc removed.
