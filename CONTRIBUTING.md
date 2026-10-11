# Contributing to xaq

Thanks for your interest in xaq. Contributions of code, docs, tests, and
experiments are welcome.

## Developer Certificate of Origin (DCO)

xaq uses the [Developer Certificate of Origin](https://developercertificate.org/)
rather than a Contributor License Agreement. By signing off on your commits you
certify that you wrote the patch or otherwise have the right to submit it under
the project's Apache-2.0 license.

Sign off each commit by adding a `Signed-off-by` line — `git commit -s` does this
automatically:

```
Signed-off-by: Your Name <you@example.com>
```

## Ground rules

- **License:** all contributions are under [Apache-2.0](LICENSE). Do not paste in
  code you don't have the right to relicense.
- **No proprietary encoders:** the bio-mimetic audio front-end is developed
  separately. Audio contributions here should build on the generic STFT path in
  `python/xaq/xaq/encoders/` (Python) and `cpp_core/src/v3/encoder_stft.cpp`
  (C++), or plug in via the `xaq.encoders` entry-point group.
- **Keep it buildable:** `cmake --build cpp_core/build` and the Python import
  smoke should pass before you open a PR.
- **Third-party deps:** if you add a bundled or vendored dependency, list it in
  [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) with its license.

## Enable the hooks (one command, do it on a fresh clone)

```sh
git config core.hooksPath .githooks
cp .secrets-local.example .secrets-local          # then add your own SSID etc.
```

`.githooks/pre-push` refuses to publish real network identifiers — MAC/BSSID addresses by
shape (no setup needed), and anything you list in `.secrets-local` (gitignored, because a
denylist in the repo publishes what it protects). `pre-commit` runs the same check early so
a hit costs an amend instead of a rebase. Both are bypassable with `--no-verify`, which is
the point: they are a backstop for the rule in [REPORTS.md](REPORTS.md) §9.5, not a
replacement for reading it.

This exists because a real SSID and two router BSSIDs reached the public repo through a
documentation write-up on 2026-09-05 and had to be removed by rewriting history. They were
never in code or config — no config hygiene would have caught it — so the check scans
content, including commit messages.

## Building and testing

See [README.md](README.md#build). For the Python engine:

```sh
pip install -e python/xaq_core -e python/xaq
pytest python/xaq/tests
```
