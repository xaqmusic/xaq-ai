# Contributing to Zanshin

Thanks for your interest in Zanshin. Contributions of code, docs, tests, and
experiments are welcome.

## Developer Certificate of Origin (DCO)

Zanshin uses the [Developer Certificate of Origin](https://developercertificate.org/)
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
  `python/zanshin/zanshin/encoders/` (Python) and `cpp_core/src/v3/encoder_stft.cpp`
  (C++), or plug in via the `zanshin.encoders` entry-point group.
- **Keep it buildable:** `cmake --build cpp_core/build` and the Python import
  smoke should pass before you open a PR.
- **Third-party deps:** if you add a bundled or vendored dependency, list it in
  [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) with its license.

## Building and testing

See [README.md](README.md#build). For the Python engine:

```sh
pip install -e python/zanshin_core -e python/zanshin
pytest python/zanshin/tests
```
