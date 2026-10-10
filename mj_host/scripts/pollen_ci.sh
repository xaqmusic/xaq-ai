#!/usr/bin/env bash
# pollen_ci.sh — run Pollen's own CI gates on a checkout of pollen-robotics/microduck, the way
# their `.github/workflows/ci.yml` runs them, before a pull request is shown to the operator
# (REPORTS.md §9.7: "their formatter, linter, and test suite run clean, with before/after counts").
#
#   mj_host/scripts/pollen_ci.sh <checkout> [<baseline ref, default origin/main>]
#
# Runs, in their order: cargo fmt --all --check; cargo clippy --workspace --all-targets under
# RUSTFLAGS=-D warnings; cargo test --workspace; then cargo llvm-cov with their floor (72 % lines,
# ci.yml COVERAGE_FLOOR). Then the test and coverage counts on the baseline ref, so the PR text can
# say "1363 -> 1366".
#
# Two things learned on the first PR (2026-09-10):
#   * Their workspace tests reach the host's systemd over the system bus (configd unit status,
#     robotctl's systemctl wrappers). On a desktop, polkit answers with password dialogs; a CI
#     runner has no agent and the call fails. DBUS_SYSTEM_BUS_ADDRESS points at a dead socket here
#     so the tests see what the runner sees.
#   * The updater's `apply` suite races its own Busy lock under coverage instrumentation, failing a
#     different test each run; coverage runs with --test-threads=1.
# Needs their build deps (ci.yml): libudev-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
# libgstreamer-plugins-bad1.0-dev, and `cargo install cargo-llvm-cov` + `rustup component add
# llvm-tools-preview`. Refuses to switch refs on a dirty tree.
set -uo pipefail
co="${1:?checkout path}"; base="${2:-origin/main}"
cd "$co" || exit 2
export RUSTFLAGS="-D warnings"
export DBUS_SYSTEM_BUS_ADDRESS="unix:path=/nonexistent/system_bus_socket"
[ -z "$(git status --porcelain)" ] || { echo "tree is dirty: commit or stash before switching refs"; exit 2; }
here="$(git rev-parse --abbrev-ref HEAD)"
tmp="$(mktemp -d)"
sum() { echo "passed $(grep -o '[0-9]* passed' "$1" | awk '{s+=$1} END {print s+0}')  failed $(grep -o '[0-9]* failed' "$1" | awk '{s+=$1} END {print s+0}')  ignored $(grep -o '[0-9]* ignored' "$1" | awk '{s+=$1} END {print s+0}')  binaries $(grep -c '^test result' "$1")"; }
cov() { cargo llvm-cov --workspace --summary-only --ignore-filename-regex '(^|/)xtask/' --fail-under-lines 72 -- --test-threads=1 2>&1 | grep -E "^TOTAL|^test .* FAILED|error:" | tail -3; }
echo "== $here: cargo fmt --all --check"; cargo fmt --all --check && echo "fmt clean" || echo "FMT FAILED"
echo "== $here: cargo clippy --workspace --all-targets (-D warnings)"; cargo clippy --workspace --all-targets 2>&1 | grep -E "^(warning|error)" | head -10; echo "clippy rc=${PIPESTATUS[0]}"
echo "== $here: cargo test --workspace"; cargo test --workspace > "$tmp/branch.txt" 2>&1; echo "rc=$?"; sum "$tmp/branch.txt"; grep -E "^test .* FAILED" "$tmp/branch.txt" | head -10
echo "== $here: coverage (floor 72 % lines)"; cov
echo "== $base: cargo test --workspace"; git checkout -q --detach "$base"; cargo test --workspace > "$tmp/base.txt" 2>&1; echo "rc=$?"; sum "$tmp/base.txt"
echo "== $base: coverage"; cov
git checkout -q "$here"
echo "logs in $tmp"
