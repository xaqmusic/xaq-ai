# Windows compatibility — scoping

**Status:** scoping doc, 2026-08-08. Not started. This project has been built and run on
Linux exclusively; nothing here has been compile-verified on an actual Windows machine — no
Windows environment was available to do so. This is a static read of the actual source,
build system, and tooling, with file:line citations, not a tested build.

**Bottom line:** nothing found is architecturally unfixable. The codebase is mostly portable
already (clean `std::thread`/`std::filesystem` usage, no `fork`/`exec`/`dlopen`/`mmap`
anywhere, the Python side ships proper Windows wheels for every dependency). The gaps are a
short, concrete list: one unguarded X11 dependency that blocks compilation outright, a
`.gdextension` file with no Windows entries, a hardcoded `.so` filename in the CMake copy
step, ZeroMQ's pkg-config discovery, and a POSIX-socket control channel that compiles but
goes silently dead on Windows.

---

## 1. Blockers — won't compile / won't load as-is

| # | What | Where | Fix shape |
|---|---|---|---|
| 1 | `X11WindowCaptureStream` and its `#include <X11/Xlib.h>`/`<X11/Xutil.h>` are **completely unguarded** — no `#ifdef` at all — and it is the *only* implementation of `create_window_video`, called unconditionally from both the OpenCV and no-OpenCV branches. Stops the whole `ogma_infra` library from compiling on Windows. | `cpp_core/src/video_capture.cpp:7-8,13-135,262-264,282-285` | Wrap the class + includes in `#ifdef __linux__` / `X11_FOUND`, add a Windows stub (throw "not implemented", or a real GDI/DXGI capture later). CMake's own `find_package(X11 QUIET)` is already correctly optional (`CMakeLists.txt:73-76,112-114`) — the bug is only in the `.cpp`, never having been taught to check it. |
| 2 | `ami_ogma_host.gdextension` declares only `linux.debug.x86_64` / `linux.release.x86_64`. Even a working `.dll` has no key to resolve on Windows. | `godot_host/project/addons/ami_ogma/ami_ogma_host.gdextension:11-12` | Add `windows.debug.x86_64` / `windows.release.x86_64` pointing at `ami_ogma_host.dll`, mirroring the existing Linux entries exactly. |
| 3 | The post-build copy step hardcodes the destination filename as the literal string `"ami_ogma_host.so"` — not a portable generator expression. | `godot_host/CMakeLists.txt:93-98` (specifically the literal at line 96) | Use `$<TARGET_FILE_NAME:ami_ogma_host>` (or an explicit platform-conditional destination) instead of the hardcoded `.so`. The *source* side (`$<TARGET_FILE:ami_ogma_host>`) is already portable — only the destination name needs fixing. |
| 4 | ZeroMQ discovery is `find_package(PkgConfig REQUIRED)` + `pkg_check_modules(ZMQ REQUIRED libzmq)`. pkg-config isn't native to MSVC toolchains. | `cpp_core/CMakeLists.txt:68-69`, consumed at `:245-247` | Works largely unmodified under MSYS2/MinGW+pkgconf (a real `.pc` file ships there). The MSVC/vcpkg path needs an actual `if(WIN32) find_package(ZeroMQ CONFIG REQUIRED) ... else() pkg_check_modules(...) endif()` branch — vcpkg's `zeromq` port is a CMake *config package* (`libzmq`/`libzmq-static` imported targets), a different API surface from `ZMQ_LIBRARIES`/`ZMQ_INCLUDE_DIRS`. `-DCMAKE_TOOLCHAIN_FILE=vcpkg.cmake` alone does not make `pkg_check_modules` itself work. |
| 5 | `Replay.cpp` calls unguarded `::stat()`/`struct stat`. MSVC doesn't expose plain `stat()` (it's `_stat`). | `cpp_core/src/ogma/golden/Replay.cpp:5,28` | Compiles fine under MinGW; needs an `_stat` alias/guard for MSVC specifically. Minor, low-traffic file. |

## 2. Compiles, but silently dead on Windows

| # | What | Where | Fix shape |
|---|---|---|---|
| 6 | `control_server.cpp` (the inspector's TCP control channel, port 7400) is hand-rolled POSIX BSD sockets (`socket`/`bind`/`listen`/`poll`/`accept`/`send`/`read`/`close`). It already has a non-POSIX branch — but that branch only logs `"Control Server non-POSIX implementation missing!"` and no-ops; `server_fd_` doesn't exist outside the POSIX branch. The inspector cannot talk to the brain at all on Windows. | `cpp_core/include/control_server.hpp:11-13`, `cpp_core/src/control_server.cpp:7-9,63-64` and the whole POSIX-guarded implementation | Real fix, not a guard tweak: either a genuine Winsock2 implementation (near-identical API — `close`→`closesocket`, `SOCKET` type, `WSAStartup`/`WSACleanup`), or — cleaner given `DiagPublisher` already uses ZeroMQ successfully elsewhere in this same codebase — rewrite this channel onto `zmq_bind("tcp://...")` too, so there is one portable transport instead of two (one portable, one POSIX-only). |
| 7 | Window capture (X11) has no Windows equivalent implemented — once (1) is fixed with a guard + stub, this becomes a documented feature gap rather than a compile blocker. | same as §1.1 | Out of scope unless someone wants DXGI/GDI screen capture on Windows; not required for the core brain/GDExtension/inspector loop. |

## 3. Mechanical, needed, not hard

| # | What | Where |
|---|---|---|
| 8 | 8 harness scripts hardcode the bare binary name `"godot4"` (no `.exe`) as a `subprocess.run([...])` list argument, and default their scratch directory to `/tmp/xaq_*`. | `godot_host/project/scripts_tools/seedavg.py:19,39-41` and identically in `arenaavg.py:30,54`, `forgetavg.py:46,71`, `gaitalign.py:49,77`, `humpavg.py:20,34`, `lesionavg.py:42,66`, `recoveravg.py:29,43`, `robustavg.py:37,87` |
| 9 | `cpp_adapter.py` hardcodes the `ami-ogma-v3` binary name with no `.exe` handling. | `python/xaq/xaq/cpp_adapter.py:33-36,254-272` |
| 10 | `tools/run_inspector.sh` is the *only* shell script in the repo — bash-specific (`set -euo pipefail`, `${BASH_SOURCE[0]}`, `${PYTHONPATH:+:$PYTHONPATH}`). No native Windows port exists; a `.ps1` would need to be hand-written, not translated line-for-line, since it also assumes `.venv/bin/` (POSIX layout) rather than `.venv/Scripts/`. | `tools/run_inspector.sh` (all 34 lines) |
| 11 | Zero Windows-specific setup instructions anywhere. No `.venv\Scripts\activate`, no `.bat`/`.ps1`, nothing — Windows users get WSL/Git Bash only by implication, never stated. | `AGENTS.md`, `CLAUDE.md`, `tools/xaq_inspector/README.md` |

## 4. Already clean — verified, not assumed

- **Threading**: `std::thread`/`std::mutex`/`std::atomic` throughout; zero raw `pthread_*` calls in library code. `pthread` only appears as a link dependency on ~60 *test* targets, and `OGMA_BUILD_TESTS` is forced `OFF` when `cpp_core` is pulled in as a GDExtension subdirectory (`godot_host/CMakeLists.txt:14`) — so the `.dll` build path never touches this.
- **No `fork`/`exec`/`posix_spawn`/`dlopen`/`dlsym`/`mmap` anywhere** in `cpp_core/` or `godot_host/src/`.
- **Path handling** is clean — `std::filesystem` in C++, `os.path.join` in Python throughout; no raw `/`-concatenation or hardcoded `/tmp`/`/proc` roots outside the harness scripts already listed in §3.
- **Eigen, nlohmann/json, GoogleTest** — header-only or fully cross-platform FetchContent deps; `gtest_force_shared_crt ON` (`cpp_core/CMakeLists.txt:51`) is already the correct flag to avoid MSVC CRT-mismatch link errors.
- **`extension_api.json` / `gdextension_interface.h`** (the committed Godot 4.6.2 API dump godot-cpp is overridden with) are **not** OS-specific content — `gdextension_interface.h` is pure C99 with no OS `#ifdef`s, and `extension_api.json`'s only build-variant axis is float/double × 32/64-bit pointer width, not platform. These do **not** need re-dumping from a Windows Godot binary; the Linux-dumped files should work unmodified for a Windows x86_64 single-precision build.
- **Python dependencies** (`numpy`, `torch`, `scipy`, `pyzmq`, `python-socketio`, `PyQt6`, `pyqtgraph`) all ship official Windows wheels on PyPI. No compiled-from-source assumption, no Linux-only Qt platform-plugin reference (`xcb`/`wayland`) anywhere in the codebase. Plain `pip install` should work as-is.

## 5. Producing an actual `.dll`

Real-world GDExtension projects — including godot-cpp's own CI — build **natively per-OS**,
not cross-compiled. A `windows-latest` GitHub Actions runner with MSVC (or Windows-hosted
MinGW) is the standard, low-friction path. Cross-compiling from Linux via MinGW-w64 is
possible in principle for pure C++, but every FetchContent'd dependency (Eigen, nlohmann/json,
GoogleTest, godot-cpp, plus ZeroMQ's pkg-config discovery specifically) would need to
configure/build cleanly under the cross toolchain too — fragile, and not how this ecosystem
is actually built in practice.

**One concrete risk worth flagging ahead of time:** this project's generated godot-cpp file
tree already reaches 163 characters on a *short* Linux checkout path (e.g.
`.../godot_host/build/_deps/godot_cpp-build/CMakeFiles/godot-cpp.dir/gen/src/classes/open_xr_spatial_capability_configuration_plane_tracking.cpp.o`,
measured directly on this machine). A deeper Windows checkout path, or MSVC/Ninja's own
extra intermediate-directory segments, could realistically push the longest-named generated
files past the 260-character `MAX_PATH` default — a well-known, real CMake-on-Windows failure
mode, not hypothetical here. Mitigation: enable Windows long-path support
(`LongPathsEnabled`, supported since Windows 10 1607 and honored by modern
CMake/MSBuild/git), and/or build from a short root path (CI runners' default short work
directories already help with this).

## 6. Suggested order, if this gets picked up

1. §1.1 (X11 guard) and §1.2–1.3 (`.gdextension` + CMake copy path) first — these three
   together are what actually gets a `.dll` built and loadable at all.
2. §1.4 (ZeroMQ/vcpkg branch) — needed for the same first build, likely done alongside #1.
3. §2.6 (control server) — needed before the inspector is usable on Windows, but the brain
   itself can build and run headless without it.
4. §3 (harness scripts, `run_inspector.sh`, docs) — needed for a Windows *user*, not for the
   build itself; can follow once the build path is proven.
5. A `windows-latest` CI job, once (1)–(2) land, to keep this from silently rotting again —
   the same way the Linux build has no reason to catch a Windows regression otherwise.
