# Third-Party Notices

xaq depends on the third-party components listed below. Each remains the
property of its respective authors and is governed by its own license, not by
xaq's Apache-2.0 license. Reviewed 2026-08-07 against the actual build
(`cpp_core/CMakeLists.txt`, `godot_host/CMakeLists.txt`, and every
`pyproject.toml`/`requirements.txt` in the tree) — update this file whenever
a dependency is added, removed, or re-pinned.

## C++ runtime (`cpp_core/`)

Fetched at build time via CMake `FetchContent`:

| Component | Version | License | Source |
|---|---|---|---|
| Eigen | 3.4.0 | MPL-2.0 | https://gitlab.com/libeigen/eigen |
| nlohmann/json | 3.11.2 | MIT | https://github.com/nlohmann/json |
| GoogleTest (tests only) | 1.14.0 | BSD-3-Clause | https://github.com/google/googletest |

System dependencies (linked, not vendored):

| Component | License | Notes |
|---|---|---|
| ZeroMQ (libzmq) | MPL-2.0 | Required — inter-EPM message bus |
| OpenCV | Apache-2.0 | Optional — video capture / optical flow |
| X11 (libX11) | MIT | Optional — window capture (Linux) |

Vendored header:

| Component | License | Path |
|---|---|---|
| `sha256.h` | Public domain | `cpp_core/third_party/sha256.h` — minimal single-header SHA-256 (FIPS 180-4); self-attests public domain in its own header comment |

## Godot host (`godot_host/`)

| Component | License | Source |
|---|---|---|
| godot-cpp | MIT | https://github.com/godotengine/godot-cpp, pinned to commit `7e18e40` (fetched at build time) |
| Godot Engine API (`extension_api.json`, `gdextension_interface.h`) | MIT | dumped from Godot 4.6.x |

## Python (`python/`)

| Component | License | Notes |
|---|---|---|
| NumPy | BSD-3-Clause | required |
| PyTorch | BSD-3-Clause | required (engine); optional extra for `xaq_core` |
| SciPy | BSD-3-Clause | required (engine) |
| python-socketio | MIT | substrate bus |
| pyzmq | BSD-3-Clause | substrate bus |
| PyQt6 | GPL-3.0 / commercial | **`[ui]` extra only** — review licensing implications before distributing UI builds |
| pyqtgraph | MIT | `[ui]` extra |

> **PyQt6 is GPL-3.0 (or commercial).** Verified 2026-08-29 by import grep: PyQt6/pyqtgraph
> usage is confined to `python/xaq/xaq/server/ui/`, `python/xaq/xaq/observer/widgets/`,
> `tools/xaq_inspector/`, and `tools/xaq_voice_studio/` — never imported from `xaq_core` or
> the engine core, so running xaq headless pulls in neither. The `tools/xaq_voice` engine
> itself is C++ and links no Qt, so the audio side runs on the Pi without it. Still applies
> to anyone who **distributes** a build bundling the Qt UI: that build must comply with
> GPL-3.0 (or hold a commercial PyQt6 license).

## Tools (`tools/`)

`tools/xaq_inspector/requirements.txt` — a standalone sidecar app, pinned separately from
`python/xaq`'s `[ui]` extra:

| Component | License | Notes |
|---|---|---|
| PyQt6 | GPL-3.0 / commercial | unconditional dependency of this tool (see the PyQt6 note above) |
| pyqtgraph | MIT | |
| pyzmq | BSD-3-Clause | |
| NumPy | BSD-3-Clause | |
