# Third-Party Notices

xaq depends on the third-party components listed below. Each remains the
property of its respective authors and is governed by its own license, not by
xaq's Apache-2.0 license. This file is a starting inventory; verify and
complete it (versions, license texts) before any public release.

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
| `sha256.h` | *verify* | `cpp_core/third_party/sha256.h` — confirm origin/license before release |

## Godot host (`godot_host/`)

| Component | License | Source |
|---|---|---|
| godot-cpp | MIT | https://github.com/godotengine/godot-cpp (fetched at build time) |
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

> **Action before release:** PyQt6 is GPL-3.0 (or commercial). It is confined to
> the optional `[ui]` extra and is not required to run the engine, but any
> distributed build that bundles the Qt UI must comply with its terms.
