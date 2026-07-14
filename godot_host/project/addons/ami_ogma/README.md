# addons/ami_ogma — Zanshin Godot GDExtension

This addon is the Godot 4.6 GDExtension host for the Zanshin runtime.

## Why is this called `ami_ogma`?

`ami_ogma` is Zanshin's **original internal codename**. This addon directory,
the built extension `ami_ogma_host.so`, and the GDExtension entry symbol
`ami_ogma_library_init` keep the old name so existing scenes and saved configs
that reference `res://addons/ami_ogma/...` keep working. `ami_ogma` == `ogma` ==
**Zanshin** — same project. See [../../../../docs/NAMING.md](../../../../docs/NAMING.md).

## Building the extension

`ami_ogma_host.so` is **not committed** — rebuild it from source:

```sh
cmake -S godot_host -B godot_host/build && cmake --build godot_host/build -j
```

The build copies the `.so` into this directory. Godot will fail to load the
extension until you have run the build once.
