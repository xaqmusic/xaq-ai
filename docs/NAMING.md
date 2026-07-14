# Naming: `ami_ogma` == `ogma` == Zanshin

Zanshin began as an internal project codenamed **AMI-Ogma** ("ogma" /
"ami_ogma") and was renamed **Zanshin** for its public release. To avoid
churning stable, hard-to-migrate identifiers, the following intentionally keep
the old name and are **not** bugs:

- The C++ namespace `ami_ogma::` (all of `cpp_core/`).
- The Godot addon path `res://addons/ami_ogma/`, the built extension
  `ami_ogma_host.so`, and the GDExtension entry symbol `ami_ogma_library_init`.
- Environment variables and config keys prefixed `OGMA_` (e.g. `OGMA_CELL_CONFIG`).

If you see `ami_ogma` or `ogma` anywhere in this repo, read it as **Zanshin** —
they refer to the same project. New Python code uses the `zanshin` /
`zanshin_core` packages; there is no plan to rename the C++/Godot identifiers.
