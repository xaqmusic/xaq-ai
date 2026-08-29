# Patches

A patch is one instrument: which brain signals are sources, which oscillator each drives,
through what normalisation, into which synthesis destination. `xaq_voice --config <file>`
loads one; the studio's **Save As…** writes one.

This directory is deliberately empty of patches.

`xaq_voice` with no `--config` discovers what the running brain publishes and builds a
patch from it, so nothing here is needed to make sound. What is worth keeping is a patch
somebody **tuned by ear** for a particular brain — and that is a listening decision, not
something that can be derived. Save yours here, named for the config it was tuned
against (`picrawler_native_measured.json`, `cell_corridor.json`), so the next session can
hear what you heard.

A patch is small, diffable JSON. Treat it the way the arena configs are treated: an
artifact of a session, worth the commit.
