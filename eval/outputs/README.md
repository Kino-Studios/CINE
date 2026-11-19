# Eval outputs

Committed here: `results.csv` (one line per results row and metric), `manifest.json` (for every
row: the bag digests, the commit and dirty flag each flight ran on, the image ID, the commit the
scorer ran on, and the exact command), and any hand-picked figures or videos. Run directories
live in `eval/runs/`, which git ignores.

Regenerate `results.csv`, `manifest.json` and the generated table in `docs/results.md` with
`cine eval standings`, after listing the rows in `eval/standings.toml`. Edit the manifest, never
the generated files.
