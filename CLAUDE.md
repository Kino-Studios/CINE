# CINE — Agent Instructions

## Formatting (run after every iteration)

After any change to Python files, run the repository formatter before considering the
work done:

```bash
python3 tools/format.py            # formats all first-party tracked python in the repo
# or target specific files:
python3 tools/format.py path/to/changed_file.py
```

`tools/format.py` enforces the approved whitespace convention that ruff cannot: a blank
line before every `return`/`raise` that is not first in its block, no blank line directly
after a `def`/`class` header, and single blank lines inside bodies. It also runs `ruff`
(quotes, commas, line length, imports) when ruff is on PATH. The committed output is
stable across repeated runs (the per-run "formatted" print can be a cosmetic false
positive when ruff and the blank-line rule touch the same line; the on-disk content does
not change between passes).

### Scope — first-party only

The formatter excludes vendored upstream code via `_EXCLUDE_PARTS`. In particular
`third_party/` (the upstream ego-planner tree, which bundles third-party libraries such
as `odeint`) is NOT formatted, so it stays diffable against upstream. Format only
first-party code: `cine_agent`, `cine_bringup`, `cine_config`, `cine_depth`,
`cine_mlp_correction`, and `tools`. `python3 tools/format.py --check` reports files that
would change without writing them (exit 1 if any).

The remaining convention rules are review-time judgment, not automated:

- **Sparse comments** — remove banner/"what" comments; keep non-obvious "why" comments.
- **Descriptive names** — no one-letter locals (`e`→`exc`, `f`→`file_handle`); never
  rename public API, ROS topics, params, dict keys, or `self.` attributes.

Verify every touched file still compiles (`python3 -m py_compile <file>`) before finishing.

Constants live as JSON in `cine_config/config/`; code reads them through
`cine_config.constants` (Python) or `cine_config/constants.hpp` (C++), never by editing the
import path. Bare `pytest` from the repository root runs the pure-Python tests. Changes
under `third_party/ego_planner_swarm/` are recorded in its `UPSTREAM.md`.
