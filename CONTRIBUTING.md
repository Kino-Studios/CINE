# Contributing to CINE

Questions and bug reports go in [GitHub issues](https://github.com/Kino-Studios/CINE/issues).
Pull requests are welcome too, and contributions are accepted under GPL-3.0-only, the
licence of this repository.

## Checks

Run these before opening a pull request. CI runs the pre-commit hooks and `pytest`, but not
`colcon test`.

```bash
pip install pre-commit && pre-commit install
pytest                              # pure-Python tests, no ROS needed
python3 tools/format.py --check     # whitespace convention
cd ~/cine_ws && colcon test --packages-select cine_config cine_agent cine_mlp_correction
```

## Style

Python follows ruff (line length 100, single quotes, Python 3.12) and then
`python3 tools/format.py`. Use descriptive names and keep comments for the non-obvious.
Tunable constants live as JSON in `cine_config/config/`, not in node source files.

## Third-party code

Changes under `third_party/` are recorded in `third_party/ego_planner_swarm/UPSTREAM.md`.
