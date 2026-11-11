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

## Command-line tool

The `cine` command (package `cine_cli`) runs on the ground PC and the Mac. Install it from
the repository root with pipx, not plain `pip`, because Ubuntu 24.04 refuses `pip` into the
system Python (`sudo apt install pipx` if pipx is missing):

```bash
pipx install --editable ./cine_cli
pipx inject cine-cli ../CINE-Interfaces/cine_interfaces   # needed by `contract check`
cine --help                                               # lists every command
```

The editable install means `git pull` updates the tool without reinstalling it. The
`inject` line adds the CINE-Interfaces package to the same environment, so `contract check`
can import it.

## Topics shared with CINE-Sensing

Topics that cross the Wi-Fi link to the drone's Raspberry Pi are listed in the contract in
[CINE-Interfaces](https://github.com/Kino-Studios/CINE-Interfaces), and
`tools/test_interface_contract.py` fails when this repository drifts from it. Install the
checker next to this checkout before running `pytest`:

```bash
git clone https://github.com/Kino-Studios/CINE-Interfaces ../CINE-Interfaces
pip install -e ../CINE-Interfaces/cine_interfaces
```

To rename or retype one of those topics, edit `contract.json` in CINE-Interfaces first,
then change the code here and in CINE-Sensing, and move the commit pinned in
`interfaces.repos` and `.github/workflows/ci.yml` forward.

## Style

Python follows ruff (line length 100, single quotes, Python 3.12) and then
`python3 tools/format.py`. Use descriptive names and keep comments for the non-obvious.
Tunable constants live as JSON in `cine_config/config/`, not in node source files.

## Third-party code

Changes under `third_party/` are recorded in `third_party/ego_planner_swarm/UPSTREAM.md`.
