"""The Gazebo launch runs each sensing node it needs, and no second copy of the detector."""

import ast
from pathlib import Path

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'gazebo_sim.launch.py'


def _launched_executables():
    return [
        keyword.value.value
        for call in ast.walk(ast.parse(LAUNCH_FILE.read_text()))
        if isinstance(call, ast.Call) and getattr(call.func, 'id', None) == 'Node'
        for keyword in call.keywords
        if keyword.arg == 'executable' and isinstance(keyword.value, ast.Constant)
    ]


def test_sim_runs_the_detector_that_publishes_detections():
    assert 'bb_extractor' in _launched_executables()


def test_sim_never_runs_the_log_only_detector_on_the_same_camera():
    # yolo_camera runs the same model on the same image and only logs what it sees; in the
    # cloud sim it cost about one CPU core that the simulator needed.
    assert 'yolo_camera' not in _launched_executables()
