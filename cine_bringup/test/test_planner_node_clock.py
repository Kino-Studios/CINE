import re
from pathlib import Path

PLANNER_ROOT = (
    Path(__file__).resolve().parents[2] / 'third_party' / 'ego_planner_swarm' / 'src' / 'planner'
)
TRAJECTORY_TIMING_FILES = (
    'plan_manage/src/traj_server.cpp',
    'plan_manage/src/planner_manager.cpp',
    'plan_manage/src/ego_replan_fsm.cpp',
    'bspline_opt/src/bspline_optimizer.cpp',
    'traj_utils/include/traj_utils/plan_container.hpp',
)
# Planner compute-time variables: wall time is right for them, both readings share one clock.
COMPUTE_TIMING_ASSIGNMENT = re.compile(
    r'\b(t_start|t_opt|t_refine|t0|t1|t2|now)\s*=\s*rclcpp::Clock\(\)\.now\(\)',
)
OWN_CLOCK = re.compile(r'rclcpp::Clock(?:\s+\w+)?\s*[({]')
SYSTEM_TIME_TYPE = re.compile(r'RCL_SYSTEM_TIME')
DEFAULT_CONSTRUCTED_TIME = re.compile(r'rclcpp::Time\s+\w+(?:,\s*\w+)*;')


def _code_lines(relative_path):
    lines = (PLANNER_ROOT / relative_path).read_text().splitlines()

    return [
        (number, line.split('//')[0])
        for number, line in enumerate(lines, start=1)
        if line.split('//')[0].strip()
    ]


def _lines_matching(pattern, files=TRAJECTORY_TIMING_FILES):
    return [
        f'{relative_path}:{number}: {line.strip()}'
        for relative_path in files
        for number, line in _code_lines(relative_path)
        if pattern.search(line)
    ]


def test_trajectory_timing_does_not_build_its_own_clock():
    # A standalone rclcpp::Clock never attaches to /clock and reads wall time under sim time;
    # this catches any trajectory stamp or sample that bypasses the node's clock.
    offenders = [
        entry for entry in _lines_matching(OWN_CLOCK) if not COMPUTE_TIMING_ASSIGNMENT.search(entry)
    ]

    assert offenders == []


def test_trajectory_timing_does_not_use_the_system_time_type():
    # Subtracting a system-type time from a ros-type time throws at run time, and a
    # system-type stamp is wall time even when use_sim_time is true.
    assert _lines_matching(SYSTEM_TIME_TYPE) == []


def test_trajectory_time_members_are_not_default_constructed():
    # A default-constructed rclcpp::Time has the system time type. Subtracting it from a
    # node-clock reading throws, so every stored trajectory time must name its ros type.
    assert _lines_matching(DEFAULT_CONSTRUCTED_TIME) == []
