"""Tests for the command-line half of the Modal runner, which needs no Modal install."""

import subprocess
from datetime import datetime
from pathlib import Path, PurePosixPath

import pytest
from sim_cli import (
    git_output,
    local_path_for,
    parse_arguments,
    plan_from_arguments,
    plan_with_git_state,
    repository_root,
)

WIND_NAMES = ['calm', 'gusty']
NOW = datetime(2026, 11, 20, 12, 0, 0)


def test_defaults_describe_a_ten_minute_mission_on_eight_cores():
    arguments = parse_arguments([])

    assert (arguments.mode, arguments.cpu, arguments.memory_gib, arguments.flight_seconds) == (
        'mission',
        8.0,
        16.0,
        600,
    )


def test_defaults_spend_nothing():
    assert parse_arguments([]).confirm_spend is False


def test_defaults_fly_the_simulator_pose_in_calm_air_without_a_gpu():
    arguments = parse_arguments([])

    assert (arguments.nav_source, arguments.wind, arguments.gpu) == ('sim', 'calm', None)


def test_fetch_and_a_launch_flag_together_are_refused():
    with pytest.raises(SystemExit):
        parse_arguments(['--fetch', 'some_run', '--confirm-spend'])


def test_plan_carries_the_chosen_resources_and_a_dated_run_id():
    arguments = parse_arguments(['--mode', 'probe', '--cpu', '4', '--memory-gib', '12'])

    plan, problems = plan_from_arguments(arguments, WIND_NAMES, NOW, 'ab12')

    assert (plan.mode, plan.cpu_cores, plan.memory_gib, plan.run_id, problems) == (
        'probe',
        4.0,
        12.0,
        '2026-11-20_12-00-00_probe_sim_calm_ab12',
        [],
    )


def test_plan_reports_a_wind_missing_from_the_profiles_file():
    arguments = parse_arguments(['--wind', 'storm'])

    _, problems = plan_from_arguments(arguments, WIND_NAMES, NOW, 'ab12')

    assert problems == ["wind 'storm' is not in wind_profiles.yaml (calm, gusty)"]


def test_plan_reports_too_little_memory():
    arguments = parse_arguments(['--memory-gib', '4'])

    _, problems = plan_from_arguments(arguments, WIND_NAMES, NOW, 'ab12')

    assert problems == ['memory_gib 4.0 is outside 8 to 256']


def test_a_file_inside_the_run_lands_under_the_destination(tmp_path):
    assert (
        local_path_for('run_a/eval/score.json', 'run_a', tmp_path)
        == tmp_path / 'eval' / 'score.json'
    )


def test_a_leading_slash_from_the_volume_listing_is_accepted(tmp_path):
    assert local_path_for('/run_a/launch.log', 'run_a', tmp_path) == tmp_path / 'launch.log'


def test_a_file_from_another_run_is_skipped(tmp_path):
    assert local_path_for('run_b/launch.log', 'run_a', tmp_path) is None


def test_a_path_climbing_out_of_the_run_is_skipped(tmp_path):
    assert local_path_for('run_a/../run_b/launch.log', 'run_a', tmp_path) is None


def test_the_run_directory_itself_is_not_a_file(tmp_path):
    assert local_path_for('run_a', 'run_a', Path(tmp_path)) is None


def test_a_backslash_climb_that_windows_would_follow_is_skipped(tmp_path):
    assert local_path_for('run_a/..\\..\\x', 'run_a', tmp_path) is None


def test_a_drive_letter_that_windows_would_treat_as_absolute_is_skipped(tmp_path):
    assert local_path_for('run_a/C:x', 'run_a', tmp_path) is None


def test_repository_root_is_two_levels_above_the_script_folder():
    script = PurePosixPath('/home/pilot/CINE/scripts/cloud/run_sim_modal.py')

    assert repository_root(script) == PurePosixPath('/home/pilot/CINE')


def test_repository_root_is_none_where_modal_places_the_script_in_the_container():
    assert repository_root(PurePosixPath('/root/run_sim_modal.py')) is None


def test_a_git_call_that_times_out_reads_as_unknown(monkeypatch, tmp_path):
    def hang(*arguments, **keywords):
        raise subprocess.TimeoutExpired('git', keywords['timeout'])

    monkeypatch.setattr(subprocess, 'run', hang)

    assert git_output(['status', '--porcelain'], tmp_path) is None


def test_a_git_call_that_exits_nonzero_reads_as_unknown(monkeypatch, tmp_path):
    # The temp folder may itself sit inside some git repository; stop git searching above it.
    monkeypatch.setenv('GIT_CEILING_DIRECTORIES', str(tmp_path.parent))

    assert git_output(['rev-parse', 'HEAD'], tmp_path) is None


def test_a_git_call_is_given_a_timeout_of_thirty_seconds(monkeypatch, tmp_path):
    seen = {}

    def record(*arguments, **keywords):
        seen.update(keywords)

        return subprocess.CompletedProcess(arguments, 0, stdout='abc\n')

    monkeypatch.setattr(subprocess, 'run', record)
    git_output(['rev-parse', 'HEAD'], tmp_path)

    assert seen['timeout'] == 30


def test_a_plan_made_where_git_fails_has_no_commit_and_no_dirty_flag(monkeypatch, tmp_path):
    monkeypatch.setenv('GIT_CEILING_DIRECTORIES', str(tmp_path.parent))
    arguments = parse_arguments([])
    plan, _ = plan_from_arguments(arguments, WIND_NAMES, NOW, 'ab12')

    stamped = plan_with_git_state(plan, tmp_path)

    assert (stamped.git_commit, stamped.git_dirty) == (None, None)
