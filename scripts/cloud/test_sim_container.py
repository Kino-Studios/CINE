import json
import os
import select
import sys
import textwrap
import time
from dataclasses import replace
from pathlib import Path

import pytest
from sim_container import (
    SAMPLED_TOPICS,
    ContainerPaths,
    FlightLimits,
    ProbeStep,
    Timing,
    apply_frame_check,
    child_environment,
    deep_sample_script,
    find_bag,
    flight_limits,
    fly,
    gate_label,
    judge_frame,
    lowest_headroom,
    probe_steps,
    render_check,
    run_captured,
    run_container,
    scan_log,
    score_bag,
    storage_id_for,
    udp_counters_text,
)
from sim_plan import SimRunPlan

REPO_ROOT = Path(__file__).resolve().parents[2]
posix_only = pytest.mark.skipif(
    sys.platform == 'win32',
    reason='the driver uses POSIX process groups',
)

# Stands in for the simulator stack. The first argument picks a behaviour; the bag
# directory comes from FAKE_BAGS so the SIGINT handler can finish a bag like rosbag2 does.
FAKE_STACK = textwrap.dedent(
    """\
    import os, signal, sys, time
    from pathlib import Path

    behaviour = sys.argv[1] if len(sys.argv) > 1 else os.environ.get('FAKE_BEHAVIOUR', 'fly')
    bags = Path(os.environ.get('FAKE_BAGS', '.'))

    def launched():
        print('[INFO] [launch]: All log files can be found below /root/.ros/log', flush=True)

    def finish_bag(signum, frame):
        bag = bags / '2026-11-20_12-00-00_sim_cine_field_calm'
        bag.mkdir(parents=True, exist_ok=True)
        (bag / 'bag_0.mcap').write_bytes(b'mcap')
        if behaviour != 'unfinished_bag':
            (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}')
        print("[ERROR] [cost_node-7]: process has died [pid 5, exit code 1, cmd 'x'].", flush=True)
        sys.exit(0)

    if behaviour == 'die_early':
        print('colcon build failed', flush=True)
        sys.exit(2)
    if behaviour == 'never_launch':
        time.sleep(60)
    if behaviour == 'exit_after_launch':
        launched()
        time.sleep(0.3)
        sys.exit(0)
    if behaviour == 'ignore_sigint':
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        launched()
        time.sleep(60)
    if behaviour == 'slow_launch':
        time.sleep(1.0)
    if behaviour == 'crash_then_fly':
        launched()
        print("[ERROR] [vins_node-5]: process has died [pid 41, exit code -11, cmd 'v'].", flush=True)
    else:
        launched()
    signal.signal(signal.SIGINT, finish_bag)
    time.sleep(60)
    """,
)


def _write_fake_stack(directory):
    script = directory / 'fake_stack.py'
    script.write_text(FAKE_STACK)

    return script


def _limits(**overrides):
    fields = {
        'flight_seconds': 1.0,
        'deadline': time.monotonic() + 30,
        'memory_stop_gib': 1.0,
        'poll_seconds': 0.05,
        'interrupt_grace_seconds': 5.0,
        'terminate_grace_seconds': 5.0,
    }
    fields.update(overrides)

    return FlightLimits(**fields)


def _fly(tmp_path, behaviour, limits, **keywords):
    script = _write_fake_stack(tmp_path)
    environment = {'PATH': '/usr/bin:/bin', 'FAKE_BAGS': str(tmp_path / 'bags')}

    return fly(
        [sys.executable, str(script), behaviour],
        environment,
        tmp_path / 'launch.log',
        limits,
        **keywords,
    )


@posix_only
def test_flight_is_interrupted_once_the_flight_time_has_passed(tmp_path):
    report = _fly(tmp_path, 'fly', _limits())

    assert report.launch_started
    assert report.stopped_by == 'flight_time'
    assert report.ended_by_signal == 'SIGINT'


@posix_only
def test_interrupted_flight_lets_the_stack_finish_its_bag(tmp_path):
    _fly(tmp_path, 'fly', _limits())

    assert (
        tmp_path / 'bags' / '2026-11-20_12-00-00_sim_cine_field_calm' / 'metadata.yaml'
    ).exists()


@posix_only
def test_stack_that_dies_before_launch_is_reported_with_its_exit_code(tmp_path):
    report = _fly(tmp_path, 'die_early', _limits())

    assert not report.launch_started
    assert report.stack_exit_code == 2
    assert report.stopped_by is None


@posix_only
def test_stack_that_ignores_sigint_is_terminated(tmp_path):
    report = _fly(tmp_path, 'ignore_sigint', _limits(interrupt_grace_seconds=0.5))

    assert report.ended_by_signal == 'SIGTERM'


@posix_only
def test_deadline_stops_a_stack_that_never_launches(tmp_path):
    report = _fly(tmp_path, 'never_launch', _limits(deadline=time.monotonic() + 0.5))

    assert report.stopped_by == 'deadline'
    assert not report.launch_started


@pytest.mark.parametrize(
    'topic',
    [topic for topic in SAMPLED_TOPICS if topic.startswith('/cinematography/')],
)
def test_every_sampled_cinematography_topic_is_named_by_the_stack(topic):
    # A renamed topic would otherwise sample silence and read as a dead publisher.
    sources = [
        *(REPO_ROOT / 'cine_agent' / 'cine_agent').rglob('*.py'),
        *(REPO_ROOT / 'cine_gazebo').rglob('*.py'),
    ]

    assert any(f"'{topic}'" in source.read_text(encoding='utf-8') for source in sources)


@posix_only
def test_low_memory_stops_the_flight_early(tmp_path):
    report = _fly(tmp_path, 'fly', _limits(flight_seconds=30.0), read_headroom=lambda: 0.4)

    assert report.stopped_by == 'memory'
    assert report.lowest_headroom_gib == 0.4


@posix_only
def test_low_memory_during_the_build_does_not_stop_the_run(tmp_path):
    # Before launch no bag exists to protect, and a compile spike is expected.
    report = _fly(
        tmp_path,
        'never_launch',
        _limits(deadline=time.monotonic() + 0.5),
        read_headroom=lambda: 0.4,
    )

    assert report.stopped_by == 'deadline'


@posix_only
def test_a_build_spike_does_not_stop_the_flight_once_memory_recovers(tmp_path):
    # The stack launches after 1 s; memory is low only for the first 0.5 s.
    started = time.monotonic()
    report = _fly(
        tmp_path,
        'slow_launch',
        _limits(),
        read_headroom=lambda: 0.4 if time.monotonic() - started < 0.5 else 5.0,
    )

    assert report.stopped_by == 'flight_time'
    assert report.lowest_headroom_gib == 0.4


@posix_only
def test_without_a_flight_time_the_driver_waits_for_the_stack_to_exit(tmp_path):
    report = _fly(tmp_path, 'exit_after_launch', _limits(flight_seconds=None))

    assert report.stack_exit_code == 0
    assert report.stopped_by is None


@posix_only
def test_midflight_callback_runs_once(tmp_path):
    calls = []

    _fly(tmp_path, 'fly', _limits(), on_midflight=lambda: calls.append('sampled'))

    assert calls == ['sampled']


@posix_only
def test_log_offset_at_stop_separates_shutdown_exits_from_crashes(tmp_path):
    report = _fly(tmp_path, 'fly', _limits())
    scan = scan_log(tmp_path / 'launch.log', report.log_offset_at_stop)

    assert 'process has died' not in scan.died_before_stop
    assert 'cost_node' in scan.died_after_stop


@posix_only
def test_flight_time_ends_when_the_stop_is_decided_not_when_shutdown_ends(tmp_path):
    report = _fly(tmp_path, 'ignore_sigint', _limits(interrupt_grace_seconds=2.0))

    assert report.flight_seconds < 2.0


def test_log_scan_without_a_stop_counts_every_death_before_it(tmp_path):
    log = tmp_path / 'launch.log'
    log.write_bytes(b"[ERROR] [vins_node-5]: process has died [pid 4, exit code 1, cmd 'v'].\n")

    scan = scan_log(log, None)

    assert (scan.died_before_stop.count('process has died'), scan.died_after_stop) == (1, '')


def test_log_scan_counts_a_death_after_the_smoke_verdict_as_a_shutdown_exit(tmp_path):
    log = tmp_path / 'launch.log'
    log.write_bytes(
        b'PASS: closest distance to the setpoint: autopilot 0.44 m\n'
        b"[ERROR] [vins_node-9]: process has died [pid 4, exit code -6, cmd 'v'].\n",
    )

    scan = scan_log(log, None)

    assert (scan.died_before_stop, scan.died_after_stop.count('vins_node')) == ('', 1)


def test_log_scan_counts_a_death_before_the_smoke_verdict_as_a_crash(tmp_path):
    log = tmp_path / 'launch.log'
    log.write_bytes(
        b"[ERROR] [vins_node-9]: process has died [pid 4, exit code -6, cmd 'v'].\n"
        b'FAIL: not within 1 m inside 60 s of sim time\n',
    )

    scan = scan_log(log, None)

    assert (scan.died_before_stop.count('vins_node'), scan.died_after_stop) == (1, '')


def test_log_scan_keeps_only_the_tail_of_a_long_log(tmp_path):
    log = tmp_path / 'launch.log'
    log.write_bytes(b'x' * 10_000 + b'last line\n')

    assert scan_log(log, None, tail_bytes=10).tail == 'last line\n'


def test_log_scan_survives_bytes_that_are_not_utf8(tmp_path):
    log = tmp_path / 'launch.log'
    log.write_bytes(b'\xff\xfe broken\n')

    assert 'broken' in scan_log(log, None).tail


@posix_only
def test_timed_out_command_output_is_text_not_a_bytes_literal():
    outcome = run_captured(['bash', '-c', 'echo hello; sleep 5'], {'PATH': '/usr/bin:/bin'}, 1)

    assert outcome['exit_code'] == 124
    assert outcome['output'].startswith('hello')


def _ends_within(process_id, seconds):
    # SIGKILL is delivered asynchronously, so the process may still be tearing down
    # when run_captured returns; a pidfd turns readable the moment it has exited.
    try:
        descriptor = os.pidfd_open(process_id)
    except ProcessLookupError:
        return True

    try:
        return bool(select.select([descriptor], [], [], seconds)[0])
    finally:
        os.close(descriptor)


@posix_only
def test_timed_out_command_takes_its_background_children_with_it(tmp_path):
    pid_file = tmp_path / 'child.pid'

    run_captured(
        ['bash', '-c', f'sleep 30 & echo $! > {pid_file}; wait'],
        {'PATH': '/usr/bin:/bin'},
        1,
    )

    assert _ends_within(int(pid_file.read_text()), 5)


@posix_only
def test_finished_command_leaves_no_background_children_behind(tmp_path):
    pid_file = tmp_path / 'child.pid'

    run_captured(
        ['bash', '-c', f'sleep 30 >/dev/null 2>&1 & echo $! > {pid_file}'],
        {'PATH': '/usr/bin:/bin'},
        10,
    )

    assert _ends_within(int(pid_file.read_text()), 5)


def test_missing_program_is_reported_as_exit_127():
    outcome = run_captured(['/nonexistent/program'], {'PATH': '/usr/bin:/bin'}, 5)

    assert outcome['exit_code'] == 127


def test_child_environment_drops_modal_python_paths_and_settings():
    base = {
        'PATH': '/usr/bin',
        'PYTHONPATH': '/root:/__modal/deps',
        'MODAL_TASK_ID': 'ta-1',
        'HOME': '/root',
    }

    environment = child_environment(base, {'WIND': 'calm'})

    assert environment == {'PATH': '/usr/bin', 'HOME': '/root', 'WIND': 'calm'}


def test_lowest_headroom_ignores_missing_readings():
    assert lowest_headroom([None, 3.5, 1.25]) == 1.25


def test_lowest_headroom_without_readings_is_none():
    assert lowest_headroom([None, None]) is None


def _make_bag(bags, name, complete):
    bag = bags / name
    bag.mkdir(parents=True)
    (bag / 'bag_0.mcap').write_bytes(b'mcap')

    if complete:
        (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}')

    return bag


def test_find_bag_reports_the_newest_bag_directory(tmp_path):
    _make_bag(tmp_path, '2026-11-20_10-00-00_sim_cine_field_calm', complete=True)
    newest = _make_bag(tmp_path, '2026-11-20_11-00-00_sim_cine_field_calm', complete=True)

    assert find_bag(tmp_path).path == newest


def test_find_bag_marks_a_bag_without_metadata_incomplete(tmp_path):
    _make_bag(tmp_path, '2026-11-20_11-00-00_sim_cine_field_calm', complete=False)

    assert not find_bag(tmp_path).complete


def test_find_bag_in_an_empty_or_missing_directory_is_none(tmp_path):
    assert find_bag(tmp_path / 'missing') is None


def test_storage_id_of_an_mcap_bag(tmp_path):
    bag = _make_bag(tmp_path, 'bag', complete=False)

    assert storage_id_for(bag) == 'mcap'


def test_storage_id_of_a_sqlite_bag(tmp_path):
    (tmp_path / 'bag_0.db3').write_bytes(b'db')

    assert storage_id_for(tmp_path) == 'sqlite3'


def test_storage_id_of_an_empty_bag_is_none(tmp_path):
    assert storage_id_for(tmp_path) is None


def test_gate_label_is_read_from_the_json_line():
    assert gate_label('noise\n{"label": "REVIEW", "failures": []}\n') == 'REVIEW'


def test_gate_label_of_unreadable_output_is_none():
    assert gate_label('Traceback (most recent call last):\n') is None


def test_probe_steps_have_unique_names_and_bounded_timeouts():
    steps = probe_steps('/weights')
    names = [step.name for step in steps]

    assert len(names) == len(set(names))
    assert all(0 < step.timeout_seconds <= 900 for step in steps)


def test_probe_compares_multicast_and_loopback_discovery():
    names = [step.name for step in probe_steps('/weights')]

    assert {'dds_default', 'dds_loopback', 'gz_transport_default', 'gz_transport_loopback'} <= set(
        names,
    )


def _paths(tmp_path):
    entrypoint = tmp_path / 'entrypoint.sh'
    entrypoint.write_text(
        '#!/usr/bin/env bash\n'
        # Wrapped commands (samples, probe steps) are stubbed so the host's own ROS or
        # Gazebo, if installed, never runs.
        # A reindex writes the metadata a real `ros2 bag reindex` would.
        'if [ "$1" = ros2 ] && [ "$2" = bag ]; then touch "$4/metadata.yaml"; exit 0; fi\n'
        'if [ "$#" -gt 0 ]; then echo "stubbed: $1"; exit 0; fi\n'
        f'exec {sys.executable} {_write_fake_stack(tmp_path)} "${{FAKE_BEHAVIOUR:-fly}}"\n',
    )
    entrypoint.chmod(0o755)
    ros_setup = tmp_path / 'setup.bash'
    ros_setup.write_text('')

    return ContainerPaths(
        entrypoint=str(entrypoint),
        work=tmp_path / 'work',
        bags=tmp_path / 'bags',
        runs=tmp_path / 'runs',
        repo_root=REPO_ROOT,
        colcon_base=tmp_path / 'colcon',
        ros_setup=ros_setup,
        eval_python=sys.executable,
        weights=tmp_path / 'weights',
        xvfb=None,
    )


def _fake_eval(tmp_path):
    script = tmp_path / 'fake_eval.sh'
    # Called as: fake_eval -m cine_eval <step> ... --json
    script.write_text(
        '#!/usr/bin/env bash\n'
        'if [ "$3" = gate ]; then echo \'{"label": "REVIEW", "failures": []}\'; exit 1; fi\n'
        'exit 0\n',
    )
    script.chmod(0o755)

    return str(script)


def _run_plan(**overrides):
    fields = {
        'mode': 'mission',
        'cpu_cores': 8.0,
        'memory_gib': 16.0,
        'gpu': None,
        'flight_seconds': 1,
        'nav_source': 'sim',
        'wind': 'calm',
        'run_id': 'test-run',
    }
    fields.update(overrides)

    return SimRunPlan(**fields)


FAST_TIMING = Timing(
    poll_seconds=0.05,
    interrupt_grace_seconds=5.0,
    terminate_grace_seconds=5.0,
    sample_seconds=0.2,
    memory_stop_gib=0.0,
)


@pytest.fixture
def fake_bags(tmp_path, monkeypatch):
    monkeypatch.setenv('FAKE_BAGS', str(tmp_path / 'bags'))


@posix_only
def test_run_container_saves_the_result_log_plan_and_bag_in_the_run_directory(tmp_path, fake_bags):
    paths = _paths(tmp_path)

    run_container(_run_plan(), paths, FAST_TIMING)
    run_directory = paths.runs / 'test-run'

    assert {'result.json', 'launch.log', 'plan.json', 'samples.jsonl'} <= {
        p.name for p in run_directory.iterdir()
    }
    assert (run_directory / 'bag' / 'metadata.yaml').exists()


@posix_only
def test_run_container_reports_the_crash_before_the_stop_and_not_the_shutdown_exit(
    tmp_path,
    fake_bags,
    monkeypatch,
):
    monkeypatch.setenv('FAKE_BEHAVIOUR', 'crash_then_fly')

    result = run_container(_run_plan(), _paths(tmp_path), FAST_TIMING)

    assert result['status'] == 'process_died'
    assert result['crashes'] == [{'name': 'vins_node', 'exit_code': -11}]
    assert result['shutdown_exits'] == [{'name': 'cost_node', 'exit_code': 1}]


@posix_only
def test_run_container_records_a_scoring_failure_without_losing_the_flight(tmp_path, fake_bags):
    result = run_container(_run_plan(), _paths(tmp_path), FAST_TIMING)

    assert result['status'] == 'ok'
    assert result['scoring']['extract']['exit_code'] != 0
    assert 'score' not in result['scoring']
    assert result['gate'] is None


@posix_only
def test_run_container_for_a_failed_build_keeps_the_log_tail(tmp_path, fake_bags, monkeypatch):
    monkeypatch.setenv('FAKE_BEHAVIOUR', 'die_early')

    result = run_container(_run_plan(), _paths(tmp_path), FAST_TIMING)

    assert result['status'] == 'failed_before_launch'
    assert 'colcon build failed' in result['log_tail']
    assert result['bag'] is None


@posix_only
def test_run_container_writes_the_returned_result_to_result_json(tmp_path, fake_bags):
    paths = _paths(tmp_path)

    result = run_container(_run_plan(), paths, FAST_TIMING)

    assert json.loads((paths.runs / 'test-run' / 'result.json').read_text()) == result


@posix_only
def test_run_container_commits_the_volume_after_saving(tmp_path, fake_bags):
    commits = []

    run_container(
        _run_plan(),
        _paths(tmp_path),
        FAST_TIMING,
        commit=lambda: commits.append('commit'),
    )

    assert commits


@posix_only
def test_gate_review_is_a_verdict_not_a_scoring_failure(tmp_path):
    paths = replace(_paths(tmp_path), eval_python=_fake_eval(tmp_path))

    scoring, label = score_bag(paths, tmp_path, tmp_path / 'eval', time.monotonic() + 600)

    assert (label, scoring['gate']['exit_code']) == ('REVIEW', 1)


@posix_only
def test_gate_label_survives_a_gate_line_longer_than_the_stored_output(tmp_path):
    script = tmp_path / 'long_gate.sh'
    script.write_text(
        '#!/usr/bin/env bash\n'
        'if [ "$3" = gate ]; then\n'
        '  printf \'{"label": "REVIEW", "failures": ["%s"]}\\n\' "$(printf "x%.0s" $(seq 5000))"\n'
        '  exit 1\n'
        'fi\n'
        'exit 0\n',
    )
    script.chmod(0o755)
    paths = replace(_paths(tmp_path), eval_python=str(script))

    _, label = score_bag(paths, tmp_path, tmp_path / 'eval', time.monotonic() + 600)

    assert label == 'REVIEW'


def test_scoring_is_skipped_when_no_time_is_left(tmp_path):
    scoring, label = score_bag(_paths(tmp_path), tmp_path, tmp_path / 'eval', time.monotonic())

    assert (scoring['extract']['skipped'], label) == ('out of time', None)


def test_mission_flight_stops_early_enough_to_shut_down_and_score():
    limits = flight_limits(_run_plan(flight_seconds=600), Timing(), started=1000.0)

    # Allowances: build 3600, weights 900, launch 600, flight 600, shutdown 120, scoring 900.
    assert limits.deadline == 1000.0 + 6720 - 120 - 900


def test_check_mode_waits_out_the_smoke_scripts_own_three_stage_shutdown():
    limits = flight_limits(_run_plan(mode='check'), Timing(), started=0.0)

    # smoke_flight.py waits up to 30 s after each of SIGINT, SIGTERM and SIGKILL.
    assert limits.interrupt_grace_seconds > 90


@posix_only
def test_result_json_is_written_even_when_the_driver_raises(tmp_path, fake_bags, monkeypatch):
    def broken_find_bag(bags_directory):
        raise RuntimeError('disk vanished')

    monkeypatch.setattr('sim_container.find_bag', broken_find_bag)
    paths = _paths(tmp_path)

    run_container(_run_plan(), paths, FAST_TIMING)
    saved = json.loads((paths.runs / 'test-run' / 'result.json').read_text())

    assert saved['status'] == 'driver_error'
    assert 'disk vanished' in saved['driver_error']


@posix_only
def test_bag_left_without_metadata_is_reindexed(tmp_path, fake_bags, monkeypatch):
    monkeypatch.setenv('FAKE_BEHAVIOUR', 'unfinished_bag')

    result = run_container(_run_plan(), _paths(tmp_path), FAST_TIMING)

    assert result['bag'] == {
        'name': '2026-11-20_12-00-00_sim_cine_field_calm',
        'complete': True,
        'reindexed': True,
    }


@posix_only
def test_smoke_check_that_exits_cleanly_passes(tmp_path, fake_bags, monkeypatch):
    monkeypatch.setenv('FAKE_BEHAVIOUR', 'exit_after_launch')

    result = run_container(_run_plan(mode='check'), _paths(tmp_path), FAST_TIMING)

    assert result['status'] == 'check_passed'


@posix_only
def test_samples_record_the_kernel_and_cgroup_memory_readings_apart(tmp_path, fake_bags):
    paths = _paths(tmp_path)

    run_container(_run_plan(), paths, FAST_TIMING)
    first_sample = (paths.runs / 'test-run' / 'samples.jsonl').read_text().splitlines()[0]

    assert {'meminfo_available_gib', 'cgroup_headroom_gib'} <= set(json.loads(first_sample))


@posix_only
def test_result_lists_the_bag_files_saved_with_the_run(tmp_path, fake_bags):
    result = run_container(_run_plan(), _paths(tmp_path), FAST_TIMING)

    assert {'bag/bag_0.mcap', 'bag/metadata.yaml', 'result.json'} <= set(result['files'])


@posix_only
def test_result_json_is_saved_before_the_bag_is_copied(tmp_path, fake_bags, monkeypatch):
    # A container killed during a long bag copy must still leave a result behind.
    paths = _paths(tmp_path)
    result_path = paths.runs / 'test-run' / 'result.json'
    saved_before_copy = []

    def recording_copy(source, destination, warnings):
        if destination.name == 'bag':
            saved_before_copy.append(result_path.exists())

    monkeypatch.setattr('sim_container._copy_tree', recording_copy)

    run_container(_run_plan(), paths, FAST_TIMING)

    assert saved_before_copy == [True]


@posix_only
def test_a_failed_volume_commit_still_returns_the_result(tmp_path, fake_bags):
    def broken_commit():
        raise OSError('volume unavailable')

    result = run_container(_run_plan(), _paths(tmp_path), FAST_TIMING, commit=broken_commit)

    assert result['status'] == 'ok'


@posix_only
def test_a_restarted_attempt_sets_the_earlier_attempt_aside(tmp_path, fake_bags):
    paths = _paths(tmp_path)
    run_directory = paths.runs / 'test-run'
    run_directory.mkdir(parents=True)
    (run_directory / 'samples.jsonl').write_text('{"earlier": true}\n')

    run_container(_run_plan(), paths, FAST_TIMING)

    earlier = run_directory / 'earlier_attempts' / '1' / 'samples.jsonl'
    assert earlier.read_text() == '{"earlier": true}\n'
    assert 'earlier' not in (run_directory / 'samples.jsonl').read_text()


@posix_only
def test_probe_runs_every_step_and_records_each_exit_code(tmp_path):
    result = run_container(_run_plan(mode='probe'), _paths(tmp_path), FAST_TIMING)

    assert result['status'] == 'probe_done'
    assert [step['name'] for step in result['probe']] == [
        step.name for step in probe_steps('/weights')
    ]
    assert all('exit_code' in step for step in result['probe'])


@posix_only
def test_render_check_finds_the_camera_world_installed_by_the_ros_vendored_gazebo(tmp_path):
    vendored_share = tmp_path / 'opt' / 'ros' / 'jazzy' / 'opt' / 'gz_sim_vendor' / 'share'
    world = vendored_share / 'gz' / 'gz-sim8' / 'worlds' / 'camera_sensor.sdf'
    world.parent.mkdir(parents=True)
    world.write_text('<sdf/>')
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    (fake_bin / 'gz').write_text('#!/bin/sh\nexit 0\n')
    (fake_bin / 'gz').chmod(0o755)

    outcome = run_captured(
        ['bash', '-c', render_check([str(tmp_path / 'opt' / 'ros')])],
        {'PATH': f'{fake_bin}:/usr/bin:/bin'},
        20,
    )

    assert f'world: {world}' in outcome['output']


def test_frame_judge_fails_a_frame_where_every_pixel_is_the_same_color():
    gz_text = 'width: 2\nheight: 1\nstep: 6\ndata: "\\347\\347\\347\\347\\347\\347"\n'

    verdict = judge_frame(gz_text)

    assert not verdict.passed
    assert verdict.distinct_colors == 1


def test_frame_judge_passes_a_frame_with_two_colors():
    gz_text = 'width: 2\nheight: 1\nstep: 6\ndata: "\\347\\347\\347\\"\\\\A"\n'

    verdict = judge_frame(gz_text)

    assert verdict.passed
    assert verdict.distinct_colors == 2


def test_frame_judge_decodes_an_escaped_quote_and_its_octal_form_to_one_byte():
    gz_text = 'width: 2\nheight: 1\nstep: 6\ndata: "\\"\\"\\"\\042\\042\\042"\n'

    verdict = judge_frame(gz_text)

    assert verdict.distinct_colors == 1


def test_frame_judge_fails_a_frame_shorter_than_height_times_step():
    gz_text = 'width: 2\nheight: 2\nstep: 6\ndata: "\\001\\002\\003\\004\\005\\006"\n'

    verdict = judge_frame(gz_text)

    assert not verdict.passed
    assert '6 of 12 bytes' in verdict.summary


def test_frame_judge_fails_when_no_image_was_received():
    verdict = judge_frame('')

    assert not verdict.passed


def test_frame_check_fails_a_step_that_exited_cleanly_with_a_blank_frame(tmp_path):
    frame_path = tmp_path / 'frame.txt'
    frame_path.write_text('width: 2\nheight: 1\nstep: 6\ndata: "\\347\\347\\347\\347\\347\\347"\n')
    step = ProbeStep('render', 'true', 10, {}, frame_path=str(frame_path))

    outcome = apply_frame_check(step, {'exit_code': 0, 'seconds': 1.0, 'output': 'world: x\n'})

    assert outcome['exit_code'] == 3
    assert '1 distinct color' in outcome['output']


def test_frame_check_leaves_a_step_without_a_frame_unchanged():
    step = ProbeStep('system', 'true', 10, {})
    outcome = {'exit_code': 0, 'seconds': 1.0, 'output': 'cpus: 8\n'}

    assert apply_frame_check(step, outcome) == outcome


def test_midflight_snapshot_reads_both_udp_drop_counter_sources():
    script = deep_sample_script()

    assert 'cat /proc/net/udp' in script
    assert 'Udp:' in script
    assert '/proc/net/snmp' in script


def test_midflight_snapshot_labels_proc_net_udp_as_unreliable_for_drops():
    assert 'gVisor reports drops and queues as 0' in deep_sample_script()


def test_midflight_snapshot_prints_the_udp_counters_read_at_sampler_start():
    script = deep_sample_script(udp_counters_at_start='Udp: 1 2 3 4')

    assert 'udp counters at sampler start' in script
    assert 'Udp: 1 2 3 4' in script


def test_udp_counters_are_read_from_the_snmp_udp_lines(tmp_path):
    snmp = tmp_path / 'snmp'
    snmp.write_text('Tcp: a b\nTcp: 1 2\nUdp: InDatagrams RcvbufErrors\nUdp: 7 9\n')

    assert udp_counters_text(snmp) == 'Udp: InDatagrams RcvbufErrors\nUdp: 7 9\n'


def test_udp_counters_of_a_missing_file_are_empty(tmp_path):
    assert udp_counters_text(tmp_path / 'absent') == ''


def test_the_relayed_odometry_topic_is_sampled_to_show_the_relay_is_connected():
    assert '/cinematography/drone_odometry' in SAMPLED_TOPICS
