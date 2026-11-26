import subprocess
import sys
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import pytest
from sim_plan import (
    INSTALLER_ROOT_CHECK_PATCH,
    BuildParallelism,
    OutcomeFacts,
    SimRunPlan,
    StatsSample,
    bounded_log,
    build_parallelism,
    cgroup_memory_headroom_gib,
    classify_outcome,
    cloud_dockerfile,
    colcon_defaults_yaml,
    estimate_cost,
    executable_paths,
    format_estimate,
    git_state_from_output,
    interval_real_time_factors,
    is_safe_run_id,
    make_run_id,
    memory_available_gib,
    parse_crashes,
    parse_real_time_factors,
    parse_stats_samples,
    read_stats_output,
    record_stats_reading,
    restore_modes_command,
    stack_environment,
    summarize_factors,
    summarize_intervals,
    summary_lines,
    time_budget,
    upload_ignore_patterns,
    validate_plan,
    wind_profile_names,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
WIND_NAMES = ['calm', 'gusty']


def _plan(**overrides):
    fields = {
        'mode': 'mission',
        'cpu_cores': 8.0,
        'memory_gib': 16.0,
        'gpu': None,
        'flight_seconds': 600,
        'nav_source': 'sim',
        'wind': 'calm',
        'run_id': '2026-11-20_12-00-00_mission_sim_calm_ab12',
    }
    fields.update(overrides)

    return SimRunPlan(**fields)


def test_default_plan_has_no_problems():
    assert validate_plan(_plan(), WIND_NAMES) == []


def test_unknown_mode_is_reported():
    assert validate_plan(_plan(mode='race'), WIND_NAMES) == [
        "mode 'race' is not one of mission, check, probe",
    ]


def test_unknown_nav_source_is_reported():
    assert validate_plan(_plan(nav_source='gps'), WIND_NAMES) == [
        "nav_source 'gps' is not one of sim, vins",
    ]


def test_wind_missing_from_the_profiles_file_is_reported():
    assert validate_plan(_plan(wind='storm'), WIND_NAMES) == [
        "wind 'storm' is not in wind_profiles.yaml (calm, gusty)",
    ]


def test_unknown_gpu_is_reported():
    assert validate_plan(_plan(gpu='RTX9090'), WIND_NAMES) == [
        "gpu 'RTX9090' is not one of T4, L4, L40S, A100-40GB, A100-80GB, H100",
    ]


def test_too_few_cores_for_the_stack_is_reported():
    assert validate_plan(_plan(cpu_cores=1.0), WIND_NAMES) == [
        'cpu_cores 1.0 is outside 2 to 64',
    ]


def test_memory_below_the_stack_floor_is_reported():
    assert validate_plan(_plan(memory_gib=4.0), WIND_NAMES) == [
        'memory_gib 4.0 is outside 8 to 256',
    ]


def test_flight_shorter_than_takeoff_is_reported():
    assert validate_plan(_plan(flight_seconds=20), WIND_NAMES) == [
        'flight_seconds 20 is outside 60 to 14400',
    ]


def test_flight_length_is_not_checked_for_a_probe():
    assert validate_plan(_plan(mode='probe', flight_seconds=0), WIND_NAMES) == []


def test_unsafe_run_id_is_reported():
    assert validate_plan(_plan(run_id='../escape'), WIND_NAMES) == [
        "run_id '../escape' may hold only letters, digits, '.', '_' and '-'",
    ]


def test_every_problem_is_reported_at_once():
    problems = validate_plan(_plan(nav_source='gps', wind='storm'), WIND_NAMES)

    assert len(problems) == 2


def test_wind_names_are_the_top_level_keys_of_the_profiles_file():
    text = (REPO_ROOT / 'cine_gazebo' / 'config' / 'wind_profiles.yaml').read_text()

    assert wind_profile_names(text) == ['calm', 'gusty']


def test_wind_names_ignore_comments_and_nested_keys():
    text = '# calm: no\nbreezy:\n  linear_velocity: [1, 0, 0]\n'

    assert wind_profile_names(text) == ['breezy']


def test_run_id_carries_time_mode_nav_source_wind_and_token():
    run_id = make_run_id(datetime(2026, 11, 20, 9, 5, 7), 'mission', 'sim', 'gusty', 'ab12')

    assert run_id == '2026-11-20_09-05-07_mission_sim_gusty_ab12'


def test_run_id_with_path_separator_is_unsafe():
    assert not is_safe_run_id('runs/abc')


def test_run_id_made_of_dots_is_unsafe():
    assert not is_safe_run_id('..')


def test_empty_run_id_is_unsafe():
    assert not is_safe_run_id('')


def test_generated_run_id_is_safe():
    assert is_safe_run_id('2026-11-20_09-05-07_mission_sim_gusty_ab12')


def test_build_parallelism_on_eight_cores_and_sixteen_gib():
    assert build_parallelism(8.0, 16.0) == BuildParallelism(package_workers=3, make_jobs=2)


def test_build_parallelism_is_bounded_by_memory_not_cores():
    assert build_parallelism(32.0, 8.0) == BuildParallelism(package_workers=3, make_jobs=1)


def test_build_parallelism_never_drops_below_one_job():
    assert build_parallelism(2.0, 2.0) == BuildParallelism(package_workers=1, make_jobs=1)


def test_build_parallelism_is_bounded_by_cores_on_a_large_memory_machine():
    assert build_parallelism(2.0, 64.0) == BuildParallelism(package_workers=3, make_jobs=1)


def test_colcon_defaults_set_the_package_workers_for_build():
    assert colcon_defaults_yaml(BuildParallelism(3, 2)) == 'build:\n  parallel-workers: 3\n'


def test_cloud_dockerfile_keeps_the_original_and_appends_the_cloud_lines():
    original = 'FROM ubuntu\nENTRYPOINT ["/usr/local/bin/entrypoint.sh"]\n'

    derived = cloud_dockerfile(original)

    assert derived.startswith(original)
    assert 'xvfb' in derived
    assert 'ln -sf /usr/bin/python3 /usr/local/bin/python' in derived
    assert derived.rstrip().endswith('ENTRYPOINT []')


def test_cloud_dockerfile_adds_a_newline_when_the_original_lacks_one():
    derived = cloud_dockerfile('FROM ubuntu')

    assert derived.startswith('FROM ubuntu\n')


def test_cloud_dockerfile_of_the_shipped_dockerfile_installs_xvfb_after_the_snapshot_setup():
    original = (REPO_ROOT / 'docker' / 'gazebo-sim' / 'Dockerfile').read_text()

    derived = cloud_dockerfile(original)

    assert derived.index('snapshot.ubuntu.com') < derived.index('xvfb')


def test_cloud_dockerfile_builds_a_user_section_as_root_with_a_pass_through_sudo():
    original = 'FROM ubuntu\nUSER ardupilot\nRUN git init .\nUSER root\nRUN id\n'

    derived = cloud_dockerfile(original)

    assert derived.index('ln -sf /usr/bin/env /usr/local/sbin/sudo') < derived.index('git init .')
    assert derived.index('git config --system --add safe.directory') < derived.index('git init .')


def test_cloud_dockerfile_removes_the_pass_through_sudo_when_the_section_ends():
    original = 'FROM ubuntu\nUSER ardupilot\nRUN git init .\nUSER root\nRUN id\n'

    derived = cloud_dockerfile(original)

    assert derived.index('git init .') < derived.index('rm -f /usr/local/sbin/sudo')
    assert derived.index('rm -f /usr/local/sbin/sudo') < derived.index('RUN id')


def test_cloud_dockerfile_patches_the_installer_root_check_only_inside_a_user_section():
    original = 'FROM ubuntu\nUSER ardupilot\nRUN git init .\nUSER root\nRUN id\n'

    derived = cloud_dockerfile(original)

    assert f'RUN {INSTALLER_ROOT_CHECK_PATCH}; git init .\n' in derived
    assert '\nRUN id\n' in derived


@pytest.mark.skipif(sys.platform == 'win32', reason='runs the patch through bash')
def test_installer_root_check_patch_switches_off_only_the_root_check(tmp_path):
    installer = tmp_path / 'Tools' / 'environment_install' / 'install-prereqs-ubuntu.sh'
    installer.parent.mkdir(parents=True)
    installer.write_text(
        'set -e\n'
        'if [ $EUID == 0 ]; then\n'
        '    echo "Please do not run this script as root"\n'
        '    exit 1\n'
        'fi\n'
        'OPT="/opt"\n',
    )

    subprocess.run(['bash', '-c', INSTALLER_ROOT_CHECK_PATCH], cwd=tmp_path, check=True)

    assert installer.read_text().splitlines()[1] == 'if false; then'
    assert installer.read_text().splitlines()[5] == 'OPT="/opt"'


def test_executable_paths_keeps_only_files_git_marks_executable():
    listing = (
        '100755 1111111111111111111111111111111111111111 0\tcine_agent/scripts/ros2_nodes/a\0'
        '100644 2222222222222222222222222222222222222222 0\tcine_agent/setup.py\0'
        '120000 3333333333333333333333333333333333333333 0\tlink\0'
    )

    assert executable_paths(listing) == ['cine_agent/scripts/ros2_nodes/a']


def test_executable_paths_keeps_a_path_with_spaces_whole():
    listing = '100755 1111111111111111111111111111111111111111 0\tdocs/run me.sh\0'

    assert executable_paths(listing) == ['docs/run me.sh']


@pytest.mark.skipif(sys.platform == 'win32', reason='runs the command through bash')
def test_restore_modes_command_marks_the_listed_files_executable(tmp_path):
    script = tmp_path / 'node dir' / 'camera_pose_publisher'
    script.parent.mkdir()
    script.write_text('#!/usr/bin/env python3\n')
    script.chmod(0o644)

    command = restore_modes_command(str(tmp_path), ['node dir/camera_pose_publisher'])
    subprocess.run(['bash', '-c', command], check=True)

    assert script.stat().st_mode & 0o111 == 0o111


@pytest.mark.skipif(sys.platform == 'win32', reason='runs the command through bash')
def test_restore_modes_command_skips_a_listed_file_the_upload_left_out(tmp_path):
    command = restore_modes_command(str(tmp_path), ['ignored_by_upload.sh'])

    assert subprocess.run(['bash', '-c', command]).returncode == 0


def test_cloud_dockerfile_of_the_shipped_dockerfile_leaves_no_user_line_for_modal_to_ignore():
    original = (REPO_ROOT / 'docker' / 'gazebo-sim' / 'Dockerfile').read_text()

    derived = cloud_dockerfile(original)

    assert '\nUSER ' not in derived


def test_upload_ignores_cover_nested_virtual_environments_and_git():
    patterns = upload_ignore_patterns()

    assert '**/.venv/**' in patterns
    assert '**/.git/**' in patterns
    assert '**/__pycache__/**' in patterns


def test_upload_ignores_skip_the_restricted_airframe_meshes():
    assert '**/meshes/restricted_*' in upload_ignore_patterns()


def test_upload_ignores_match_the_files_they_name():
    modal = pytest.importorskip('modal')
    matcher = modal.FilePatternMatcher(*upload_ignore_patterns())

    assert matcher(Path('scripts/cloud/.venv/Lib/site.py'))
    assert matcher(Path('.git/HEAD'))
    assert matcher(Path('cine_gazebo/models/cine_source_one/meshes/restricted_frame.obj'))
    assert not matcher(Path('cine_gazebo/launch/gazebo_sim.launch.py'))


def test_stack_environment_runs_headless_with_the_plan_choices():
    environment = stack_environment(
        _plan(wind='gusty', nav_source='vins'),
        BuildParallelism(3, 2),
        '/c.yaml',
    )

    assert environment['HEADLESS'] == 'true'
    assert environment['NAV_SOURCE'] == 'vins'
    assert environment['WIND'] == 'gusty'
    assert environment['MISSION'] == 'true'
    assert environment['CHECK'] == 'false'


def test_stack_environment_passes_the_clean_git_state_to_the_launch():
    environment = stack_environment(
        _plan(git_commit='abc123', git_dirty=False),
        BuildParallelism(3, 2),
        '/c.yaml',
    )

    assert (environment['CINE_GIT_COMMIT'], environment['CINE_GIT_DIRTY']) == ('abc123', '0')


def test_stack_environment_passes_a_dirty_tree_as_one():
    environment = stack_environment(
        _plan(git_commit='abc123', git_dirty=True),
        BuildParallelism(3, 2),
        '/c.yaml',
    )

    assert environment['CINE_GIT_DIRTY'] == '1'


def test_stack_environment_omits_git_state_the_host_did_not_supply():
    environment = stack_environment(_plan(), BuildParallelism(3, 2), '/c.yaml')

    assert 'CINE_GIT_COMMIT' not in environment
    assert 'CINE_GIT_DIRTY' not in environment


def test_a_plan_survives_the_round_trip_through_json_fields_with_its_git_state():
    plan = _plan(git_commit='abc123', git_dirty=True)

    assert SimRunPlan(**asdict(plan)) == plan


def test_git_state_reads_a_clean_tree():
    assert git_state_from_output('abc123\n', '') == ('abc123', False)


def test_git_state_reads_a_tree_with_changes_as_dirty():
    assert git_state_from_output('abc123\n', ' M cine_gazebo/setup.py\n') == ('abc123', True)


def test_git_state_with_a_failed_status_call_is_a_known_commit_and_unknown_dirty():
    assert git_state_from_output('abc123\n', None) == ('abc123', None)


def test_git_state_with_a_failed_head_call_is_unknown():
    assert git_state_from_output(None, '') == (None, None)


def test_git_state_without_a_commit_is_unknown():
    assert git_state_from_output('', '') == (None, None)


def test_stack_environment_caps_the_build():
    environment = stack_environment(_plan(), BuildParallelism(3, 2), '/c.yaml')

    assert environment['MAKEFLAGS'] == '-j2'
    assert environment['COLCON_DEFAULTS_FILE'] == '/c.yaml'


def test_stack_environment_for_a_check_runs_the_smoke_flight_without_the_mission():
    environment = stack_environment(_plan(mode='check'), BuildParallelism(3, 2), '/c.yaml')

    assert environment['CHECK'] == 'true'
    assert environment['MISSION'] == 'false'


def test_stack_environment_keeps_dds_discovery_on_loopback():
    environment = stack_environment(_plan(), BuildParallelism(3, 2), '/c.yaml')

    assert '<Peer address="127.0.0.1"/>' in environment['CYCLONEDDS_URI']
    assert environment['GZ_IP'] == '127.0.0.1'


def test_time_budget_for_a_ten_minute_mission():
    budget = time_budget(_plan(flight_seconds=600))

    assert budget.typical_seconds == 2700
    assert budget.function_timeout_seconds == 7020
    assert budget.internal_deadline_seconds == 6720


def test_time_budget_for_a_probe_has_no_flight_or_build():
    budget = time_budget(_plan(mode='probe'))

    assert budget.typical_seconds == 600
    assert budget.function_timeout_seconds == 2100


def test_time_budget_for_a_check_ignores_the_flight_length():
    assert time_budget(_plan(mode='check', flight_seconds=9000)) == time_budget(_plan(mode='check'))


def test_time_budget_for_a_check_has_no_scoring_stage():
    # build 1200 + weights 300 + launch 180 + check flight 600 + shutdown 120
    assert time_budget(_plan(mode='check')).typical_seconds == 2400


def test_time_budget_never_exceeds_the_modal_maximum():
    assert time_budget(_plan(flight_seconds=14400)).function_timeout_seconds <= 86400


def test_cost_of_eight_cores_and_sixteen_gib_per_hour():
    estimate = estimate_cost(_plan())

    assert estimate.usd_per_hour == pytest.approx(0.505152, abs=1e-5)


def test_cost_ceiling_is_the_function_timeout_at_the_hourly_rate():
    estimate = estimate_cost(_plan())

    assert estimate.ceiling_seconds == 7020
    assert estimate.ceiling_usd == pytest.approx(0.505152 * 7020 / 3600, abs=1e-4)


def test_cost_includes_the_gpu_rate():
    estimate = estimate_cost(_plan(gpu='T4'))

    assert estimate.usd_per_hour == pytest.approx(0.505152 + 0.5904, abs=1e-4)


def test_estimate_text_names_the_spend_flag_and_the_ceiling():
    text = format_estimate(_plan(), estimate_cost(_plan()), launching=False)

    assert '--confirm-spend' in text
    assert 'ceiling' in text
    assert 'assumed' in text


def test_estimate_ceiling_holds_per_attempt_since_a_preempted_call_restarts():
    text = format_estimate(_plan(), estimate_cost(_plan()), launching=False)

    assert 'per attempt' in text


def test_estimate_before_a_confirmed_launch_does_not_claim_nothing_was_launched():
    text = format_estimate(_plan(), estimate_cost(_plan()), launching=True)

    assert 'Nothing was launched' not in text


def test_crashes_are_read_from_launch_exit_lines():
    log = (
        '[INFO] [vins_node-5]: process started with pid [41]\n'
        "[ERROR] [vins_node-5]: process has died [pid 41, exit code -11, cmd 'vins_node'].\n"
        "[ERROR] [depth_publisher_node-9]: process has died [pid 77, exit code 1, cmd 'x'].\n"
    )

    crashes = parse_crashes(log)

    assert [(crash.name, crash.exit_code) for crash in crashes] == [
        ('vins_node', -11),
        ('depth_publisher_node', 1),
    ]


def test_processes_stopped_by_our_sigint_are_not_crashes():
    log = (
        "[ERROR] [bag_recorder-3]: process has died [pid 9, exit code -2, cmd 'ros2 bag record'].\n"
    )

    assert parse_crashes(log) == []


def test_clean_exits_are_not_crashes():
    assert parse_crashes('[INFO] [mavros-2]: process has finished cleanly [pid 12]\n') == []


def test_real_time_factors_are_read_from_gz_stats_output():
    text = 'real_time_factor: 0.93\nsim_time {\n}\nreal_time_factor: 0.41\n'

    assert parse_real_time_factors(text) == [0.93, 0.41]


def test_real_time_factor_in_scientific_notation_is_read():
    assert parse_real_time_factors('real_time_factor: 9.5e-01\n') == [0.95]


def test_memory_available_is_read_from_meminfo_in_gib():
    meminfo = 'MemTotal:       16777216 kB\nMemAvailable:    2097152 kB\n'

    assert memory_available_gib(meminfo) == 2.0


def test_memory_available_is_none_without_the_field():
    assert memory_available_gib('MemTotal: 1 kB\n') is None


def test_cgroup_headroom_is_limit_minus_usage_in_gib():
    assert cgroup_memory_headroom_gib('1073741824\n', '4294967296\n') == 3.0


def test_cgroup_headroom_counts_reclaimable_file_cache_as_free():
    memory_stat = 'anon 2147483648\ninactive_file 1073741824\nactive_file 5\n'

    assert cgroup_memory_headroom_gib('3221225472\n', '4294967296\n', memory_stat) == 2.0


def test_cgroup_without_a_limit_has_no_headroom_figure():
    assert cgroup_memory_headroom_gib('1073741824\n', 'max\n') is None


def test_cgroup_with_unreadable_values_has_no_headroom_figure():
    assert cgroup_memory_headroom_gib('', '') is None


def test_bounded_log_keeps_both_ends_of_a_long_log():
    text = 'head' + 'x' * 1000 + 'tail'

    bounded = bounded_log(text, 100)

    assert bounded.startswith('head')
    assert bounded.endswith('tail')
    assert 'characters elided' in bounded
    assert len(bounded) < 200


def test_bounded_log_leaves_a_short_log_alone():
    assert bounded_log('short', 100) == 'short'


def _facts(**overrides):
    fields = {
        'mode': 'mission',
        'launch_started': True,
        'stack_exit_code': None,
        'stopped_by': 'flight_time',
        'bag_complete': True,
        'crash_count': 0,
    }
    fields.update(overrides)

    return OutcomeFacts(**fields)


def test_a_full_flight_with_a_complete_bag_is_ok():
    assert classify_outcome(_facts()) == 'ok'


def test_a_stack_that_dies_before_the_launch_failed_before_launch():
    assert classify_outcome(_facts(launch_started=False, stack_exit_code=2, stopped_by=None)) == (
        'failed_before_launch'
    )


def test_a_memory_stop_is_reported_even_with_a_bag():
    assert classify_outcome(_facts(stopped_by='memory')) == 'memory_stop'


def test_a_deadline_stop_is_reported_even_with_a_bag():
    assert classify_outcome(_facts(stopped_by='deadline')) == 'deadline_stop'


def test_a_stack_that_exits_by_itself_during_the_flight_exited_early():
    assert classify_outcome(_facts(stack_exit_code=0, stopped_by=None)) == 'stack_exited'


def test_a_flight_without_a_finished_bag_has_no_bag():
    assert classify_outcome(_facts(bag_complete=False)) == 'no_bag'


def test_a_flight_where_a_node_died_is_flagged():
    assert classify_outcome(_facts(crash_count=2)) == 'process_died'


def test_a_smoke_check_that_exits_zero_passed():
    assert (
        classify_outcome(_facts(mode='check', stack_exit_code=0, stopped_by=None)) == 'check_passed'
    )


def test_a_smoke_check_that_exits_non_zero_failed():
    assert (
        classify_outcome(_facts(mode='check', stack_exit_code=1, stopped_by=None)) == 'check_failed'
    )


def test_a_smoke_check_cut_off_by_the_deadline_is_a_deadline_stop():
    assert classify_outcome(_facts(mode='check', stopped_by='deadline')) == 'deadline_stop'


def test_a_memory_stop_during_the_build_is_a_memory_stop():
    assert classify_outcome(_facts(launch_started=False, stopped_by='memory')) == 'memory_stop'


def test_a_deadline_stop_during_the_build_is_a_deadline_stop():
    assert classify_outcome(_facts(launch_started=False, stopped_by='deadline')) == 'deadline_stop'


def test_real_time_factor_summary_has_count_mean_and_range():
    assert summarize_factors([0.5, 1.0, 0.75]) == {
        'samples': 3,
        'mean': 0.75,
        'min': 0.5,
        'max': 1.0,
    }


def test_real_time_factor_summary_of_no_samples_is_none():
    assert summarize_factors([]) is None


def _result(**overrides):
    fields = {
        'run_id': 'r1',
        'mode': 'mission',
        'status': 'ok',
        'timings': {'total_seconds': 2400.0, 'launch_seconds': 1500.0, 'flight_seconds': 600.0},
        'crashes': [],
        'real_time_factor': {'samples': 20, 'mean': 0.62, 'min': 0.4, 'max': 0.9},
        'step_real_time_factor': None,
        'memory': {'lowest_headroom_gib': 3.2},
        'bag': {'name': 'b1', 'complete': True, 'reindexed': False},
        'gate': 'PASS',
        'cost': {'elapsed_usd': 0.34},
        'warnings': [],
    }
    fields.update(overrides)

    return fields


def test_summary_names_the_status_gate_and_cost():
    text = '\n'.join(summary_lines(_result()))

    assert 'ok' in text
    assert 'PASS' in text
    assert '$0.34' in text


def test_summary_lists_each_crashed_process():
    text = '\n'.join(summary_lines(_result(crashes=[{'name': 'vins_node', 'exit_code': -11}])))

    assert 'vins_node (exit -11)' in text


def test_summary_of_a_failed_build_has_no_flight_figures():
    result = _result(status='failed_before_launch', real_time_factor=None, bag=None, gate=None)

    text = '\n'.join(summary_lines(result))

    assert 'failed_before_launch' in text
    assert 'real-time factor: not measured' in text


def test_summary_of_a_probe_lists_each_step_result():
    probe = [
        {'name': 'dds_loopback', 'exit_code': 0, 'seconds': 3.0},
        {'name': 'dds_default', 'exit_code': 124, 'seconds': 25.0},
    ]

    text = '\n'.join(summary_lines(_result(mode='probe', status='probe_done', probe=probe)))

    assert 'dds_loopback: pass' in text
    assert 'dds_default: FAIL (exit 124)' in text


STATS_BLOCK = """sim_time {
  sec: 12
  nsec: 500000000
}
real_time {
  sec: 100
  nsec: 250000000
}
real_time_factor: 0.8
paused: false
iterations: 12500
"""


def test_stats_text_gives_sim_time_real_time_and_step_factor():
    assert parse_stats_samples(STATS_BLOCK) == [StatsSample(12.5, 100.25, 0.8)]


def test_stats_time_with_a_zero_sec_field_omitted_is_still_read():
    text = 'sim_time {\n  nsec: 250000000\n}\nreal_time {\n  sec: 3\n}\nreal_time_factor: 0.5\n'

    assert parse_stats_samples(text) == [StatsSample(0.25, 3.0, 0.5)]


def test_two_messages_in_one_output_give_two_samples():
    second = STATS_BLOCK.replace('sec: 12', 'sec: 13').replace('sec: 100', 'sec: 101')

    assert len(parse_stats_samples(STATS_BLOCK + second)) == 2


def test_output_without_stats_gives_no_samples():
    assert parse_stats_samples('') == []


def test_interval_factor_is_sim_time_gained_over_real_time_gained():
    samples = [StatsSample(10.0, 100.0, 0.9), StatsSample(11.0, 102.0, 0.9)]

    assert interval_real_time_factors(samples) == [0.5]


def test_three_samples_give_two_intervals():
    samples = [StatsSample(0.0, 0.0, 1.0), StatsSample(1.0, 2.0, 1.0), StatsSample(4.0, 6.0, 1.0)]

    assert interval_real_time_factors(samples) == [0.5, 0.75]


def test_a_single_sample_gives_no_interval():
    assert interval_real_time_factors([StatsSample(1.0, 1.0, 1.0)]) == []


def test_an_interval_with_no_real_time_gained_is_skipped():
    samples = [StatsSample(1.0, 5.0, 1.0), StatsSample(2.0, 5.0, 1.0)]

    assert interval_real_time_factors(samples) == []


def test_a_sample_lost_to_a_timeout_stretches_the_next_interval():
    kept = [StatsSample(0.0, 0.0, 1.0), StatsSample(3.0, 20.0, 1.0)]

    assert interval_real_time_factors(kept) == [0.15]


def test_summary_mean_is_total_sim_time_over_total_real_time():
    samples = [StatsSample(0.0, 0.0, 1.0), StatsSample(1.0, 2.0, 1.0), StatsSample(4.0, 6.0, 1.0)]

    assert summarize_intervals(samples) == {'samples': 2, 'mean': 0.667, 'min': 0.5, 'max': 0.75}


def test_summary_of_fewer_than_two_samples_is_none():
    assert summarize_intervals([StatsSample(1.0, 1.0, 1.0)]) is None


def _frozen_flight_samples():
    fast_steps = [StatsSample(index * 0.001, index * 0.00125, 0.8) for index in range(10)]
    frozen_step = StatsSample(0.010, 9 * 0.00125 + 0.08, 0.0123)

    return fast_steps + [frozen_step]


def test_honest_mean_is_far_below_the_per_step_mean_when_one_step_freezes():
    samples = _frozen_flight_samples()

    honest = summarize_intervals(samples)['mean']
    per_step = summarize_factors([sample.step_real_time_factor for sample in samples])['mean']

    assert honest < 0.15
    assert per_step > 0.7


def test_summary_lines_label_the_honest_figure_as_sim_time_over_wall_time():
    text = '\n'.join(summary_lines(_result()))

    assert 'sim time over wall time' in text


def test_summary_lines_label_the_step_figure_as_per_step():
    result = _result(step_real_time_factor={'samples': 50, 'mean': 0.372, 'min': 0.01, 'max': 0.9})

    text = '\n'.join(summary_lines(result))

    assert 'per-step real-time factor' in text
    assert '0.37' in text


def test_summary_lines_read_an_old_result_without_the_step_key():
    result = _result()
    del result['step_real_time_factor']

    text = '\n'.join(summary_lines(result))

    assert 'older run' in text


def _stats_block(sim_seconds, real_seconds, step_factor):
    return (
        f'sim_time {{\n  sec: {sim_seconds}\n}}\nreal_time {{\n  sec: {real_seconds}\n}}\n'
        f'real_time_factor: {step_factor}\n'
    )


def test_an_output_with_several_messages_contributes_only_its_first():
    output = _stats_block(10, 100, 0.8) + _stats_block(11, 101, 0.7)

    reading = read_stats_output(output)

    assert reading.sample == StatsSample(10.0, 100.0, 0.8)


def test_the_row_fields_record_the_chosen_sample_times_and_step_factor():
    reading = read_stats_output(_stats_block(10, 100, 0.8))

    assert reading.row_fields == {
        'real_time_factor': 0.8,
        'sim_seconds': 10.0,
        'real_seconds': 100.0,
    }


def test_an_empty_output_gives_no_sample_and_null_row_fields():
    reading = read_stats_output('')

    assert reading.sample is None
    assert reading.row_fields == {
        'real_time_factor': None,
        'sim_seconds': None,
        'real_seconds': None,
    }


def test_a_timed_out_output_between_two_stats_blocks_is_spanned_by_one_interval():
    outputs = [
        _stats_block(10, 100, 0.8),
        '',
        '\ntimed out after 20 s',
        _stats_block(13, 130, 0.8),
    ]
    samples = []
    record_stats_reading(samples, read_stats_output(outputs[0]))
    record_stats_reading(samples, read_stats_output(outputs[1]))
    record_stats_reading(samples, read_stats_output(outputs[2]))
    record_stats_reading(samples, read_stats_output(outputs[3]))

    assert interval_real_time_factors(samples) == [0.1]


def test_recording_a_reading_without_a_sample_leaves_the_samples_unchanged():
    samples = []

    record_stats_reading(samples, read_stats_output(''))

    assert samples == []


def test_row_step_factor_comes_from_the_same_message_as_the_row_times():
    first_without_factor = 'sim_time {\n  sec: 5\n}\nreal_time {\n  sec: 40\n}\n'

    reading = read_stats_output(first_without_factor + _stats_block(6, 41, 0.7))

    assert reading.row_fields['real_time_factor'] is None


def test_a_cut_off_stats_block_with_the_timeout_note_gives_no_sample():
    reading = read_stats_output('sim_time {\n  sec: 5\n\ntimed out after 20 s')

    assert reading.sample is None


def test_summary_is_none_when_the_end_points_have_no_real_time_between_them():
    samples = [StatsSample(0.0, 0.0, 1.0), StatsSample(5.0, 10.0, 1.0), StatsSample(0.0, 0.0, 1.0)]

    assert summarize_intervals(samples) is None
