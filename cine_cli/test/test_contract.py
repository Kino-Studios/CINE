import dataclasses
import json
import sys

from cine_cli.common import CapturedOutput


def test_contract_check_passes_on_this_repository(invoke):
    result = invoke('contract', 'check')

    assert result.exit_code == 0


def test_contract_check_json_is_one_object(invoke):
    result = invoke('contract', 'check', '--json')

    assert isinstance(json.loads(result.stdout), dict)


def test_contract_check_without_checker_exits_3(invoke, monkeypatch):
    monkeypatch.setitem(sys.modules, 'cine_interfaces', None)

    result = invoke('contract', 'check')

    assert result.exit_code == 3
    assert 'pipx inject cine-cli' in result.stderr


RATE_TOOL_PREFIX = ('python3', 'tools/measure_topic_rates.py')
GRAPH_QUERY_PREFIX = ('ros2', 'topic', 'info')


def rate_report(topics):
    return CapturedOutput(
        returncode=0,
        stdout=json.dumps({'window_seconds': 5.0, 'topics': topics}),
    )


def rate_tool_command(runner):
    return next(
        command for command in runner.captured_commands if tuple(command[:2]) == RATE_TOOL_PREFIX
    )


def test_exchange_prints_the_measured_rate_of_each_topic(invoke, runner):
    runner.captures[RATE_TOOL_PREFIX] = rate_report(
        {'/camera/image_raw': {'messages': 20, 'rate_hz': 4.0}},
    )

    result = invoke('contract', 'exchange')

    assert '/camera/image_raw' in result.stdout
    assert '4.0 Hz' in result.stdout


def test_exchange_measures_the_simulator_camera_with_its_message_type(invoke, runner):
    invoke('contract', 'exchange')

    assert '/camera/image_raw=sensor_msgs/msg/Image' in rate_tool_command(runner)


def test_exchange_in_sim_mode_skips_topics_only_the_drone_publishes(invoke, runner):
    invoke('contract', 'exchange')

    assert '/vio/odometry=nav_msgs/msg/Odometry' not in rate_tool_command(runner)


def test_exchange_in_hardware_mode_measures_the_drone_odometry(invoke, runner):
    invoke('contract', 'exchange', '--mode', 'hardware')

    assert '/vio/odometry=nav_msgs/msg/Odometry' in rate_tool_command(runner)


def test_exchange_skips_topics_the_contract_gives_no_publisher(invoke, runner):
    invoke('contract', 'exchange')

    assert not any(
        argument.startswith('/cinematography/translational_velocity_setpoints=')
        for argument in rate_tool_command(runner)
    )


def test_exchange_measures_over_the_requested_window(invoke, runner):
    invoke('contract', 'exchange', '--seconds', '3')

    command = rate_tool_command(runner)
    assert command[command.index('--seconds') + 1] == '3.0'


def test_exchange_fails_when_a_contract_topic_is_missing_from_the_graph(invoke, runner):
    runner.captures[GRAPH_QUERY_PREFIX] = CapturedOutput(returncode=1, stdout='')
    runner.captures[RATE_TOOL_PREFIX] = rate_report({})

    result = invoke('contract', 'exchange')

    assert result.exit_code == 1
    assert 'topic /camera/image_raw is not in the graph' in result.stdout


def test_exchange_fails_when_a_published_topic_carries_no_messages(invoke, runner):
    runner.captures[RATE_TOOL_PREFIX] = rate_report(
        {'/camera/image_raw': {'messages': 0, 'rate_hz': None}},
    )

    result = invoke('contract', 'exchange')

    assert result.exit_code == 1
    assert 'ERROR [simulated_camera_image] /camera/image_raw: no messages in 5 s' in result.stdout


def test_exchange_reports_a_silent_accepted_mismatch_as_info(invoke, runner):
    runner.captures[RATE_TOOL_PREFIX] = rate_report(
        {'/depth/image_raw': {'messages': 0, 'rate_hz': None}},
    )

    result = invoke('contract', 'exchange')

    assert 'INFO [depth_image] /depth/image_raw: no messages in 5 s' in result.stdout


def test_exchange_json_carries_findings_and_rates_in_one_object(invoke, runner):
    runner.captures[RATE_TOOL_PREFIX] = rate_report(
        {'/camera/image_raw': {'messages': 20, 'rate_hz': 4.0}},
    )

    report = json.loads(invoke('contract', 'exchange', '--json').stdout)

    assert report['topics']['/camera/image_raw']['rate_hz'] == 4.0
    assert isinstance(report['findings'], list)


def test_exchange_exits_3_when_the_rate_tool_cannot_run(invoke, runner):
    runner.captures[RATE_TOOL_PREFIX] = CapturedOutput(returncode=1, stdout='')

    result = invoke('contract', 'exchange')

    assert result.exit_code == 3
    assert 'rate measurement failed' in result.stderr


def test_exchange_without_ros2_exits_3(invoke, environment):
    without_ros2 = dataclasses.replace(environment, which=lambda name: None)

    result = invoke('contract', 'exchange', environment=without_ros2)

    assert result.exit_code == 3


def test_exchange_without_checker_exits_3(invoke, monkeypatch):
    monkeypatch.setitem(sys.modules, 'cine_interfaces', None)

    result = invoke('contract', 'exchange')

    assert result.exit_code == 3
