import json
from dataclasses import replace

from cine_cli.common import ROS2_MISSING_MESSAGE, CapturedOutput

UNBUILT_WORKSPACE = CapturedOutput(returncode=1, stdout='')


def test_doctor_all_healthy_exits_0_with_three_checks(invoke):
    result = invoke('doctor', '--json')

    checks = json.loads(result.stdout)['checks']
    assert result.exit_code == 0
    assert [check['name'] for check in checks] == ['ros2', 'ROS_DOMAIN_ID', 'workspace built']
    assert all(check['passed'] for check in checks)


def test_doctor_unbuilt_workspace_fails_that_check(invoke, runner):
    runner.captures[('ros2', 'pkg', 'prefix', 'cine_bringup')] = UNBUILT_WORKSPACE

    result = invoke('doctor', '--json')

    checks = {check['name']: check for check in json.loads(result.stdout)['checks']}
    assert result.exit_code == 1
    assert checks['workspace built']['passed'] is False
    assert checks['ros2']['passed'] is True


def test_doctor_without_ros2_exits_3_and_reports_one_check(invoke, environment):
    without_ros2 = replace(environment, which=lambda name: None)

    result = invoke('doctor', '--json', environment=without_ros2)

    checks = json.loads(result.stdout)['checks']
    assert result.exit_code == 3
    assert [check['name'] for check in checks] == ['ros2']
    assert checks[0]['passed'] is False
    assert checks[0]['detail'] == ROS2_MISSING_MESSAGE


def test_doctor_prints_one_line_per_check_in_human_form(invoke):
    result = invoke('doctor')

    assert 'PASS  ROS_DOMAIN_ID  unset (0)' in result.stdout.splitlines()


def test_doctor_reports_the_domain_id_when_set(invoke, environment):
    with_domain = replace(environment, variables={'ROS_DOMAIN_ID': '42'})

    result = invoke('doctor', '--json', environment=with_domain)

    checks = {check['name']: check for check in json.loads(result.stdout)['checks']}
    assert checks['ROS_DOMAIN_ID']['detail'] == '42'


def test_doctor_runs_without_a_repository_root(invoke, environment):
    without_root = replace(environment, repository_root=None)

    result = invoke('doctor', '--json', environment=without_root)

    assert result.exit_code == 0
