from cine_cli.common import Environment


def test_collect_baseline_runs_its_script(invoke, runner):
    result = invoke('collect', 'baseline')

    assert result.exit_code == 0
    assert runner.commands[0][0] == ['bash', 'tools/collect_baseline_data.sh']


def test_collect_without_ros2_exits_3(runner, repository_root, invoke):
    environment = Environment(
        repository_root=repository_root, runner=runner, which=lambda name: None, variables={}
    )

    result = invoke('collect', 'baseline', environment=environment)

    assert result.exit_code == 3
