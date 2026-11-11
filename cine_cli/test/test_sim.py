def test_sim_start_baseline_dry_run_prints_the_script_call(invoke):
    result = invoke('sim', 'start', 'baseline', '--dry-run')

    assert result.exit_code == 0
    assert result.stdout == 'bash tools/start_simulation.sh baseline\n'


def test_sim_start_passes_each_set_as_a_launch_argument(invoke, runner):
    result = invoke('sim', 'start', 'constrained', '--set', 'rviz=false', '--set', 'seed=3')

    assert result.exit_code == 0
    assert runner.commands[0][0] == [
        'bash',
        'tools/start_simulation.sh',
        'constrained',
        'rviz:=false',
        'seed:=3',
    ]


def test_sim_start_rejects_a_set_without_equals(invoke):
    result = invoke('sim', 'start', 'baseline', '--set', 'rviz')

    assert result.exit_code == 2


def test_sim_start_rejects_an_unknown_mode(invoke):
    result = invoke('sim', 'start', 'fast')

    assert result.exit_code == 2


def test_sim_start_set_without_mode_passes_an_empty_mode_first(invoke, runner):
    result = invoke('sim', 'start', '--set', 'rviz=false')

    assert result.exit_code == 0
    assert runner.commands[0][0] == ['bash', 'tools/start_simulation.sh', '', 'rviz:=false']


def test_sim_start_set_without_mode_dry_run_quotes_the_empty_mode(invoke):
    result = invoke('sim', 'start', '--set', 'rviz=false', '--dry-run')

    assert result.stdout == "bash tools/start_simulation.sh '' rviz:=false\n"


def test_sim_start_rejects_a_set_with_an_empty_value(invoke):
    result = invoke('sim', 'start', 'baseline', '--set', 'rviz=')

    assert result.exit_code == 2
