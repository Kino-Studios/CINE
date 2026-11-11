from dataclasses import replace


def test_analyze_stats_runs_the_script_with_its_options(invoke, runner, repository_root):
    invoke('analyze', 'stats', '--trajectory-root', 'data/t', '--datasets', 'baseline')

    assert runner.commands == [
        (
            [
                'python3',
                'tools/analysis/trajectory_stats.py',
                '--trajectory-root',
                'data/t',
                '--datasets',
                'baseline',
            ],
            repository_root,
        )
    ]


def test_analyze_stats_forwards_repeated_datasets_as_one_flag(invoke, runner):
    invoke('analyze', 'stats', '--datasets', 'baseline', '--datasets', 'cinematic')

    assert runner.commands[0][0] == [
        'python3',
        'tools/analysis/trajectory_stats.py',
        '--datasets',
        'baseline',
        'cinematic',
    ]


def test_analyze_stats_forwards_output_dir(invoke, runner):
    invoke('analyze', 'stats', '--output-dir', 'out')

    assert runner.commands[0][0][-2:] == ['--output-dir', 'out']


def test_analyze_mean_passes_time_step(invoke, runner):
    invoke('analyze', 'mean', '--time-step', '0.01')

    assert runner.commands[0][0] == [
        'python3',
        'tools/analysis/trajectory_mean.py',
        '--time-step',
        '0.01',
    ]


def test_analyze_mean_passes_combined_output(invoke, runner):
    invoke('analyze', 'mean', '--combined-output', 'combined.csv')

    assert runner.commands[0][0][-2:] == ['--combined-output', 'combined.csv']


def test_analyze_without_options_passes_none(invoke, runner):
    invoke('analyze', 'stats')

    assert runner.commands[0][0] == ['python3', 'tools/analysis/trajectory_stats.py']


def test_analyze_dry_run_prints_the_command_and_starts_nothing(invoke, runner):
    result = invoke('analyze', 'mean', '--dry-run')

    assert result.stdout == 'python3 tools/analysis/trajectory_mean.py\n'
    assert runner.commands == []


def test_analyze_works_without_ros2(invoke, environment):
    without_ros2 = replace(environment, which=lambda name: None)

    result = invoke('analyze', 'stats', environment=without_ros2)

    assert result.exit_code == 0
