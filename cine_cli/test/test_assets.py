def test_assets_download_runs_the_script_with_python3(invoke, runner, repository_root):
    invoke('assets', 'download')

    assert runner.commands == [(['python3', 'tools/download_assets.py'], repository_root)]


def test_assets_download_passes_each_only_name(invoke, runner, repository_root):
    invoke('assets', 'download', '--only', 'a', '--only', 'b')

    assert runner.commands == [
        (
            ['python3', 'tools/download_assets.py', '--only', 'a', '--only', 'b'],
            repository_root,
        )
    ]
