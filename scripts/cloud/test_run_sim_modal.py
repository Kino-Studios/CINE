"""Tests for the Modal client half of the runner; skipped where Modal is not installed."""

import io
import sys
from dataclasses import dataclass

import pytest

modal = pytest.importorskip('modal')
run_sim_modal = pytest.importorskip('run_sim_modal')


class _RefusesToSpend:
    def __getattr__(self, name):
        raise AssertionError(f'{name} was reached without --confirm-spend')


def test_a_run_without_the_spend_flag_launches_nothing(monkeypatch, capsys):
    monkeypatch.setattr(run_sim_modal, 'app', _RefusesToSpend())
    monkeypatch.setattr(run_sim_modal, 'fly_simulator', _RefusesToSpend())

    exit_code = run_sim_modal.main(['--mode', 'probe'])

    assert exit_code == 0
    assert 'Nothing was launched' in capsys.readouterr().out


def test_output_redirected_on_windows_can_print_the_check_mark_modal_shows(monkeypatch):
    # Windows Python writes a redirected stdout in cp1252, which has no U+2713.
    redirected = io.BytesIO()
    windows_stdout = io.TextIOWrapper(redirected, encoding='cp1252', newline='\n')
    monkeypatch.setattr(sys, 'stdout', windows_stdout)
    monkeypatch.setattr(sys, 'stderr', io.TextIOWrapper(io.BytesIO(), encoding='cp1252'))

    run_sim_modal.use_utf8_output()
    print('✓ Initialized.')
    sys.stdout.flush()

    assert redirected.getvalue() == '✓ Initialized.\n'.encode()


def test_an_unsafe_run_id_is_refused_before_the_volume_is_read(monkeypatch):
    monkeypatch.setattr(run_sim_modal, 'RUNS_VOLUME', _RefusesToSpend())

    assert run_sim_modal.main(['--fetch', '../other']) == 2


@dataclass
class _Entry:
    path: str
    type: int


class _FakeVolume:
    def __init__(self, files):
        self.files = files

    def listdir(self, path, recursive=False):
        return [_Entry(name, modal.volume.FileEntryType.FILE) for name in self.files]

    def read_file(self, path):
        yield self.files[path]


def test_a_file_that_cannot_be_written_is_skipped_and_the_rest_still_download(
    tmp_path,
    monkeypatch,
):
    monkeypatch.setattr(run_sim_modal, 'DOWNLOAD_ROOT', tmp_path)
    monkeypatch.setattr(
        run_sim_modal,
        'RUNS_VOLUME',
        _FakeVolume({'run_a/launch.log': b'log', 'run_a/result.json': b'{}'}),
    )
    # A directory where the log should go makes writing it fail.
    (tmp_path / 'run_a' / 'launch.log').mkdir(parents=True)

    destination = run_sim_modal.download_run('run_a')

    assert (destination / 'result.json').read_bytes() == b'{}'


class _BreaksMidway(_FakeVolume):
    def read_file(self, path):
        if path == 'run_a/launch.log':
            yield b'partial'

            raise RuntimeError('connection reset')

        yield self.files[path]


def test_a_download_that_breaks_midway_leaves_no_partial_file(tmp_path, monkeypatch):
    monkeypatch.setattr(run_sim_modal, 'DOWNLOAD_ROOT', tmp_path)
    monkeypatch.setattr(
        run_sim_modal,
        'RUNS_VOLUME',
        _BreaksMidway({'run_a/launch.log': b'log', 'run_a/result.json': b'{}'}),
    )

    destination = run_sim_modal.download_run('run_a')

    assert not (destination / 'launch.log').exists()
    assert (destination / 'result.json').read_bytes() == b'{}'


def test_the_repository_is_found_from_the_local_script():
    assert (run_sim_modal.REPO_ROOT / 'scripts' / 'cloud' / 'run_sim_modal.py').is_file()
