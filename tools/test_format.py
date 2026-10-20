import subprocess
import sys
from pathlib import Path

FORMATTER = Path(__file__).resolve().parent / 'format.py'
SOURCE_MISSING_BLANK_BEFORE_RETURN = (
    'def doubled(value):\n    result = value * 2\n    return result\n'
)


def _run_formatter(*arguments):
    return subprocess.run(
        [sys.executable, str(FORMATTER), *arguments], capture_output=True, text=True, check=False
    )


def test_check_fails_and_names_a_file_missing_the_blank_line_before_return(tmp_path):
    source_path = tmp_path / 'module.py'
    source_path.write_text(SOURCE_MISSING_BLANK_BEFORE_RETURN)

    result = _run_formatter('--check', str(source_path))

    assert result.returncode == 1
    assert str(source_path) in result.stdout


def test_check_writes_nothing(tmp_path):
    source_path = tmp_path / 'module.py'
    source_path.write_text(SOURCE_MISSING_BLANK_BEFORE_RETURN)

    _run_formatter('--check', str(source_path))

    assert source_path.read_text() == SOURCE_MISSING_BLANK_BEFORE_RETURN


def test_check_passes_once_the_file_is_formatted(tmp_path):
    source_path = tmp_path / 'module.py'
    source_path.write_text(SOURCE_MISSING_BLANK_BEFORE_RETURN)
    _run_formatter(str(source_path))

    result = _run_formatter('--check', str(source_path))

    assert result.returncode == 0
