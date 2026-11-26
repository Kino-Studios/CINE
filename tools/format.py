#!/usr/bin/env python3
"""CINE source formatter: the whitespace convention ruff cannot enforce.

A blank line before every `return`/`raise` that is not first in its block, none right after a
`def`/`class` header, and blank runs in bodies collapsed to one; module-level spacing is ruff's.

Run from a repo root:
    python3 tools/format.py                # format all tracked python in this repo
    python3 tools/format.py path/to/file.py [more.py ...]
    python3 tools/format.py --check        # list files that would change; exit 1 if any

Idempotent: it only moves blank lines. If `ruff` is on PATH it runs first.
"""

import argparse
import ast
import os
import shutil
import subprocess
import sys

_EXCLUDE_PARTS = (
    '/venv/',
    '/install/',
    '/build/',
    '/ceres-solver',
    '/eigen-',
    '/__pycache__/',
    '/.ruff_cache/',
    '/log/',
    '/third_party/',
)


def _iter_repo_python(root):
    try:
        tracked = subprocess.run(
            ['git', 'ls-files', '*.py'],
            cwd=root,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.split()
    except (subprocess.CalledProcessError, FileNotFoundError):
        tracked = []

    for rel in tracked:
        if any(part in f'/{rel}' for part in _EXCLUDE_PARTS):
            continue

        yield os.path.join(root, rel)


def _block_opener(line):
    stripped = line.strip()

    return stripped.endswith(':') and stripped.split(' ', 1)[0] in {
        'def',
        'class',
        'if',
        'elif',
        'else',
        'for',
        'while',
        'with',
        'try',
        'except',
        'finally',
    }


def _statement_lines(source):
    """Line numbers (1-based) of return/raise statements that are not the first
    statement of their enclosing block, plus the header line of every def/class."""

    tree = ast.parse(source)
    returns = set()
    headers = set()

    for node in ast.walk(tree):
        body = getattr(node, 'body', None)
        if isinstance(body, list):
            for index, child in enumerate(body):
                if index > 0 and isinstance(child, (ast.Return, ast.Raise)):
                    returns.add(child.lineno)

        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            headers.add(node.body[0].lineno - 1)

    return returns, headers


def _next_indent(lines):
    """For each line index, the leading-space count of the next non-blank line
    (-1 if none follows). Used to tell body-level blank runs from module-level
    gaps, which ruff owns."""

    upcoming = -1
    result = [-1] * len(lines)

    for index in range(len(lines) - 1, -1, -1):
        if lines[index].strip():
            upcoming = len(lines[index]) - len(lines[index].lstrip())

        result[index] = upcoming

    return result


def _format_source(source):
    try:
        returns, _ = _statement_lines(source)
    except SyntaxError:
        return source

    lines = source.split('\n')
    next_indent = _next_indent(lines)
    out = []

    for number, line in enumerate(lines, start=1):
        prev = out[-1] if out else ''
        if number in returns and prev.strip() and not _block_opener(prev):
            out.append('')

        is_blank = line.strip() == ''
        # Collapse blank runs only inside an indented block; a run before column-0 code is a
        # module-level gap, where ruff enforces two blank lines around top-level definitions.
        if is_blank and prev.strip() == '' and out and next_indent[number - 1] > 0:
            continue

        if is_blank and _block_opener(prev):
            continue

        out.append(line)

    return '\n'.join(out)


def _run_ruff(paths):
    if not shutil.which('ruff'):
        return

    # --force-exclude: honour ruff.toml's excludes even for paths named explicitly.
    subprocess.run(['ruff', 'check', '--fix', '--quiet', '--force-exclude', *paths], check=False)
    subprocess.run(['ruff', 'format', '--quiet', '--force-exclude', *paths], check=False)


def main(argv):
    parser = argparse.ArgumentParser(description='Apply the repository whitespace convention.')
    parser.add_argument('paths', nargs='*', help='files to format (default: all tracked python)')
    parser.add_argument(
        '--check',
        action='store_true',
        help='write nothing; list files that would change and exit 1 if there are any',
    )
    arguments = parser.parse_args(argv)
    if arguments.paths:
        paths = [os.path.abspath(path) for path in arguments.paths]
    else:
        paths = list(_iter_repo_python(os.getcwd()))

    if not arguments.check:
        _run_ruff(paths)

    changed = 0

    for path in paths:
        with open(path) as handle:
            original = handle.read()

        formatted = _format_source(original)
        if formatted == original:
            continue

        changed += 1

        if arguments.check:
            print(f'would reformat {path}')
            continue

        with open(path, 'w') as handle:
            handle.write(formatted)

        print(f'formatted {path}')

    if arguments.check:
        print(f'{changed} file(s) would change of {len(paths)} checked')

        return 1 if changed else 0

    print(f'{changed} file(s) changed of {len(paths)} checked')

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
