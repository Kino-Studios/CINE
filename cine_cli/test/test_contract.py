import json
import sys


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
