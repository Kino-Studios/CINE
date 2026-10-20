import json

import pytest

import cine_config


def test_load_reads_json_from_override_directory(tmp_path, monkeypatch):
    (tmp_path / 'camera').mkdir()
    (tmp_path / 'camera' / 'camera-params.json').write_text(json.dumps({'width': 640}))
    monkeypatch.setenv('CINE_CONFIG_ROOT', str(tmp_path))
    assert cine_config.load('camera/camera-params') == {'width': 640}


def test_load_missing_file_names_the_file(tmp_path, monkeypatch):
    monkeypatch.setenv('CINE_CONFIG_ROOT', str(tmp_path))
    with pytest.raises(FileNotFoundError, match='camera-params.json'):
        cine_config.load('camera/camera-params')


def test_load_malformed_json_names_the_file(tmp_path, monkeypatch):
    (tmp_path / 'camera').mkdir()
    (tmp_path / 'camera' / 'camera-params.json').write_text('{not json')
    monkeypatch.setenv('CINE_CONFIG_ROOT', str(tmp_path))
    with pytest.raises(ValueError, match='camera-params.json'):
        cine_config.load('camera/camera-params')


def test_load_finds_installed_share_when_override_unset(tmp_path, monkeypatch):
    pytest.importorskip('ament_index_python')
    monkeypatch.delenv('CINE_CONFIG_ROOT', raising=False)
    monkeypatch.chdir(tmp_path)
    assert cine_config.load('topics/topic-params')['topics']['qos_depth'] == 10
