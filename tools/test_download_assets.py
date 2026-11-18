from pathlib import Path

import download_assets
import pytest

ASSET_BYTES = b'asset bytes'
ASSET_BYTES_SHA256 = '84293ed06cb3210e7d549afec3140d0c48494416ad25b7f25196afffaa5eb796'
TEST_ASSET = download_assets.Asset(
    name='baseline_run.csv',
    sha256=ASSET_BYTES_SHA256,
    destination='data/trajectories/baseline',
)


def _download_writing(content):
    def download(url, target_path):
        Path(target_path).write_bytes(content)

    return download


def _download_that_must_not_run(url, target_path):
    raise AssertionError(f'download called for {url}')


def test_missing_asset_is_downloaded_to_its_destination(tmp_path):
    download_assets.fetch_asset(TEST_ASSET, tmp_path, 'v1.0.0', _download_writing(ASSET_BYTES))

    assert (tmp_path / 'data/trajectories/baseline/baseline_run.csv').read_bytes() == ASSET_BYTES


def test_asset_is_fetched_from_the_release_named_by_the_tag(tmp_path):
    requested_urls = []

    def recording_download(url, target_path):
        requested_urls.append(url)
        Path(target_path).write_bytes(ASSET_BYTES)

    download_assets.fetch_asset(TEST_ASSET, tmp_path, 'v1.0.0', recording_download)

    assert requested_urls == [
        'https://github.com/Kino-Studios/CINE/releases/download/v1.0.0/baseline_run.csv'
    ]


def test_present_asset_with_matching_checksum_is_not_downloaded_again(tmp_path):
    target = tmp_path / 'data/trajectories/baseline/baseline_run.csv'
    target.parent.mkdir(parents=True)
    target.write_bytes(ASSET_BYTES)

    download_assets.fetch_asset(TEST_ASSET, tmp_path, 'v1.0.0', _download_that_must_not_run)

    assert target.read_bytes() == ASSET_BYTES


def test_present_asset_with_wrong_checksum_is_refused_and_left_alone(tmp_path):
    target = tmp_path / 'data/trajectories/baseline/baseline_run.csv'
    target.parent.mkdir(parents=True)
    target.write_bytes(b'other bytes')

    with pytest.raises(download_assets.ChecksumMismatchError, match='baseline_run.csv'):
        download_assets.fetch_asset(TEST_ASSET, tmp_path, 'v1.0.0', _download_that_must_not_run)

    assert target.read_bytes() == b'other bytes'


def test_downloaded_asset_with_wrong_checksum_is_deleted(tmp_path):
    with pytest.raises(download_assets.ChecksumMismatchError, match='baseline_run.csv'):
        download_assets.fetch_asset(
            TEST_ASSET, tmp_path, 'v1.0.0', _download_writing(b'other bytes')
        )

    assert list((tmp_path / 'data/trajectories/baseline').iterdir()) == []


def test_release_tag_is_the_version_file_prefixed_with_v(tmp_path):
    version_file = tmp_path / 'VERSION'
    version_file.write_text('1.0.0\n')

    assert download_assets.release_tag(version_file) == 'v1.0.0'


def test_only_option_rejects_an_unknown_asset_name():
    with pytest.raises(SystemExit):
        download_assets.main(['--only', 'not-an-asset.bin'])


def _clone_writing(file_names):
    def clone(url, directory):
        meshes_directory = Path(directory) / 'meshes'
        meshes_directory.mkdir(parents=True)
        for file_name in file_names:
            (meshes_directory / file_name).write_text(file_name)

        return True

    return clone


def _clone_that_fails(url, directory):
    return False


def test_private_meshes_are_copied_into_the_source_one_model(tmp_path):
    download_assets.fetch_private_meshes(
        tmp_path, _clone_writing(['restricted_motors.obj', 'restricted_motors.mtl'])
    )

    meshes_directory = tmp_path / 'cine_gazebo/models/cine_source_one/meshes'
    assert sorted(path.name for path in meshes_directory.iterdir()) == [
        'restricted_motors.mtl',
        'restricted_motors.obj',
    ]


def test_private_meshes_come_from_the_cine_cad_repository(tmp_path):
    requested_urls = []

    def recording_clone(url, directory):
        requested_urls.append(url)

        return False

    download_assets.fetch_private_meshes(tmp_path, recording_clone)

    assert requested_urls == ['https://github.com/Kino-Studios/CINE-CAD.git']


def test_failed_private_mesh_clone_warns_and_writes_nothing(tmp_path, capsys):
    download_assets.fetch_private_meshes(tmp_path, _clone_that_fails)

    assert 'warning' in capsys.readouterr().err
    assert not (tmp_path / 'cine_gazebo').exists()
