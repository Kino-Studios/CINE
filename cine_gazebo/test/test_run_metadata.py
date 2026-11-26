import hashlib
import json

import numpy as np
import pytest
from cine_eval.analysis import CONDITION_FIELDS, condition_key
from cine_eval.channels.mission import score_mission
from cine_eval.record import DRONE_TRUE, FlightRecord, Stream
from cine_eval.settings import load_scoring_settings
from cine_eval.standings import RunFacts, _comparability
from cine_gazebo.run_metadata import (
    build_run_metadata,
    directory_sha1,
    file_sha1,
    git_state_from_environment,
    parse_seed,
    write_run_metadata_when_directory_exists,
)

WAYPOINTS = [(0.0, 3.0, 2.5), (-5.1, 6.0, 3.0)]
GIT = {'CINE': {'commit': 'abc123', 'dirty': False}}


def _metadata(**overrides):
    arguments = {
        'scenario': 'cine_field_mission',
        'world': 'cine_field',
        'wind': 'calm',
        'seed': None,
        'nav_source': 'sim',
        'waypoints': WAYPOINTS,
        'waypoints_sha1': 'a' * 40,
        'focus_target_m': 1.0,
        'git': GIT,
        'launch_arguments': {'airframe': 'iris', 'headless': 'true'},
        'config_sha1': 'b' * 40,
        'perception_source': 'detector',
        'image_id': None,
    }
    arguments.update(overrides)

    return build_run_metadata(**arguments)


def _record_with_run(run, final_position):
    time = np.arange(0.0, 10.0 + 1e-9, 0.1)
    position = np.stack(
        [
            np.linspace(0.0, final_position[0], len(time)),
            np.linspace(0.0, final_position[1], len(time)),
            np.linspace(0.0, final_position[2], len(time)),
        ],
        axis=1,
    )
    stream = Stream(
        name=DRONE_TRUE,
        frame_id='cine_field',
        time=time,
        receive_time=time.copy(),
        fields={'position': position},
    )

    return FlightRecord(streams={DRONE_TRUE: stream}, metadata={'run': run})


def _facts(run):
    return RunFacts(name='run_a', metadata={'run': run}, summaries={}, provenance={}, score=None)


@pytest.mark.parametrize('field', CONDITION_FIELDS)
def test_run_metadata_has_every_field_the_pairing_key_reads(field):
    assert field in _metadata()


def test_run_metadata_lists_mission_waypoints_as_json_lists():
    assert _metadata()['waypoints'] == [[0.0, 3.0, 2.5], [-5.1, 6.0, 3.0]]


def test_run_metadata_names_the_world_as_the_waypoints_frame():
    assert _metadata(world='cine_field')['waypoints_frame'] == 'cine_field'


def test_run_metadata_points_the_world_file_at_the_committed_template():
    assert _metadata()['world_file'] == 'cine_gazebo/worlds/cine_field.sdf.template'


def test_run_metadata_records_the_focus_target_as_a_number():
    assert _metadata(focus_target_m=1.0)['focus_target_m'] == 1.0


def test_run_metadata_no_longer_records_a_filming_time_window():
    assert 'filming_window' not in _metadata()


def test_run_metadata_records_wind_name_and_seed():
    metadata = _metadata(wind='gusty', seed=7)

    assert (metadata['wind'], metadata['seed']) == ('gusty', 7)


def test_run_metadata_records_every_launch_argument_given():
    metadata = _metadata(launch_arguments={'airframe': 'source_one', 'world': 'cine_field'})

    assert metadata['launch_arguments'] == {'airframe': 'source_one', 'world': 'cine_field'}


def test_run_metadata_records_the_config_digest():
    assert _metadata(config_sha1='c' * 40)['config_sha1'] == 'c' * 40


def test_run_metadata_labels_the_perception_source():
    assert _metadata(perception_source='detector')['perception_source'] == 'detector'


def test_run_metadata_leaves_out_perception_source_and_image_id_when_unknown():
    metadata = _metadata(perception_source=None, image_id=None)

    assert ('perception_source' in metadata, 'image_id' in metadata) == (False, False)


def test_run_metadata_records_image_id_when_known():
    assert _metadata(image_id='sha256:abc')['image_id'] == 'sha256:abc'


def test_run_metadata_records_the_navigation_source():
    assert _metadata(nav_source='vins')['nav_source'] == 'vins'


def test_run_metadata_does_not_claim_an_outcome_at_launch():
    assert 'outcome' not in _metadata()


def test_run_metadata_survives_a_json_round_trip():
    metadata = _metadata()

    assert json.loads(json.dumps(metadata)) == metadata


def test_waypoint_digest_is_the_sha1_of_the_file_contents(tmp_path):
    waypoint_file = tmp_path / 'waypoints.yaml'
    waypoint_file.write_bytes(b'- [0.0, 3.0, 2.5]\n')

    assert file_sha1(waypoint_file) == hashlib.sha1(b'- [0.0, 3.0, 2.5]\n').hexdigest()


def test_directory_digest_changes_when_a_file_changes(tmp_path):
    (tmp_path / 'a.json').write_text('{"x": 1}')
    before = directory_sha1(tmp_path)

    (tmp_path / 'a.json').write_text('{"x": 2}')

    assert directory_sha1(tmp_path) != before


def test_directory_digest_changes_when_a_file_is_renamed(tmp_path):
    (tmp_path / 'a.json').write_text('{}')
    before = directory_sha1(tmp_path)

    (tmp_path / 'a.json').rename(tmp_path / 'b.json')

    assert directory_sha1(tmp_path) != before


def test_directory_digest_covers_files_in_subdirectories(tmp_path):
    (tmp_path / 'sub').mkdir()
    (tmp_path / 'sub' / 'a.json').write_text('{}')
    before = directory_sha1(tmp_path)

    (tmp_path / 'sub' / 'a.json').write_text('{"x": 1}')

    assert directory_sha1(tmp_path) != before


def test_directory_digest_is_the_same_for_identical_trees(tmp_path):
    for name in ('first', 'second'):
        (tmp_path / name).mkdir()
        (tmp_path / name / 'a.json').write_text('{}')

    assert directory_sha1(tmp_path / 'first') == directory_sha1(tmp_path / 'second')


def test_empty_seed_argument_means_no_seed():
    assert parse_seed('') is None


def test_numeric_seed_argument_becomes_an_integer():
    assert parse_seed('42') == 42


def test_non_numeric_seed_argument_is_rejected():
    with pytest.raises(ValueError, match='seed'):
        parse_seed('abc')


def test_git_state_reads_commit_and_clean_flag_from_environment():
    environment = {'CINE_GIT_COMMIT': 'deadbeef', 'CINE_GIT_DIRTY': '0'}

    assert git_state_from_environment(environment) == {
        'CINE': {'commit': 'deadbeef', 'dirty': False},
    }


def test_git_state_reads_a_dirty_flag_of_one_as_dirty():
    environment = {'CINE_GIT_COMMIT': 'deadbeef', 'CINE_GIT_DIRTY': '1'}

    assert git_state_from_environment(environment)['CINE']['dirty'] is True


def test_git_state_without_environment_is_null_commit_and_null_dirty():
    assert git_state_from_environment({}) == {'CINE': {'commit': None, 'dirty': None}}


def test_git_state_with_an_unrecognised_dirty_value_is_null():
    environment = {'CINE_GIT_COMMIT': 'deadbeef', 'CINE_GIT_DIRTY': 'maybe'}

    assert git_state_from_environment(environment)['CINE']['dirty'] is None


def test_git_state_with_an_empty_commit_is_null():
    environment = {'CINE_GIT_COMMIT': '', 'CINE_GIT_DIRTY': '0'}

    assert git_state_from_environment(environment)['CINE']['commit'] is None


def test_scorer_pairs_a_run_by_the_metadata_instead_of_calling_it_unpaired():
    key = condition_key(_metadata(), 'run_a')

    assert key[0] == ('string', 'cine_field_mission')


def test_scorer_reads_the_clean_dirty_flag_as_known_and_clean():
    facts = _facts(_metadata())

    assert (facts.dirty, facts.dirty_flag_unknown) == (False, False)


def test_scorer_reports_a_missing_dirty_flag_as_unknown_not_as_dirty():
    facts = _facts(_metadata(git=git_state_from_environment({})))

    assert (facts.dirty, facts.dirty_flag_unknown) == (False, True)


def test_scorer_comparability_note_names_the_unknown_dirty_flag():
    facts = _facts(_metadata(git=git_state_from_environment({})))

    comparable, note = _comparability({'note': '', 'comparable': True}, [facts])

    assert comparable is False
    assert 'a run.json has a missing or unknown dirty flag, so it counts as dirty' in note


def test_mission_channel_measures_arrival_from_the_run_metadata():
    record = _record_with_run(
        _metadata(waypoints=[(0.0, 0.0, 1.0), (0.0, 5.0, 1.0)]),
        (0.0, 5.0, 1.0),
    )

    result = score_mission(record, load_scoring_settings())

    assert result.summary['completed'] == 1.0


def test_written_run_json_is_accepted_by_the_scorer_as_the_launch_wrote_it(tmp_path):
    bag_directory = tmp_path / 'bag'
    bag_directory.mkdir()
    metadata = _metadata(waypoints=[(0.0, 0.0, 1.0), (0.0, 5.0, 1.0)])

    written = write_run_metadata_when_directory_exists(
        bag_directory,
        metadata,
        timeout_s=0.0,
        wait=lambda: None,
    )
    run = json.loads(written.read_text())
    result = score_mission(_record_with_run(run, (0.0, 5.0, 1.0)), load_scoring_settings())

    assert (condition_key(run, 'run_a')[0], result.summary['completed']) == (
        ('string', 'cine_field_mission'),
        1.0,
    )


def test_run_metadata_is_written_as_run_json_in_the_bag_directory(tmp_path):
    bag_directory = tmp_path / 'bag'
    bag_directory.mkdir()

    written = write_run_metadata_when_directory_exists(
        bag_directory,
        {'seed': None},
        timeout_s=0.0,
        wait=lambda: None,
    )

    assert json.loads(written.read_text()) == {'seed': None}


def test_writer_waits_until_the_recorder_has_made_the_bag_directory(tmp_path):
    bag_directory = tmp_path / 'bag'

    written = write_run_metadata_when_directory_exists(
        bag_directory,
        {'seed': 1},
        timeout_s=5.0,
        wait=bag_directory.mkdir,
    )

    assert written == bag_directory / 'run.json'


def test_writer_gives_up_when_the_bag_directory_never_appears(tmp_path):
    with pytest.raises(FileNotFoundError, match='bag'):
        write_run_metadata_when_directory_exists(
            tmp_path / 'bag',
            {'seed': 1},
            timeout_s=0.0,
            wait=lambda: None,
        )
