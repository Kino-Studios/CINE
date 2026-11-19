from pathlib import Path

import numpy as np
import pytest
from cine_eval.record import FlightRecord
from cine_eval.scene import (
    Box,
    Cylinder,
    WorldError,
    distance_to_obstacle,
    load_scene,
    load_world,
    resolve_world_path,
    skipped_models,
)

import cine_eval

TEMPLATE = (
    Path(cine_eval.__file__).resolve().parents[2] / 'cine_gazebo/worlds/cine_field.sdf.template'
)


def _by_name(scene):
    return {obstacle.name: obstacle for obstacle in scene}


def test_load_scene_reads_the_five_static_obstacles():
    scene = _by_name(load_scene(TEMPLATE))

    assert sorted(scene) == [
        'hedge_east',
        'hedge_north',
        'pillar_far_north',
        'pillar_north_east',
        'pillar_north_west',
    ]
    assert scene['pillar_north_west'].radius == 0.8
    assert scene['pillar_north_west'].centre.tolist() == [-8.0, 9.0, 3.0]
    assert scene['hedge_east'].size.tolist() == [1.5, 6.0, 3.0]


def test_load_scene_names_the_included_runway_as_skipped():
    assert 'runway (include)' in skipped_models(TEMPLATE)


def test_load_scene_honours_degrees_and_ignores_roll_and_pitch(tmp_path):
    world = tmp_path / 'w.sdf'
    world.write_text(
        '<sdf version="1.9"><world name="w">{{wind}}'
        '<model name="slab"><static>true</static><pose degrees="true">1 2 3 10 20 90</pose>'
        '<link name="l"><collision name="c"><geometry><box><size>1 2 3</size></box></geometry>'
        '</collision></link></model></world></sdf>',
    )

    slab = load_scene(world)[0]

    assert slab.yaw == pytest.approx(np.pi / 2)


def test_plane_collision_is_skipped_by_name(tmp_path):
    world = tmp_path / 'w.sdf'
    world.write_text(
        '<sdf version="1.9"><world name="w">'
        '<model name="ground"><static>true</static><link name="l"><collision name="c">'
        '<geometry><plane><normal>0 0 1</normal></plane></geometry></collision></link></model>'
        '</world></sdf>',
    )

    assert load_scene(world) == []
    assert skipped_models(world) == ['ground (plane)']


def test_distance_to_box_and_cylinder_matches_hand_geometry():
    box = Box('box', np.array([0.0, 0.0, 1.0]), np.array([2.0, 2.0, 2.0]), 0.0)
    cylinder = Cylinder('post', np.array([0.0, 0.0, 3.0]), 1.0, 6.0)

    outside_box = distance_to_obstacle(np.array([[3.0, 0.0, 1.0]]), box)
    inside_cylinder = distance_to_obstacle(np.array([[0.2, 0.0, 3.0]]), cylinder)
    outside_cylinder = distance_to_obstacle(np.array([[4.0, 0.0, 3.0]]), cylinder)

    assert outside_box[0] == pytest.approx(2.0)
    assert inside_cylinder[0] == pytest.approx(-0.8)
    assert outside_cylinder[0] == pytest.approx(3.0)


def test_distance_to_a_rotated_box_uses_the_box_frame():
    box = Box('box', np.zeros(3), np.array([4.0, 2.0, 2.0]), np.pi / 4)
    along_long_axis = 2.5 * np.array([[np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0]])

    assert distance_to_obstacle(along_long_axis, box)[0] == pytest.approx(0.5)


def test_distance_above_a_cylinder_end_cap_is_vertical():
    cylinder = Cylinder('post', np.array([0.0, 0.0, 3.0]), 1.0, 6.0)

    assert distance_to_obstacle(np.array([[0.0, 0.0, 7.5]]), cylinder)[0] == pytest.approx(1.5)


def test_resolve_world_path_prefers_the_run_file_entry(tmp_path):
    record = FlightRecord(streams={}, metadata={'run': {'world_file': str(tmp_path / 'x.sdf')}})

    assert resolve_world_path(record) == tmp_path / 'x.sdf'


def test_resolve_world_path_defaults_to_the_template():
    assert resolve_world_path(FlightRecord(streams={})) == TEMPLATE


def _write_world(tmp_path, static_text):
    world = tmp_path / 'w.sdf'
    static_element = '' if static_text is None else f'<static>{static_text}</static>'
    world.write_text(
        '<sdf version="1.9"><world name="w">'
        f'<model name="crate">{static_element}<link name="l"><collision name="c">'
        '<geometry><box><size>1 1 1</size></box></geometry></collision></link></model>'
        '</world></sdf>',
    )

    return world


def test_static_flag_written_as_one_is_an_obstacle(tmp_path):
    scene = load_scene(_write_world(tmp_path, ' 1 '))

    assert [obstacle.name for obstacle in scene] == ['crate']


def test_non_static_model_is_named_as_skipped(tmp_path):
    world = _write_world(tmp_path, 'false')

    assert load_scene(world) == []
    assert skipped_models(world) == ['crate (not static)']


def test_load_world_reads_the_world_name_as_the_frame_its_poses_are_in():
    assert load_world(TEMPLATE).frame == 'cine_field'


def test_world_file_cut_off_mid_element_raises_world_error_naming_the_file(tmp_path):
    world = tmp_path / 'w.sdf'
    world.write_text('<sdf version="1.9"><world name="w"><model name="crate">')

    with pytest.raises(WorldError, match='w.sdf'):
        load_world(world)


def test_missing_world_file_raises_world_error(tmp_path):
    with pytest.raises(WorldError, match='cannot read'):
        load_world(tmp_path / 'absent.sdf')


def test_world_file_without_a_world_element_raises_world_error(tmp_path):
    world = tmp_path / 'w.sdf'
    world.write_text('<sdf version="1.9"></sdf>')

    with pytest.raises(WorldError, match='no <world>'):
        load_world(world)


def test_box_without_a_size_raises_world_error_naming_the_model(tmp_path):
    world = tmp_path / 'w.sdf'
    world.write_text(
        '<sdf version="1.9"><world name="w">'
        '<model name="crate"><static>true</static><link name="l"><collision name="c">'
        '<geometry><box></box></geometry></collision></link></model></world></sdf>',
    )

    with pytest.raises(WorldError, match='crate'):
        load_world(world)


def test_pose_with_fewer_than_six_values_raises_world_error_naming_the_model(tmp_path):
    world = tmp_path / 'w.sdf'
    world.write_text(
        '<sdf version="1.9"><world name="w">'
        '<model name="crate"><static>true</static><pose>1 2 3</pose><link name="l">'
        '<collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision>'
        '</link></model></world></sdf>',
    )

    with pytest.raises(WorldError, match='crate'):
        load_world(world)
