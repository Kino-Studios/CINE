import json
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import pytest
from cine_gazebo.model_rendering import (
    CameraSpec,
    imu_noise_from_vins,
    render_model,
)
from cine_gazebo.recording import bag_directory_name, recorded_topics

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODEL_TEMPLATE = PACKAGE_ROOT / 'models' / 'cine_iris' / 'model.sdf.template'
# CI checks out CINE alone, so the sibling CINE-Sensing repository may be absent.
SHIPPED_GAZEBO_VINS = (
    PACKAGE_ROOT.parents[1]
    / 'CINE-Sensing'
    / 'cine_sensing_config'
    / 'config'
    / 'profiles'
    / 'gazebo'
    / 'vins.json'
)
EXPERIMENTAL = '{http://sdformat.org/schemas/experimental}'


def test_bag_name_carries_date_mode_world_and_wind():
    name = bag_directory_name(datetime(2026, 11, 14, 9, 5, 7), 'vins', 'cine_field', 'gusty')

    assert name == '2026-11-14_09-05-07_vins_cine_field_gusty'


def test_recording_includes_truth_and_estimates():
    topics = recorded_topics(0)

    assert {'/gazebo/drone_pose', '/gazebo/subject_pose'} <= set(topics)
    assert {'/vio/odometry', '/mavros/local_position/odom'} <= set(topics)


def test_recording_includes_every_setpoint_stage():
    topics = recorded_topics(0)

    assert 'drone_0_planning/pos_cmd' in topics
    assert {
        '/cinematography/translational_position_setpoints',
        '/cinematography/translational_velocity_setpoints',
        '/cinematography/translational_acceleration_setpoints',
        '/cinematography/attitude_position_setpoints',
        '/cinematography/attitude_velocity_setpoints',
    } <= set(topics)
    assert {
        '/mavros/setpoint_position/local',
        '/mavros/setpoint_velocity/cmd_vel',
        '/mavros/setpoint_accel/accel',
        '/mavros/setpoint_attitude/attitude',
        '/mavros/setpoint_attitude/cmd_vel',
    } <= set(topics)


def test_recording_includes_the_anchor_focus_distance():
    assert '/cinematography/focus_target_distance' in recorded_topics(0)


def test_recording_includes_the_topics_cine_eval_scores_focus_and_motion_from():
    topics = recorded_topics(0)

    assert {'/cinematography/framing_active', '/cinematography/subject_position'} <= set(topics)


def test_recording_includes_the_relayed_odometry_and_the_vins_imu_rate_odometry():
    topics = recorded_topics(0)

    assert {'/cinematography/drone_odometry', '/imu_propagate'} <= set(topics)


def test_recording_names_the_planner_command_after_the_drone_id():
    assert 'drone_3_planning/pos_cmd' in recorded_topics(3)


def _rendered_imu_noise_stddev(group, axis):
    vins_config = json.loads(SHIPPED_GAZEBO_VINS.read_text())['vins_config']
    camera = CameraSpec(1152, 648, 497.0, 497.0, 576.0, 324.0, 0.0, 0.0, 0.0, 0.0, 30.0, 0.007)
    model = ET.fromstring(
        render_model(MODEL_TEMPLATE.read_text(), camera, imu_noise_from_vins(vins_config)),
    )
    imu_sensor = model.find(
        f"model/include/{EXPERIMENTAL}params/sensor[@element_id='imu_link::imu_sensor']",
    )

    return float(imu_sensor.find(f'imu/{group}/{axis}/noise/stddev').text)


@pytest.mark.skipif(
    not SHIPPED_GAZEBO_VINS.is_file(),
    reason='CINE-Sensing is not checked out beside CINE',
)
def test_rendered_imu_noise_uses_vins_values():
    assert _rendered_imu_noise_stddev('linear_acceleration', 'x') == 0.1
    assert _rendered_imu_noise_stddev('angular_velocity', 'x') == 0.01
