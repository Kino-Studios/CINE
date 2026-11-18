import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from cine_gazebo.model_rendering import (
    CameraSpec,
    ImuNoise,
    MassBlock,
    WindProfile,
    camera_spec_from_vins_camera,
    composite_inertia,
    horizontal_fov,
    imu_noise_from_vins,
    load_wind_profiles,
    render_camera_block,
    render_model,
    render_world,
)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / 'config'
MODEL_TEMPLATE = PACKAGE_ROOT / 'models' / 'cine_iris' / 'model.sdf.template'
WORLD_TEMPLATE = PACKAGE_ROOT / 'worlds' / 'cine_field.sdf.template'
# CI checks out CINE alone, so the sibling CINE-Sensing repository may be absent.
SHIPPED_GAZEBO_CAMERA = (
    PACKAGE_ROOT.parents[1]
    / 'CINE-Sensing'
    / 'cine_sensing_config'
    / 'config'
    / 'profiles'
    / 'gazebo'
    / 'vins_camera.json'
)
EXPERIMENTAL = '{http://sdformat.org/schemas/experimental}'

FULL_FOV = {
    'image_width': 1152,
    'image_height': 648,
    'fx': 497.5985575,
    'fy': 497.5985575,
    'cx': 576.0,
    'cy': 324.0,
    'k1': -0.0578246072,
    'k2': -0.0328310705,
    'p1': -0.00243136122,
    'p2': 0.000290213152,
}

VINS_CAMERA = {
    'model_type': 'PINHOLE',
    'image_width': 640,
    'image_height': 480,
    'distortion_parameters': {'k1': -0.1, 'k2': 0.02, 'p1': 0.003, 'p2': -0.004},
    'projection_parameters': {'fx': 400.0, 'fy': 401.0, 'cx': 320.5, 'cy': 240.5},
}

GUSTY = WindProfile(
    linear_velocity=(4.0, 0.0, 0.0),
    horizontal_magnitude_stddev=1.5,
    horizontal_direction_stddev=0.3,
    vertical_stddev=0.5,
)


def _spec(values):
    return CameraSpec(
        width=values['image_width'],
        height=values['image_height'],
        fx=values['fx'],
        fy=values['fy'],
        cx=values['cx'],
        cy=values['cy'],
        k1=values['k1'],
        k2=values['k2'],
        p1=values['p1'],
        p2=values['p2'],
        rate_hz=30.0,
        noise_stddev=0.007,
    )


def _rendered_model(imu=None):
    return ET.fromstring(render_model(MODEL_TEMPLATE.read_text(), _spec(FULL_FOV), imu))


def _rendered_world(wind):
    return ET.fromstring(render_world(WORLD_TEMPLATE.read_text(), wind))


def _bridge_entries_by_ros_topic():
    entries = yaml.safe_load((CONFIG / 'bridge.yaml').read_text())

    return {entry['ros_topic_name']: entry for entry in entries}


def test_horizontal_fov_matches_full_fov_calibration():
    assert horizontal_fov(_spec(FULL_FOV)) == pytest.approx(1.7165910783, abs=1e-9)


def test_camera_block_carries_calibration_distortion():
    block = ET.fromstring(render_camera_block(_spec(FULL_FOV)))

    assert float(block.find('distortion/k1').text) == pytest.approx(-0.0578246072)
    assert float(block.find('distortion/p2').text) == pytest.approx(0.000290213152)


def test_camera_block_renders_at_streamed_resolution():
    block = ET.fromstring(render_camera_block(_spec(FULL_FOV)))

    assert (block.find('image/width').text, block.find('image/height').text) == ('1152', '648')


def test_camera_block_sets_horizontal_fov_from_focal_length():
    block = ET.fromstring(render_camera_block(_spec(FULL_FOV)))

    assert float(block.find('horizontal_fov').text) == pytest.approx(1.7165910783, abs=1e-9)


def test_camera_block_carries_lens_intrinsics():
    block = ET.fromstring(render_camera_block(_spec(FULL_FOV)))

    assert float(block.find('lens/intrinsics/fx').text) == pytest.approx(497.5985575)
    assert float(block.find('lens/intrinsics/cy').text) == pytest.approx(324.0)


def test_camera_block_has_no_third_radial_term():
    block = ET.fromstring(render_camera_block(_spec(FULL_FOV)))

    assert float(block.find('distortion/k3').text) == 0.0


def test_camera_block_centres_distortion_on_the_image():
    block = ET.fromstring(render_camera_block(_spec(FULL_FOV)))

    assert block.find('distortion/center').text == '0.5 0.5'


def test_camera_block_adds_gaussian_image_noise():
    block = ET.fromstring(render_camera_block(_spec(FULL_FOV)))

    assert block.find('noise/type').text == 'gaussian'
    assert float(block.find('noise/stddev').text) == pytest.approx(0.007)


def test_camera_block_publishes_camera_info_beside_the_image():
    block = ET.fromstring(render_camera_block(_spec(FULL_FOV)))

    assert block.find('camera_info_topic').text == 'camera/camera_info'


def test_camera_spec_takes_intrinsics_from_projection_parameters():
    spec = camera_spec_from_vins_camera(VINS_CAMERA, rate_hz=30.0)

    assert (spec.fx, spec.fy, spec.cx, spec.cy) == (400.0, 401.0, 320.5, 240.5)


def test_camera_spec_takes_distortion_from_distortion_parameters():
    spec = camera_spec_from_vins_camera(VINS_CAMERA, rate_hz=30.0)

    assert (spec.k1, spec.k2, spec.p1, spec.p2) == (-0.1, 0.02, 0.003, -0.004)


def test_camera_spec_takes_resolution_from_image_size():
    spec = camera_spec_from_vins_camera(VINS_CAMERA, rate_hz=30.0)

    assert (spec.width, spec.height) == (640, 480)


def test_camera_spec_runs_at_requested_rate():
    assert camera_spec_from_vins_camera(VINS_CAMERA, rate_hz=15.0).rate_hz == 15.0


def test_camera_spec_uses_default_image_noise():
    assert camera_spec_from_vins_camera(VINS_CAMERA, rate_hz=30.0).noise_stddev == 0.007


@pytest.mark.skipif(
    not SHIPPED_GAZEBO_CAMERA.is_file(), reason='CINE-Sensing is not checked out beside CINE'
)
def test_shipped_gazebo_profile_gives_full_fov_horizontal_fov():
    vins_camera = json.loads(SHIPPED_GAZEBO_CAMERA.read_text())

    spec = camera_spec_from_vins_camera(vins_camera, rate_hz=30.0)

    assert horizontal_fov(spec) == pytest.approx(1.7165910783, abs=1e-9)


def test_imu_noise_taken_from_vins_config():
    noise = imu_noise_from_vins({'acc_n': 0.1, 'gyr_n': 0.01, 'acc_w': 2e-5, 'gyr_w': 2e-5})

    assert noise == ImuNoise(0.1, 0.01, 2e-5, 2e-5)


def test_calm_wind_is_zero():
    assert load_wind_profiles(CONFIG / 'wind_profiles.yaml')['calm'].linear_velocity == (
        0.0,
        0.0,
        0.0,
    )


def test_gusty_wind_blows_from_the_west_at_four_metres_per_second():
    assert load_wind_profiles(CONFIG / 'wind_profiles.yaml')['gusty'].linear_velocity == (
        4.0,
        0.0,
        0.0,
    )


def test_gusty_wind_direction_stddev_is_in_radians():
    gusty = load_wind_profiles(CONFIG / 'wind_profiles.yaml')['gusty']

    assert gusty.horizontal_direction_stddev == 0.3


def test_rendered_model_is_valid_xml():
    ET.fromstring(render_model(MODEL_TEMPLATE.read_text(), _spec(FULL_FOV), None))


def test_rendered_model_leaves_no_placeholder():
    rendered = render_model(MODEL_TEMPLATE.read_text(), _spec(FULL_FOV), None)

    assert '{{' not in rendered


def test_rendered_model_mounts_camera_five_centimetres_ahead_of_base_link():
    pose = _rendered_model().find("model/link[@name='camera_link']/pose")

    assert (pose.get('relative_to'), pose.text) == (
        'iris_with_standoffs::base_link',
        '0.05 0 0 0 0 0',
    )


def test_rendered_model_fixes_camera_to_base_link():
    joint = _rendered_model().find("model/joint[@name='camera_joint']")

    assert (joint.get('type'), joint.find('parent').text) == (
        'fixed',
        'iris_with_standoffs::base_link',
    )


def test_rendered_model_publishes_images_on_camera_image_raw():
    sensor = _rendered_model().find("model/link[@name='camera_link']/sensor")

    assert sensor.find('topic').text == 'camera/image_raw'


def test_rendered_model_updates_camera_at_spec_rate():
    sensor = _rendered_model().find("model/link[@name='camera_link']/sensor")

    assert float(sensor.find('update_rate').text) == 30.0


def test_rendered_model_carries_the_camera_lens():
    camera = _rendered_model().find("model/link[@name='camera_link']/sensor/camera")

    assert float(camera.find('distortion/k2').text) == pytest.approx(-0.0328310705)


def test_rendered_model_enables_wind_on_included_base_link():
    params = _rendered_model().find(f'model/include/{EXPERIMENTAL}params')

    assert params.find("link[@element_id='base_link']/enable_wind").text == 'true'


def test_rendered_model_publishes_its_true_pose():
    plugin = _rendered_model().find("model/plugin[@name='gz::sim::systems::PosePublisher']")

    assert plugin.find('publish_model_pose').text == 'true'


def test_rendered_model_with_imu_noise_sets_accelerometer_stddev():
    params = _rendered_model(ImuNoise(0.1, 0.01, 2e-5, 3e-5)).find(
        f'model/include/{EXPERIMENTAL}params'
    )

    stddev = params.find(
        "sensor[@element_id='imu_link::imu_sensor']/imu/linear_acceleration/x/noise/stddev"
    )
    assert float(stddev.text) == 0.1


def test_rendered_model_with_imu_noise_sets_gyroscope_bias_random_walk():
    params = _rendered_model(ImuNoise(0.1, 0.01, 2e-5, 3e-5)).find(
        f'model/include/{EXPERIMENTAL}params'
    )

    bias = params.find(
        "sensor[@element_id='imu_link::imu_sensor']"
        '/imu/angular_velocity/z/noise/dynamic_bias_stddev'
    )
    assert float(bias.text) == 3e-5


def test_rendered_model_without_imu_noise_leaves_imu_untouched():
    params = _rendered_model(None).find(f'model/include/{EXPERIMENTAL}params')

    assert params.find("sensor[@element_id='imu_link::imu_sensor']") is None


def test_rendered_world_is_valid_xml():
    _rendered_world(GUSTY)


def test_rendered_world_leaves_no_placeholder():
    assert '{{' not in render_world(WORLD_TEMPLATE.read_text(), GUSTY)


def test_rendered_world_seeds_wind_velocity_from_profile():
    assert _rendered_world(GUSTY).find('world/wind/linear_velocity').text == '4.0 0.0 0.0'


def test_rendered_world_writes_wind_direction_noise_in_degrees():
    plugin = _rendered_world(GUSTY).find("world/plugin[@name='gz::sim::systems::WindEffects']")

    stddev = plugin.find('horizontal/direction/noise/stddev')
    assert float(stddev.text) == pytest.approx(17.1887338539)


def test_rendered_world_writes_wind_magnitude_noise_from_profile():
    plugin = _rendered_world(GUSTY).find("world/plugin[@name='gz::sim::systems::WindEffects']")

    assert float(plugin.find('horizontal/magnitude/noise/stddev').text) == 1.5


def test_rendered_world_writes_vertical_wind_noise_from_profile():
    plugin = _rendered_world(GUSTY).find("world/plugin[@name='gz::sim::systems::WindEffects']")

    assert float(plugin.find('vertical/noise/stddev').text) == 0.5


def test_rendered_world_spawns_cine_iris():
    world = _rendered_world(GUSTY)

    assert world.find("world/include[name='cine_iris']/uri").text == 'model://cine_iris'


def test_rendered_world_pins_the_walking_subject_to_fuel_version_one():
    skin = _rendered_world(GUSTY).find("world/actor[@name='subject']/skin/filename")

    assert (
        skin.text == 'https://fuel.gazebosim.org/1.0/Mingfei/models/actor/1/files/meshes/walk.dae'
    )


def test_rendered_world_walks_the_subject_round_a_closed_loop():
    waypoints = _rendered_world(GUSTY).findall(
        "world/actor[@name='subject']/script/trajectory/waypoint"
    )

    first_position = waypoints[0].find('pose').text.split()[:3]
    last_position = waypoints[-1].find('pose').text.split()[:3]
    assert first_position == last_position


def test_bridge_carries_gazebo_clock_to_ros_clock():
    clock = _bridge_entries_by_ros_topic()['/clock']

    assert (clock['gz_topic_name'], clock['gz_type_name']) == ('/clock', 'gz.msgs.Clock')


def test_bridge_reads_camera_info_where_the_camera_publishes_it():
    camera_info = _bridge_entries_by_ros_topic()['/camera/camera_info']

    assert camera_info['gz_topic_name'] == '/camera/camera_info'


def test_bridge_publishes_drone_true_pose_from_the_pose_publisher():
    drone_pose = _bridge_entries_by_ros_topic()['/gazebo/drone_pose']

    assert drone_pose['gz_topic_name'] == '/model/cine_iris/pose'


def test_source_one_blocks_give_measured_mass_and_inertia():
    motor_forward = 0.07265
    motor_sideways = 0.0865
    blocks = [
        MassBlock(0.036, (motor_forward, motor_sideways, 0.0), (0.03, 0.03, 0.03)),
        MassBlock(0.036, (motor_forward, -motor_sideways, 0.0), (0.03, 0.03, 0.03)),
        MassBlock(0.036, (-motor_forward, motor_sideways, 0.0), (0.03, 0.03, 0.03)),
        MassBlock(0.036, (-motor_forward, -motor_sideways, 0.0), (0.03, 0.03, 0.03)),
        MassBlock(0.300, (0.0, 0.0, 0.0), (0.14, 0.10, 0.03)),
        MassBlock(0.213, (0.0, 0.0, -0.026), (0.075, 0.035, 0.047)),
        MassBlock(0.100, (0.0, 0.0, 0.041), (0.065, 0.056, 0.03)),
    ]

    mass, centre_of_gravity, inertia = composite_inertia(blocks)

    assert mass == pytest.approx(0.757, abs=1e-3)
    assert centre_of_gravity[2] == pytest.approx(-0.0019, abs=5e-4)
    assert inertia[0] == pytest.approx(1.78e-3, rel=0.03)
    assert inertia[1] == pytest.approx(1.79e-3, rel=0.03)
    assert inertia[2] == pytest.approx(2.78e-3, rel=0.03)
