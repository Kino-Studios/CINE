import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from cine_gazebo.model_rendering import CameraSpec, ImuNoise, render_model

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODEL_DIRECTORY = PACKAGE_ROOT / 'models' / 'cine_source_one'
MODEL_TEMPLATE = MODEL_DIRECTORY / 'model.sdf.template'
PARAMETER_FILE = PACKAGE_ROOT / 'config' / 'ardupilot' / 'source_one.parm'

CAMERA = CameraSpec(
    width=1152,
    height=648,
    fx=497.5985575,
    fy=497.5985575,
    cx=576.0,
    cy=324.0,
    k1=-0.0578246072,
    k2=-0.0328310705,
    p1=-0.00243136122,
    p2=0.000290213152,
    rate_hz=30.0,
    noise_stddev=0.007,
)
IMU_NOISE = ImuNoise(
    accel_stddev=0.08,
    gyro_stddev=0.004,
    accel_dynamic_bias_stddev=0.00004,
    gyro_dynamic_bias_stddev=2e-6,
)


def _rendered_model(imu=None):
    return ET.fromstring(render_model(MODEL_TEMPLATE.read_text(), CAMERA, imu)).find('model')


def _link(model, name):
    return model.find(f"link[@name='{name}']")


def _pose_numbers(element):
    return [float(value) for value in element.find('pose').text.split()]


def _parameters():
    parameters = {}
    for line in PARAMETER_FILE.read_text().splitlines():
        content = line.split('#')[0].split()
        if content:
            parameters[content[0]] = float(content[1])

    return parameters


def test_source_one_model_config_names_model_sdf():
    config = ET.parse(MODEL_DIRECTORY / 'model.config').getroot()

    assert config.find('sdf').text == 'model.sdf'


def test_source_one_template_leaves_no_placeholder_once_rendered():
    rendered = render_model(MODEL_TEMPLATE.read_text(), CAMERA, IMU_NOISE)

    assert '{{' not in rendered


def test_source_one_base_link_carries_the_composite_mass():
    inertial = _link(_rendered_model(), 'base_link').find('inertial')

    assert float(inertial.find('mass').text) == pytest.approx(0.757, abs=1e-3)


def test_source_one_base_link_centre_of_gravity_sits_just_below_the_arm_plane():
    inertial = _link(_rendered_model(), 'base_link').find('inertial')

    assert _pose_numbers(inertial)[2] == pytest.approx(-0.0019, abs=5e-4)


def test_source_one_base_link_carries_the_composite_inertia():
    inertia = _link(_rendered_model(), 'base_link').find('inertial/inertia')

    assert float(inertia.find('ixx').text) == pytest.approx(1.78e-3, rel=0.03)
    assert float(inertia.find('iyy').text) == pytest.approx(1.79e-3, rel=0.03)
    assert float(inertia.find('izz').text) == pytest.approx(2.78e-3, rel=0.03)


def test_source_one_rotors_sit_at_the_corners_of_a_quad_x():
    model = _rendered_model()

    assert _pose_numbers(_link(model, 'rotor_0'))[:2] == pytest.approx([0.07265, -0.0865])
    assert _pose_numbers(_link(model, 'rotor_1'))[:2] == pytest.approx([-0.07265, 0.0865])
    assert _pose_numbers(_link(model, 'rotor_2'))[:2] == pytest.approx([0.07265, 0.0865])
    assert _pose_numbers(_link(model, 'rotor_3'))[:2] == pytest.approx([-0.07265, -0.0865])


def test_source_one_flies_the_whole_vehicle_at_the_measured_weight():
    model = _rendered_model()
    masses = [float(link.find('inertial/mass').text) for link in model.findall('link')]

    assert sum(masses) == pytest.approx(0.757, rel=0.01)


def _control(model, channel):
    return model.find(f"plugin[@name='ArduPilotPlugin']/control[@channel='{channel}']")


def test_source_one_maps_ardupilot_channels_to_rotor_joints_in_quad_x_order():
    model = _rendered_model()

    assert _control(model, 0).find('jointName').text == 'rotor_0_joint'
    assert _control(model, 1).find('jointName').text == 'rotor_1_joint'
    assert _control(model, 2).find('jointName').text == 'rotor_2_joint'
    assert _control(model, 3).find('jointName').text == 'rotor_3_joint'


def test_source_one_spins_motors_one_and_two_counter_clockwise_and_three_and_four_clockwise():
    model = _rendered_model()

    assert float(_control(model, 0).find('multiplier').text) == 3666
    assert float(_control(model, 1).find('multiplier').text) == 3666
    assert float(_control(model, 2).find('multiplier').text) == -3666
    assert float(_control(model, 3).find('multiplier').text) == -3666


def test_source_one_reads_the_autopilot_imu_from_its_base_link():
    model = _rendered_model()
    imu_name = model.find("plugin[@name='ArduPilotPlugin']/imuName").text

    assert imu_name == 'base_link::imu_sensor'


def _blades(model, rotor):
    return [
        plugin
        for plugin in model.findall("plugin[@name='gz::sim::systems::LiftDrag']")
        if plugin.find('link_name').text == rotor
    ]


def test_source_one_rotor_has_three_blades():
    assert len(_blades(_rendered_model(), 'rotor_0')) == 3


def test_source_one_blades_carry_the_gemfan_51366_lift_numbers():
    blade = _blades(_rendered_model(), 'rotor_0')[0]

    assert float(blade.find('a0').text) == 0.3
    assert float(blade.find('cla').text) == 4.25
    assert float(blade.find('area').text) == pytest.approx(2.308e-4)


def test_source_one_blades_press_at_forty_nine_millimetres_from_the_hub():
    blade = _blades(_rendered_model(), 'rotor_0')[0]
    centre_of_pressure = [float(value) for value in blade.find('cp').text.split()]

    assert centre_of_pressure == pytest.approx([0.0486, 0.0, 0.0])


def test_source_one_clockwise_rotor_moves_its_blades_the_opposite_way():
    model = _rendered_model()
    counter_clockwise_blade = _blades(model, 'rotor_0')[0]
    clockwise_blade = _blades(model, 'rotor_2')[0]

    assert counter_clockwise_blade.find('forward').text == '0 1 0'
    assert clockwise_blade.find('forward').text == '0 -1 0'


def test_source_one_mounts_the_camera_five_centimetres_ahead_of_base_link():
    camera_link = _link(_rendered_model(), 'camera_link')

    assert _pose_numbers(camera_link)[:3] == pytest.approx([0.05, 0.0, 0.0])
    assert camera_link.find("sensor[@name='pi_camera']") is not None


def test_source_one_imu_takes_the_vins_accelerometer_noise():
    sensor = _link(_rendered_model(IMU_NOISE), 'base_link').find("sensor[@name='imu_sensor']")

    stddev = sensor.find('imu/linear_acceleration/x/noise/stddev')

    assert float(stddev.text) == 0.08


def test_source_one_imu_without_noise_has_no_noise_element():
    sensor = _link(_rendered_model(None), 'base_link').find("sensor[@name='imu_sensor']")

    assert sensor.find('imu/linear_acceleration') is None


def test_source_one_enables_wind_on_the_base_link():
    assert _link(_rendered_model(), 'base_link').find('enable_wind').text == 'true'


def test_source_one_parameters_pick_the_x_frame():
    parameters = _parameters()

    assert parameters['FRAME_CLASS'] == 1
    assert parameters['FRAME_TYPE'] == 1


def test_source_one_parameters_carry_the_measured_motor_and_filter_values():
    parameters = _parameters()

    assert parameters['MOT_THST_EXPO'] == 0.65
    assert parameters['MOT_THST_HOVER'] == 0.22
    assert parameters['INS_GYRO_FILTER'] == 75


def test_source_one_parameters_derive_rate_filters_from_the_gyro_filter():
    parameters = _parameters()

    assert parameters['ATC_RAT_RLL_FLTD'] == 37.5
    assert parameters['ATC_RAT_PIT_FLTT'] == 37.5
    assert parameters['ATC_RAT_YAW_FLTE'] == 2


def _committed_mesh_references():
    model = _rendered_model()
    references = {element.text for element in model.iter('uri')}
    references |= {element.text for element in model.iter('albedo_map')}

    return {reference for reference in references if not reference.startswith('meshes/restricted_')}


def test_source_one_template_names_the_committed_meshes_and_texture():
    assert _committed_mesh_references() == {
        'meshes/carbon.obj',
        'meshes/carbon_twill.png',
        'meshes/airframe.obj',
        'meshes/pi5.obj',
        'meshes/camera.obj',
    }


def test_source_one_committed_meshes_named_by_the_template_are_in_the_model_folder():
    missing = [ref for ref in _committed_mesh_references() if not (MODEL_DIRECTORY / ref).is_file()]

    assert missing == []


def test_source_one_material_files_named_by_committed_meshes_are_beside_them():
    meshes = [MODEL_DIRECTORY / ref for ref in _committed_mesh_references() if ref.endswith('.obj')]
    material_files = {
        mesh.parent / line.split(maxsplit=1)[1]
        for mesh in meshes
        for line in mesh.read_text().splitlines()
        if line.startswith('mtllib ')
    }
    missing = [str(path) for path in material_files if not path.is_file()]

    assert material_files and missing == []
