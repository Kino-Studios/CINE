"""Renders the Gazebo drone model and world from the simulated camera's calibration.

The model template carries `{{camera}}` and `{{imu_noise}}` and the world template carries
`{{wind}}`; the launch file fills them in and writes the results where Gazebo looks for
models and worlds. Kept free of ROS imports so CI can test it without a ROS install.
"""

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

CAMERA_TOPIC = 'camera/image_raw'
CAMERA_INFO_TOPIC = 'camera/camera_info'
# Gazebo normalises pixel values to [0, 1], so this is about two grey levels out of 255.
DEFAULT_CAMERA_NOISE_STDDEV = 0.007
# Gazebo models IMU bias as a first-order Gauss-Markov process. With a correlation time far
# longer than any flight, that process is a slow random walk like the one VINS assumes with
# acc_w and gyr_w. The 1000 s is an assumption, not a measured value.
IMU_BIAS_CORRELATION_TIME_S = 1000.0
# WindEffects' time constants for following the seed velocity; Gazebo's own wind example
# uses these.
WIND_MAGNITUDE_RISE_TIME_S = 10.0
WIND_DIRECTION_RISE_TIME_S = 30.0


@dataclass(frozen=True)
class CameraSpec:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    k1: float
    k2: float
    p1: float
    p2: float
    rate_hz: float
    noise_stddev: float


@dataclass(frozen=True)
class ImuNoise:
    accel_stddev: float
    gyro_stddev: float
    accel_dynamic_bias_stddev: float
    gyro_dynamic_bias_stddev: float


@dataclass(frozen=True)
class WindProfile:
    """Mean wind in the world frame (x east, y north, m/s) and the spread around it.

    `horizontal_direction_stddev` is in radians.
    """

    linear_velocity: tuple[float, float, float]
    horizontal_magnitude_stddev: float
    horizontal_direction_stddev: float
    vertical_stddev: float


@dataclass(frozen=True)
class MassBlock:
    """A uniform box of `mass` kg centred at `position` (m) with edge lengths `size` (m)."""

    mass: float
    position: tuple[float, float, float]
    size: tuple[float, float, float]


def camera_spec_from_vins_camera(vins_camera: dict, rate_hz: float) -> CameraSpec:
    """Build the simulated camera from a VINS camera config (`vins_camera.json`)."""

    projection = vins_camera['projection_parameters']
    distortion = vins_camera['distortion_parameters']

    return CameraSpec(
        width=int(vins_camera['image_width']),
        height=int(vins_camera['image_height']),
        fx=float(projection['fx']),
        fy=float(projection['fy']),
        cx=float(projection['cx']),
        cy=float(projection['cy']),
        k1=float(distortion['k1']),
        k2=float(distortion['k2']),
        p1=float(distortion['p1']),
        p2=float(distortion['p2']),
        rate_hz=float(rate_hz),
        noise_stddev=DEFAULT_CAMERA_NOISE_STDDEV,
    )


def composite_inertia(
    blocks: list[MassBlock],
) -> tuple[float, tuple[float, float, float], tuple[float, float, float]]:
    """Return total mass, centre of gravity and diagonal inertia about that centre.

    Each block's own box inertia is moved to the centre of gravity with the parallel-axis
    theorem. Off-diagonal terms are dropped; blocks laid out symmetrically about the
    centre's axes, as the Source One's are, have none.
    """

    total_mass = sum(block.mass for block in blocks)
    centre_of_gravity = tuple(
        sum(block.mass * block.position[axis] for block in blocks) / total_mass for axis in range(3)
    )
    inertia = [0.0, 0.0, 0.0]

    for block in blocks:
        length_x, length_y, length_z = block.size
        own_inertia = (
            block.mass * (length_y**2 + length_z**2) / 12.0,
            block.mass * (length_x**2 + length_z**2) / 12.0,
            block.mass * (length_x**2 + length_y**2) / 12.0,
        )
        offset_x, offset_y, offset_z = (
            block.position[axis] - centre_of_gravity[axis] for axis in range(3)
        )
        inertia[0] += own_inertia[0] + block.mass * (offset_y**2 + offset_z**2)
        inertia[1] += own_inertia[1] + block.mass * (offset_x**2 + offset_z**2)
        inertia[2] += own_inertia[2] + block.mass * (offset_x**2 + offset_y**2)

    return total_mass, centre_of_gravity, tuple(inertia)


def horizontal_fov(spec: CameraSpec) -> float:
    return 2.0 * math.atan(spec.width / (2.0 * spec.fx))


def render_camera_block(spec: CameraSpec) -> str:
    """Return the SDF `<camera>` element for the spec."""

    return f"""<camera name="pi_camera">
  <camera_info_topic>{CAMERA_INFO_TOPIC}</camera_info_topic>
  <horizontal_fov>{horizontal_fov(spec)!r}</horizontal_fov>
  <image>
    <width>{spec.width}</width>
    <height>{spec.height}</height>
    <format>R8G8B8</format>
  </image>
  <clip>
    <near>0.05</near>
    <far>200</far>
  </clip>
  <lens>
    <intrinsics>
      <fx>{spec.fx!r}</fx>
      <fy>{spec.fy!r}</fy>
      <cx>{spec.cx!r}</cx>
      <cy>{spec.cy!r}</cy>
      <s>0</s>
    </intrinsics>
  </lens>
  <distortion>
    <k1>{spec.k1!r}</k1>
    <k2>{spec.k2!r}</k2>
    <k3>0</k3>
    <p1>{spec.p1!r}</p1>
    <p2>{spec.p2!r}</p2>
    <center>0.5 0.5</center>
  </distortion>
  <noise>
    <type>gaussian</type>
    <mean>0</mean>
    <stddev>{spec.noise_stddev!r}</stddev>
  </noise>
</camera>"""


def _render_camera_sensor(spec: CameraSpec) -> str:
    return f"""<sensor name="pi_camera" type="camera">
  <always_on>1</always_on>
  <update_rate>{spec.rate_hz!r}</update_rate>
  <topic>{CAMERA_TOPIC}</topic>
  {render_camera_block(spec)}
</sensor>"""


def imu_noise_from_vins(vins_config: dict) -> ImuNoise:
    """Take the IMU noise VINS assumes (`acc_n`, `gyr_n`, `acc_w`, `gyr_w`)."""

    return ImuNoise(
        accel_stddev=float(vins_config['acc_n']),
        gyro_stddev=float(vins_config['gyr_n']),
        accel_dynamic_bias_stddev=float(vins_config['acc_w']),
        gyro_dynamic_bias_stddev=float(vins_config['gyr_w']),
    )


def _render_axis_noise(axis: str, stddev: float, dynamic_bias_stddev: float) -> str:
    return f"""<{axis}>
  <noise type="gaussian">
    <mean>0</mean>
    <stddev>{stddev!r}</stddev>
    <dynamic_bias_stddev>{dynamic_bias_stddev!r}</dynamic_bias_stddev>
    <dynamic_bias_correlation_time>{IMU_BIAS_CORRELATION_TIME_S!r}</dynamic_bias_correlation_time>
  </noise>
</{axis}>"""


def _render_imu_axes_noise(imu: ImuNoise) -> str:
    angular_velocity = '\n'.join(
        _render_axis_noise(axis, imu.gyro_stddev, imu.gyro_dynamic_bias_stddev)
        for axis in ('x', 'y', 'z')
    )
    linear_acceleration = '\n'.join(
        _render_axis_noise(axis, imu.accel_stddev, imu.accel_dynamic_bias_stddev)
        for axis in ('x', 'y', 'z')
    )

    return f"""<angular_velocity>
{angular_velocity}
</angular_velocity>
<linear_acceleration>
{linear_acceleration}
</linear_acceleration>"""


def _render_imu_noise(imu: ImuNoise) -> str:
    # The IMU belongs to the included iris_with_standoffs, so the noise reaches it through
    # the include's parameter passing rather than by editing that model.

    return f"""<sensor element_id="imu_link::imu_sensor">
  <imu action="add">
{_render_imu_axes_noise(imu)}
  </imu>
</sensor>"""


def render_model(template: str, camera: CameraSpec, imu: ImuNoise | None) -> str:
    """Fill a model template's placeholders.

    `{{imu_noise}}` is a whole `<sensor>` override for a model that includes the Iris;
    `{{imu_axes_noise}}` is the noise elements alone, for a model that declares its own
    `<imu>` (the Source One).
    """

    imu_noise = '' if imu is None else _render_imu_noise(imu)
    imu_axes_noise = '' if imu is None else _render_imu_axes_noise(imu)

    return (
        template.replace('{{camera}}', _render_camera_sensor(camera))
        .replace('{{imu_noise}}', imu_noise)
        .replace('{{imu_axes_noise}}', imu_axes_noise)
    )


def load_wind_profiles(path) -> dict[str, WindProfile]:
    with open(Path(path), encoding='utf-8') as profiles_file:
        raw_profiles = yaml.safe_load(profiles_file)

    return {
        name: WindProfile(
            linear_velocity=tuple(float(value) for value in values['linear_velocity']),
            horizontal_magnitude_stddev=float(values['horizontal_magnitude_stddev']),
            horizontal_direction_stddev=float(values['horizontal_direction_stddev']),
            vertical_stddev=float(values['vertical_stddev']),
        )
        for name, values in raw_profiles.items()
    }


def _render_wind(wind: WindProfile) -> str:
    velocity = ' '.join(repr(component) for component in wind.linear_velocity)
    # WindEffects works on the wind direction in degrees.
    direction_stddev_degrees = math.degrees(wind.horizontal_direction_stddev)

    return f"""<wind>
      <linear_velocity>{velocity}</linear_velocity>
    </wind>
    <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <force_approximation_scaling_factor>1</force_approximation_scaling_factor>
      <horizontal>
        <magnitude>
          <time_for_rise>{WIND_MAGNITUDE_RISE_TIME_S!r}</time_for_rise>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>{wind.horizontal_magnitude_stddev!r}</stddev>
          </noise>
        </magnitude>
        <direction>
          <time_for_rise>{WIND_DIRECTION_RISE_TIME_S!r}</time_for_rise>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>{direction_stddev_degrees!r}</stddev>
          </noise>
        </direction>
      </horizontal>
      <vertical>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>{wind.vertical_stddev!r}</stddev>
        </noise>
      </vertical>
    </plugin>"""


def render_world(template: str, wind: WindProfile) -> str:
    return template.replace('{{wind}}', _render_wind(wind))
