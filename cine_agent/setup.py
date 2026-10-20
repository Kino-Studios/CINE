import os
from glob import glob

from setuptools import Distribution, find_packages, setup

# setuptools' own install_data command; the distutils module it used to come from is
# gone from the standard library since Python 3.12.
_install_data = Distribution().get_command_class('install_data')

package_name = 'cine_agent'

ros2_executables = [
    os.path.join('scripts', 'ros2_nodes', 'cinematographic_adjustment_node'),
    os.path.join('scripts', 'ros2_nodes', 'cost_function_calculator'),
    os.path.join('scripts', 'ros2_nodes', 'waypoint_sender'),
    os.path.join('scripts', 'ros2_nodes', 'cinematic_bspline_filter'),
    os.path.join('scripts', 'ros2_nodes', 'trajectory_data_logger'),
    os.path.join('scripts', 'ros2_nodes', 'noisy_odometry_node'),
    os.path.join('scripts', 'ros2_nodes', 'cinematic_waypoint_generator'),
    os.path.join('scripts', 'ros2_nodes', 'simulated_object_detector'),
    os.path.join('scripts', 'ros2_nodes', 'simulated_focus_publisher'),
    os.path.join('scripts', 'ros2_nodes', 'simulated_light_publisher'),
    os.path.join('scripts', 'ros2_nodes', 'baseline_collection'),
    os.path.join('scripts', 'ros2_nodes', 'cinematic_collection'),
]

try:
    from colcon_core.distutils.commands.symlink_data import (
        symlink_data as _symlink_data,  # noqa: E402
    )
except ImportError:
    _symlink_data = None


# colcon switches between copied and symlinked installs in the same install tree; an
# existing file or symlink at the target makes the other mode fail, so remove it first.
def _cleanup_existing_targets(self):
    if not self.data_files:
        return
    for entry in self.data_files:
        if isinstance(entry, str):
            target = os.path.join(self.install_dir, os.path.basename(entry))
            _remove_path(target)
            continue

        directory = entry[0]
        if not os.path.isabs(directory):
            directory = os.path.join(self.install_dir, directory)

        if not entry[1]:
            continue

        for data in entry[1]:
            data_path = os.path.join(directory, os.path.basename(data))
            _remove_path(data_path)


def _remove_path(path):
    try:
        if os.path.lexists(path):
            os.remove(path)
    except OSError:
        pass


class install_data(_install_data):
    def run(self):
        _cleanup_existing_targets(self)
        super().run()


# Defined unconditionally (on install_data when colcon is absent, then left unused) so
# the class does not open an `if` block, where ruff and tools/format.py disagree on spacing.
class symlink_data(_symlink_data if _symlink_data is not None else _install_data):
    def run(self):
        _cleanup_existing_targets(self)
        super().run()


cmdclass = {'install_data': install_data}
if _symlink_data is not None:
    cmdclass['symlink_data'] = symlink_data

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), ros2_executables),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishnos',
    maintainer_email='fishnos@users.noreply.github.com',
    description=(
        'Cinematography agent for the CINE drone: cost functions, setpoint adjustment, '
        'waypoint generation and trajectory data logging.'
    ),
    license='GPL-3.0-only',
    extras_require={'test': ['pytest']},
    cmdclass=cmdclass,
)
