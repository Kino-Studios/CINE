import os
from glob import glob

from setuptools import setup

package_name = 'cine_gazebo'
share_directory = os.path.join('share', package_name)

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_directory, ['package.xml']),
        (os.path.join(share_directory, 'launch'), glob('launch/*.launch.py')),
        (os.path.join(share_directory, 'worlds'), glob('worlds/*.sdf.template')),
        (os.path.join(share_directory, 'models'), ['models/ATTRIBUTION.md']),
        (os.path.join(share_directory, 'models', 'cine_iris'), glob('models/cine_iris/*')),
        (
            os.path.join(share_directory, 'models', 'cine_source_one'),
            ['models/cine_source_one/model.config', 'models/cine_source_one/model.sdf.template'],
        ),
        (
            os.path.join(share_directory, 'models', 'cine_source_one', 'meshes'),
            glob('models/cine_source_one/meshes/*'),
        ),
        (os.path.join(share_directory, 'config'), glob('config/*.yaml')),
        (os.path.join(share_directory, 'config', 'ardupilot'), glob('config/ardupilot/*.parm')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'subject_pose_publisher = cine_gazebo.subject_pose_publisher:main',
            'sitl_flight_starter = cine_gazebo.sitl_flight_starter:main',
        ],
    },
    zip_safe=True,
    maintainer='fishnos',
    maintainer_email='fishnos@users.noreply.github.com',
    description=(
        "Gazebo world, drone model with the Pi camera's lens, and Gazebo-to-ROS bridges for "
        'flying the CINE stack against ArduPilot SITL.'
    ),
    license='GPL-3.0-only',
    extras_require={'test': ['pytest']},
)
