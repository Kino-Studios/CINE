import os
from glob import glob

from setuptools import setup

package_name = 'cine_depth'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishnos',
    maintainer_email='fishnos@users.noreply.github.com',
    description=(
        'Publishes monocular depth images estimated by Depth-Anything from the drone camera feed.'
    ),
    license='GPL-3.0-only',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'depth_publisher_node = cine_depth.depth_publisher_node:main',
        ],
    },
)
