from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cinematographic_adjustment_agent'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vtsyp',
    maintainer_email='your_email@example.com',
    description='Cinematographic adjustment agent for trajectory optimization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cinematographic_adjustment_node = cinematographic_adjustment_agent.setpoint_adjustment.cinematographic_adjustment_node:main',
            'cost_function_calculator = cinematographic_adjustment_agent.cost_functions.cost_functions:main',
            'waypoint_sender = cinematographic_adjustment_agent.setpoint_sender.waypoint_sender:main',
            'cinematic_bspline_filter = cinematographic_adjustment_agent.setpoint_adjustment.cinematic_bspline_filter:main',
            'trajectory_data_logger = cinematographic_adjustment_agent.data_collection.trajectory_data_logger:main',
            'cinematic_waypoint_generator = cinematographic_adjustment_agent.waypoint_automation.cinematic_waypoint_generator:main',
            'simulated_object_detector = cinematographic_adjustment_agent.object_detection.simulated_object_detector:main',
        ],
    },
)
