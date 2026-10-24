import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def _virtual_env_site_packages():
    # ROS 2 installs the node with the system Python's shebang, which cannot see packages
    # pip installed in the workspace venv (torch, depth_anything_3), so they are passed in.
    virtual_env = os.environ.get('VIRTUAL_ENV')
    if not virtual_env:
        return {}
    python_version = f'python{sys.version_info.major}.{sys.version_info.minor}'
    site_packages = Path(virtual_env) / 'lib' / python_version / 'site-packages'
    existing_path = os.environ.get('PYTHONPATH', '')

    return {'PYTHONPATH': os.pathsep.join(filter(None, [str(site_packages), existing_path]))}


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='cine_depth',
                executable='depth_publisher_node',
                name='depth_publisher',
                output='screen',
                parameters=[{'use_sim_time': False}],
                # Keeps packages from ~/.local out of the node, so the venv's torch is the
                # one imported.
                additional_env={'PYTHONNOUSERSITE': '1', **_virtual_env_site_packages()},
            )
        ]
    )
