from glob import glob

from setuptools import find_packages, setup

package_name = 'cine_mlp_correction'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/mlp_correction.launch.py']),
        # Empty in a fresh clone; tools/download_assets.py fills it before the build.
        (f'share/{package_name}/weights', glob('weights/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishnos',
    maintainer_email='fishnos@users.noreply.github.com',
    description=(
        'Corrects the planned cinematic B-spline with a learned MLP (ONNX Runtime) before it '
        'reaches the trajectory server.'
    ),
    license='GPL-3.0-only',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mlp_node = cine_mlp_correction.mlp_node:main',
        ],
    },
)
