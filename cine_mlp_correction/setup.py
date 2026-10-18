from setuptools import find_packages, setup

package_name = "cine_mlp_correction"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/mlp_correction.launch.py"]),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "mlp_node = cine_mlp_correction.mlp_node:main",
        ],
    },
)
