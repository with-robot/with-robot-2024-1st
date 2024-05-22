from glob import glob
from setuptools import find_packages, setup
from ament_index_python.resources import get_search_paths
import os

package_name = "gmapping"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]) + [f"{package_name}.utils_lib"],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="michael kwon",
    maintainer_email="c4now@naver.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"occupancy_gridmap = {package_name}.occupancy_gridmap:main",
            f"create_map = {package_name}.gridmap_create:main",
        ],
    },
)
