from glob import glob
from setuptools import find_packages, setup
import os

package_name = "object_detection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "resource", "model"),
            glob("resource/model/yolov8n.pt"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cchyun",
    maintainer_email="cchyun@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "object_detection = object_detection.yolov8_ros2_pt:main",
        ],
    },
)
