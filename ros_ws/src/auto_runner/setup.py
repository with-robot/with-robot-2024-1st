from glob import glob
from setuptools import find_packages, setup

package_name = "auto_runner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]) + [f"{package_name}.lib"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dykwon",
    maintainer_email="c4now@naver.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"runner = {package_name}.runner:main",
            f"runner2 = {package_name}.runner2:main",
        ],
    },
)
