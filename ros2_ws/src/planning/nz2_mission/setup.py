from glob import glob
from setuptools import find_packages, setup

package_name = "nz2_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NZ2",
    maintainer_email="nz2@example.com",
    description="NZ2 미션 FSM",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_fsm_node = nz2_mission.mission_fsm_node:main",
            "fake_skill_server = nz2_mission.fake_skill_server:main",
        ],
    },
)
