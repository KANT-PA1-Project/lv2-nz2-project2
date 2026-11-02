from glob import glob
from setuptools import find_packages, setup

package_name = "nz2_signal"

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
    description="NZ2 신호등·바코드 판독",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "signal_decoder_node = nz2_signal.signal_decoder_node:main",
            "barcode_reader_node = nz2_signal.barcode_reader_node:main",
        ],
    },
)
