import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'r12_swerve_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='R12 Maintainer',
    maintainer_email='user@example.com',
    description='R21 실물 스월브 결정론적 컨트롤러 + OpenCR 하드웨어 브리지',
    license='MIT',
    entry_points={
        'console_scripts': [
            'swerve_controller = r12_swerve_control.swerve_controller_node:main',
            'opencr_bridge = r12_swerve_control.opencr_bridge_node:main',
        ],
    },
)
