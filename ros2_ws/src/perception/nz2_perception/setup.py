# =============================================================
# 패키지 : nz2_perception
# 파일   : setup.py
# 역할   : Python 패키지 설치 설정. 노드 실행 이름(entry_points)과 launch·config 설치 경로를 정한다
#
# [해야 할 일 / TODO]
#   - 새 노드 파일을 만들면 console_scripts에 '<실행 이름> = nz2_perception.<모듈>:main' 추가
# [ROS2 메모] console_scripts에 등록한 이름으로 `ros2 run nz2_perception <이름>` 실행
# =============================================================
import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'nz2_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='천경호',
    maintainer_email='ckhy2k@gmail.com',
    description='NZ2 인지 레이어: 퍽·태그·신호등 인식 (스켈레톤)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'puck_detector_node = nz2_perception.puck_detector_node:main',
            'puck_pose_estimator_node = nz2_perception.puck_pose_estimator_node:main',
            'tag_reader_node = nz2_perception.tag_reader_node:main',
            'traffic_light_node = nz2_perception.traffic_light_node:main',
        ],
    },
)
