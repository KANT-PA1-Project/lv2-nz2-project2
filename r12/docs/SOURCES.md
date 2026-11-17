# 버전과 코드 근거

확인 기준: 2026-10-30. 움직이는 main 문서와 고정 태그 소스를 구분합니다.

| 사용 항목 | 근거 |
|---|---|
| Sim 6.1 / Lab EA 의존성 | [v3.0.0-EA pyproject](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0-EA/pyproject.toml) |
| 태스크·CLI | [EA 소스](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA) |
| URDF 변환 | [convert_urdf.py](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0-EA/scripts/tools/convert_urdf.py) |
| 위치 도달 환경 | [ReachEnvCfg](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0-EA/source/isaaclab_tasks/isaaclab_tasks/core/reach/reach_env_cfg.py) |
| PPO 기본 설정 | [RSL-RL 설정](https://github.com/isaac-sim/IsaacLab/blob/v3.0.0-EA/source/isaaclab_tasks/isaaclab_tasks/core/reach/config/franka/agents/rsl_rl_ppo_cfg.py) |
| Sim의 ROS 배포판 지원 | [6.1 문서](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros.html) |
| Lyrical Isaac ROS | [5.0 설치](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html) |
| 기본 저장소 구성 참고 | [YOLOv8 Ubuntu 실습](https://github.com/SeongminJaden/yolov8-ubuntu24-practice) |

Isaac Lab은 BSD-3-Clause 라이선스의 외부 의존성입니다. 이 저장소는 upstream 파일을 복제하지 않고 설치된 클래스와 설정을 import하여 사용합니다. 공식 사진·영상의 출처와 원본 URL은 [MEDIA_SOURCES.md](MEDIA_SOURCES.md)에 기록했습니다. 외부 미디어의 권리는 원저작자에게 있으며 코드 라이선스에 포함하지 않습니다.
