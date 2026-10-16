# ros2_ws

ROS 2 colcon 워크스페이스다. 패키지는 `src/` 아래에 패키지별 폴더로 둔다.

- 빌드 방법은 루트 [README](../README.md)의 "다른 기기에서 실행하기"를 따른다.
- 빌드 산출물(`build/`, `install/`, `log/`)은 git에 올리지 않는다.
- 소스로 받아야 하는 외부 저장소는 이 폴더의 `.repos` 파일에 커밋 해시까지 고정하고, `vcs import src < <파일>.repos`로 받는다.
