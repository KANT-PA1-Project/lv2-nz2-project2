# 로봇팔 제어와 정책학습 — 슬라이드별 설명

## 01. 로봇팔 제어와 정책학습

직접 만든 URDF로 계획·학습·배포하기
Ubuntu 24.04 · Lyrical · Sim 6.1 · ROS 5.0 · Lab 3.0 EA

## 02. 오늘 만들 결과

내 로봇으로 목표 위치까지 이동하고, 같은 작업을 학습 정책으로 수행합니다.

계획 궤적, 학습 환경 코드, checkpoint, 평가표와 실패 영상을 남깁니다.
URDF를 불러오는 단계와 작업 성공을 구분합니다.

checkpoint(체크포인트): 저장된 학습 상태 · policy(폴리시): 행동을 정하는 함수

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 03. 실습 버전과 LTS 범위

제어는 Lyrical, 시뮬레이션은 Sim 6.1, 학습은 Lab 3.0 EA로 고정합니다.

Sim 6.1 대응 태그는 v3.0.0-EA입니다.
Lab 정식 v3.0.0은 Sim 6.2 대상입니다. EA를 LTS라고 부르지 않습니다.

LTS(엘티에스): 장기 지원 · EA(이이에이): 정식 출시 전 공개 버전

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 04. Lyrical 연결: 확인할 경계

Isaac ROS 5.0은 Lyrical을 사용합니다. Sim 6.1 브리지는 Humble·Jazzy를 공식 권장합니다.

Lyrical 기준을 유지하며 네이티브 배포판 로드의 실험적 경로를 검증합니다.
관절·TF·시계·Action이 통과하기 전에는 통합 성공으로 표시하지 않습니다.

bridge(브리지): 서로 다른 실행 환경 연결 · ABI(에이비아이): 바이너리 호출 규약

[공식 자료](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros.html)

## 05. 세 역할: 계획 · 제어 · 학습

계획기는 경로를 만들고 제어기는 상태를 보며 명령합니다. 정책은 관측에서 행동을 고릅니다.

계획 궤적 추종과 정책 추론은 서로 다른 실행 모드입니다.
같은 관절에 여러 노드가 동시에 명령하지 않도록 선택기를 둡니다.

planner(플래너): 경로계획기 · controller(컨트롤러): 제어기

[공식 자료](https://control.ros.org/lyrical/doc/ros2_control/doc/index.html)

## 06. 전체 구조: 인식에서 모터까지

물체를 찾은 뒤 로봇 기준 좌표로 바꾸고, 도달 방법을 결정합니다.

YOLO의 2D 상자를 바로 팔의 3D 목표로 쓰지 않습니다.
자세 추정·깊이·TF·관절 피드백의 시간과 좌표를 맞춥니다.

TF(티에프): 좌표계 간 변환 · TCP(티씨피): 도구 기준점

[공식 자료](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/manipulation/cumotion_moveit/tutorial_custom_manipulator.html)

## 07. URDF · SRDF · XRDF · USD

파일마다 맡은 역할이 다릅니다. 로봇 모양 하나로 계획·학습·실기 설정이 모두 완성되지는 않습니다.

URDF: 기구 / SRDF: 계획 그룹 / XRDF: cuMotion용 구성
USD: 물리·센서·장면. 관절 이름과 TCP는 모두 일치시킵니다.

URDF(유알디에프) · SRDF(에스알디에프) · XRDF(엑스알디에프) · USD(유에스디)

[공식 자료](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/manipulation/cumotion_moveit/tutorial_custom_manipulator.html)

## 08. 순기구학: 관절에서 도구 위치로

관절값 q가 주어지면 링크 변환을 연결해 TCP의 위치와 방향을 구합니다.

T_base,TCP(q) = T₁(q₁) · T₂(q₂) · … · T_n(q_n)
관절 하나씩 양의 방향으로 움직여 URDF 축·원점을 확인합니다.

q(큐): 관절값 · T(티): 좌표 변환 행렬 · n(엔): 관절 수 · FK(에프케이): 순기구학

[공식 자료](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

## 09. 역기구학: 목표를 만드는 관절값

같은 TCP 위치에도 여러 관절 해가 있을 수 있습니다. 작업 공간 밖에는 해가 없을 수 있습니다.

관절 한계·충돌·현재 자세와의 거리를 함께 고려합니다.
자유도가 부족한 팔에 임의의 6D 자세를 요구하지 않습니다.

IK(아이케이): 역기구학 · DoF(디오에프): 자유도

[공식 자료](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

## 10. 공식 화면: 내 URDF 불러오기

링크가 실제 크기와 방향으로 보이는지 확인합니다. 화면의 Panda는 설명용 예제이며 실습은 자기 로봇으로 바꿉니다.

mesh(메시): 표면 형상 · collision(콜리전): 충돌용 형상

[공식 자료](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

## 11. 공식 화면: 계획 그룹 지정

arm에 포함할 관절 체인을 고릅니다. 집게 관절을 팔 체인에 잘못 섞지 않습니다.

planning group(플래닝 그룹): 함께 계획할 관절 집합

[공식 자료](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

## 12. 공식 화면: TCP와 집게 연결

손목 링크와 물체를 잡는 기준점은 다를 수 있습니다. 목표 좌표가 어느 링크에 적용되는지 확인합니다.

end effector(엔드 이펙터): 집게·도구 · TCP: 도구 기준점

[공식 자료](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

## 13. 충돌 검사와 장애물 회피

시각화 mesh와 충돌 형상은 다릅니다. 계획기는 충돌 모델로 경로의 가능 여부를 판단합니다.

팔·집게·잡은 물체를 포함합니다.
충돌쌍을 모두 제외하면 경로는 쉽게 나오지만 실제로 통과할 수 없습니다.

self-collision(셀프 콜리전): 로봇 자체 링크 사이의 충돌

[공식 자료](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/manipulation/cumotion_moveit/tutorial_custom_manipulator.html)

## 14. 공식 화면: 충돌 제외 목록 검토

자동 생성 결과를 그대로 신뢰하지 말고 실제 관절 범위에서 확인합니다. 제외한 이유를 기록합니다.

collision matrix(콜리전 매트릭스): 링크 쌍의 충돌 검사 설정

[공식 자료](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

## 15. 공식 화면: XRDF의 충돌 구

구가 링크와 집게를 충분히 덮는지 봅니다. 너무 작은 구는 충돌을 놓치고 너무 큰 구는 통로를 막습니다.

sphere(스피어): 구 · XRDF: 가속 계획기에 필요한 로봇 구성

[공식 자료](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/manipulators/manipulators_robot_description_editor.html)

## 16. cuMotion 경로계획 구조

GPU를 사용해 로봇 모델·목표·주변 장애물을 고려한 궤적을 계산합니다.

MoveIt 플러그인과 cuMotion 서버, 내 URDF/XRDF가 함께 필요합니다.
실제 모터 통신은 ros2_control 또는 별도 드라이버가 담당합니다.

GPU(지피유): 병렬 계산 장치 · trajectory(트라젝터리): 시간 정보가 있는 경로

[공식 자료](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/manipulation/cumotion_moveit/tutorial_custom_manipulator.html)

## 17. 공식 시연: cuMotion 경로계획

목표가 바뀔 때 경로가 어떻게 달라지는지 관찰합니다. 목표 변경 전후의 경로를 비교해 봅니다.

motion planning(모션 플래닝): 충돌과 한계를 고려한 운동 계획

[공식 자료](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_cumotion/index.html)

## 18. cuRoboV2와 cuMotion의 구분

cuRobo는 GPU 운동 생성 연구 도구입니다. 현재 cuMotion과 구현을 구분합니다.

직접 최적화 비용·동역학을 연구할 때 cuRobo를 비교합니다.
기본 실습은 Lyrical의 cuMotion 플러그인 경로를 유지합니다.

optimization(옵티마이제이션): 목적값을 개선하는 계산

[공식 자료](https://github.com/NVlabs/curobo/discussions/616)

## 19. 연구 시연: 움직이는 장애물

장애물이 바뀌면 계획을 언제 다시 계산하는지 봅니다. 장애물 이동 속도와 반응 지연을 함께 봅니다.

replanning(리플래닝): 달라진 상태로 계획 다시 계산

[공식 자료](https://curobo.org/)

## 20. 연구 시연: 실제 팔의 경로 실행

계획 계산과 궤적 추종을 구분합니다. 영상의 계산 시간과 전체 시스템 지연은 다릅니다.

latency(레이턴시): 입력부터 결과까지의 지연

[공식 자료](https://curobo.org/)

## 21. ros2_control: 명령과 실제 상태

컨트롤러는 하드웨어 인터페이스를 통해 목표를 쓰고 관절 상태를 읽습니다.

command interface는 position/velocity/effort 중 실제 구동기가 지원하는 방식으로 정합니다.
명령값을 실제 측정값처럼 되돌려 보내지 않습니다.

interface(인터페이스): 연결 규약 · feedback(피드백): 실제 상태의 되먹임

[공식 자료](https://control.ros.org/lyrical/doc/ros2_control/doc/index.html)

## 22. 공식 화면: 제어 인터페이스 설정

명령·상태 인터페이스를 확인합니다. 실제 통신에는 구동기 드라이버도 필요합니다.

state interface(스테이트 인터페이스): 관절 상태를 읽는 통로

[공식 자료](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

## 23. 공식 화면: 컨트롤러와 관절 연결

컨트롤러 이름·관절 목록·MoveIt Action 경로를 맞춥니다. 계획 성공과 실행 성공을 각각 확인합니다.

Action(액션): 진행 상태와 결과를 주고받는 ROS 작업 인터페이스

[공식 자료](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

## 24. MoveIt Servo: 마지막 위치 보정

카메라로 본 오차를 작은 속도·자세 명령으로 바꿔 목표를 따라갑니다.

Servo는 긴 장애물 우회 계획을 대신하지 않습니다.
전역 이동은 계획기로, 마지막 보정은 Servo로 나누어 비교합니다.

servo(서보): 피드백을 이용한 반복 보정 · Twist(트위스트): 선속도·각속도

[공식 자료](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)

## 25. 접촉 작업의 힘 제어

임피던스는 위치 오차에 따른 힘 관계를, 어드미턴스는 측정된 힘에 따른 운동 관계를 설계합니다.

모터 전류만으로 손목의 외력을 정확히 알 수 있다고 가정하지 않습니다.
센서·구동기 기능·갱신 주기와 안정성을 먼저 확인합니다.

impedance(임피던스) · admittance(어드미턴스) · F/T: 힘·토크

[공식 자료](https://control.ros.org/lyrical/doc/ros2_controllers/admittance_controller/doc/userdoc.html)

## 26. 작업 구조: 접근 → 집기 → 놓기

하나의 목표 자세만으로 전체 집기 작업을 설명할 수 없습니다. 단계마다 다른 조건과 동작이 필요합니다.

접근 전 장애물 확인 → 집게 닫힘 확인 → 물체 유지 확인 → 놓은 뒤 손 떼기.
실패하면 재탐색·재접근·종료 중 어느 단계로 갈지 정합니다.

MTC(엠티씨): MoveIt Task Constructor · state machine(스테이트 머신): 상태 전이 구조

[공식 자료](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)

## 27. nvblox와 잡은 물체 반영

깊이 영상으로 주변 형상을 갱신합니다. 로봇 자신과 잡은 물체를 어떻게 처리할지도 정해야 합니다.

카메라 보정·TF·시간을 맞춘 뒤 ESDF를 연결합니다.
서비스가 없는 초기 cuMotion 실습에서는 read_esdf_world=false입니다.

ESDF(이에스디에프): 장애물까지의 부호 있는 거리장

[공식 자료](https://nvidia-isaac-ros.github.io/v/release-5.0/concepts/scene_reconstruction/nvblox/index.html)

## 28. 설치 1: Lyrical 환경 확인

호스트의 Isaac ROS CLI로 컨테이너를 열고 배포판과 GPU부터 확인합니다.

isaac-ros activate
printenv ROS_DISTRO
ros2 pkg prefix isaac_ros_cumotion
저장소 docs/INSTALL.md 5절을 순서대로 실행합니다.

container(컨테이너): 의존성을 분리한 실행 환경

[공식 자료](https://nvidia-isaac-ros.github.io/v/release-5.0/getting_started/index.html)

## 29. 설치 2: 가상 제어로 먼저 확인

Setup Assistant로 자기 로봇 설정을 만들고 OMPL 계획을 먼저 시험합니다.

ros2 launch moveit_setup_assistant setup_assistant.launch.py
패키지 생성·빌드 후 demo.launch.py 실행
그다음 cuMotion 서버와 planning YAML을 추가합니다.

OMPL(오엠피엘): 샘플링 기반 운동계획 라이브러리

[공식 자료](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

## 30. 설치 3: Lab 환경은 별도로

Isaac ROS 컨테이너와 다른 호스트 폴더에서 EA 태그를 설치합니다.

bash scripts/install_lab.sh
Sim 6.1.0.0과 v3.0.0-EA를 확인합니다.
패키지 등록까지 docs/PRACTICE_GUIDE.md를 따라갑니다.

lock(록): 설치할 의존성 버전 기록 · venv(브이엔브이): Python 가상환경

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 31. ROS 브리지 통과 기준

같은 네트워크에 있다는 사실만으로 통신 검증이 끝나지 않습니다.

/clock → /joint_states → TF → 명령 → Action 결과 순서로 검사합니다.
Lyrical의 네이티브 연결은 실험적 경로라는 조건을 기록합니다.

QoS(큐오에스): 메시지 전달 정책 · RMW(알엠더블유): ROS 통신 구현

[공식 자료](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_ros.html)

## 32. 정책학습: 관측에서 행동으로

정책 π는 관측 o를 받아 행동 a를 정하는 함수입니다. 신경망의 가중치를 학습해 이 함수를 바꿉니다.

a_t ~ πθ(a | o_t)
관절 목표와 모터 토크는 서로 다른 행동입니다. 단위·범위·주기를 먼저 정의합니다.

π(파이): 정책 · θ(세타): 가중치 · o(오): 관측 · a(에이): 행동

[공식 자료](https://arxiv.org/abs/1707.06347)

## 33. 강화학습의 반복 구조

행동을 실행하면 다음 관측과 보상을 얻습니다. 이 경험을 모아 정책을 개선합니다.

목표는 한 순간의 보상보다 에피소드 전체의 누적 성과입니다.
로봇 모델·센서·행동·보상·종료 조건이 환경을 구성합니다.

RL(알엘): 강화학습 · episode(에피소드): 한 번의 시도

[공식 자료](https://arxiv.org/abs/1707.06347)

## 34. PPO와 RSL-RL은 다릅니다

PPO는 정책을 업데이트하는 알고리즘이고 RSL-RL은 학습을 실행하는 라이브러리입니다.

rollout을 모은 뒤 actor와 critic을 학습합니다.
RSL-RL 철자를 RCL-RL과 혼동하지 않습니다.

PPO(피피오): Proximal Policy Optimization · RSL-RL(알에스엘 알엘)

[공식 자료](https://github.com/leggedrobotics/rsl_rl)

## 35. PPO의 원리: 큰 변경을 제한

새 정책이 좋은 행동의 확률을 높이되, 한 번에 너무 멀리 변하지 않도록 제한합니다.

Lclip = E[min(r_tÂ_t, clip(r_t, 1−ε, 1+ε)Â_t)]
r_t = πθ(a_t|o_t) / πold(a_t|o_t). 보상 개선과 실제 성공률은 함께 봅니다.

r(알): 확률비 · Â(에이 햇): 이점 · ε(엡실론): 제한 폭 · E(이): 평균
π(파이): 정책 · θ(세타): 가중치 · t(티): 시점 · clip(클립): 범위 제한

[공식 자료](https://arxiv.org/abs/1707.06347)

## 36. Actor–Critic: 행동과 가치

Actor는 행동을 고르고 Critic은 현재 상태에서 앞으로 얻을 성과를 추정합니다.

시뮬레이터 정답을 Critic에만 줄 수는 있지만, 배포할 Actor의 입력은 실기에서 얻을 수 있어야 합니다.

actor(액터): 행동 정책 · critic(크리틱): 가치 추정기 · V(브이): 가치

[공식 자료](https://arxiv.org/abs/1707.06347)

## 37. 관측 함수의 입력 구성

관절 위치·속도, TCP 오차, 집게 상태, 이전 행동처럼 작업에 필요한 값을 넣습니다.

배열 이름·순서·단위·좌표계를 문서화합니다.
카메라 물체 자세를 world 기준으로 학습했다면 실기도 같은 기준으로 변환합니다.

observation(옵저베이션): 관측 · normalization(노멀라이제이션): 크기 조정

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 38. 행동 함수: 숫자를 모터 목표로

출력의 의미를 관절 위치·증분·속도·토크 중 하나로 정합니다.

q_target = clip(q_reference + scale × a, q_min, q_max)
q_reference가 현재 관절인지 고정 기준인지 반드시 명시합니다.

q(큐): 관절값 · reference(레퍼런스): 기준값 · min/max(민/맥스): 하한/상한
a(에이): 행동 · scale(스케일): 배율 · clip(클립): 범위 제한

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 39. 보상 함수: 작업 성공을 수치로

목표 접근, 작업 성공, 충돌과 불필요한 움직임을 각각 항목으로 나눕니다.

r = −w_d d − w_a||Δa||² − w_c c + w_s s
거리만 줄이고 물체를 놓치는 편법이 생기지 않도록 성공 판정을 별도로 둡니다.

r(알): 보상 · d(디): 거리 · Δa(델타 에이): 행동 변화 · c(씨): 충돌 · s(에스): 성공 · w(더블유): 가중치

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 40. 리셋과 종료를 따로 설계

리셋은 다음 시도의 초기조건을 만듭니다. 종료는 이번 시도를 끝낼 이유를 판단합니다.

물체가 겹치거나 도달 불가능한 곳에 생성되지 않게 합니다.
시간 제한과 낙하·충돌 같은 실패 종료를 구분해 기록합니다.

reset(리셋): 초기화 · terminated: 상태에 의한 종료 · truncated: 시간 등 제한

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 41. 공식 화면: 로봇팔 도달 정책 학습

같은 작업을 여러 환경에서 병렬 실행하는 모습을 봅니다. 목표 분포·로봇 모델·실기 입력 규격은 자기 로봇에 맞춰야 합니다.

parallel environments(패럴렐 인바이런먼츠): 병렬 시뮬레이션 환경

[공식 자료](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/policy_deployment/04_reach/reach_policy.html)

## 42. 내 로봇 Task 만들기

공식 생성기로 프로젝트를 만든 뒤 로봇과 작업의 설정·함수를 채웁니다.

robot_cfg / env_cfg / observations / rewards / terminations
관절 수를 6개로 고정하지 말고 실제 이름과 제어 가능한 자유도를 사용합니다.

task(태스크): 수행할 작업과 환경 · cfg(씨에프지): 설정

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 43. 학습 실행 순서

설치 확인 예제와 자기 로봇 실습을 분리합니다. 먼저 작은 환경에서 동작을 확인합니다.

무행동 → 작은 임의 행동 → 짧은 PPO 학습 → checkpoint 재생.
처음부터 수천 개 환경을 켜서 원인을 숨기지 않습니다.

zero agent(제로 에이전트): 0 행동 실행 · rollout(롤아웃): 수집한 경험

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 44. 학습 명령과 재생 명령

3.0 EA는 통합 isaaclab train/play 명령을 사용합니다. 물리 백엔드도 명시합니다.

bash scripts/lab.sh train --task Practice-Reach-OwnArm
physics=isaacsim_physx
전체 인자·ARM_CONFIG·재생 순서는 PRACTICE_GUIDE.md에 있습니다.

backend(백엔드): 실제 계산을 담당하는 구현

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 45. 평가: 학습 보상만 보지 않기

학습에 사용하지 않은 초기조건에서 성공률과 실패 유형을 측정합니다.

성공 횟수/총 시도, 위치 오차, 충돌, 완료 시간, 추론 지연을 기록합니다.
계획기·PPO·모방학습을 같은 목표와 조건으로 비교합니다.

holdout(홀드아웃): 학습에 쓰지 않은 평가 조건

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 46. 현실로 옮기기: 차이를 줄이는 방법

질량·마찰·지연·센서 잡음이 시뮬레이션과 현실에서 다릅니다. 측정한 범위를 학습에 반영합니다.

먼저 단일 관절·접촉 응답으로 모델을 맞춥니다.
그다음 현실적인 범위에서 변동시키고 실제 실패 원인과 비교합니다.

sim-to-real(심투리얼): 시뮬레이션에서 실기로 이전 · domain randomization: 환경 변동 학습

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 47. 모방학습: 사람이 보여준 행동

시연의 관측과 행동을 정답 쌍으로 사용합니다. 실패 구간과 복구 시연의 처리 기준도 정합니다.

카메라 프레임과 명령의 시각을 맞춥니다.
에피소드·촬영 세션을 단위로 train/validation/test를 나눕니다.

IL(아이엘): 모방학습 · demonstration(데먼스트레이션): 시연

[공식 자료](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/features/imitation-learning/teleop_imitation.html)

## 48. 공식 화면: Mimic 작업 예시

쌓기 작업의 물체·로봇·작업대를 확인합니다. 물체의 초기 배치를 바꾸어 시연을 확장합니다.

Mimic(미믹): 시연을 새 상황으로 옮겨 데이터 생성

[공식 자료](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/features/imitation-learning/teleop_imitation.html)

## 49. 공식 화면: 시연을 수집하는 장면

시연의 시작 상태와 목표 상태를 확인합니다. 최종 실습에서는 자기 로봇의 도달 범위와 집게 기능을 반영합니다.

teleoperation(텔레오퍼레이션): 사람이 원격으로 조작

[공식 자료](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/features/imitation-learning/teleop_imitation.html)

## 50. 공식 화면: 다른 배치로 시연 확장

물체 기준 좌표를 이용해 동작을 옮깁니다. 변환된 궤적이 충돌하거나 실패하면 성공 데이터에서 제외합니다.

data generation(데이터 제너레이션): 학습 데이터 생성

[공식 자료](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/features/imitation-learning/teleop_imitation.html)

## 51. 공식 도식: Mimic 환경 구성

작업 구간, 물체 프레임, 성공 조건이 어디에 들어가는지 확인합니다. 자기 URDF만 넣어 끝나는 과정이 아닙니다.

subtask(서브태스크): 접근·집기처럼 나눈 작업 단위

[공식 자료](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/features/imitation-learning/teleop_imitation.html)

## 52. 공식 시연: Mimic으로 학습한 쌓기

물체를 잡는 순간과 놓는 순간을 구분해 봅니다. 자체 실험에서는 놓친 물체·복구 실패도 함께 기록합니다.

BC-RNN(비씨 알엔엔): 순환 신경망을 사용한 행동 복제

[공식 자료](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/features/imitation-learning/teleop_imitation.html)

## 53. 공식 시연: 구간 사이 보간

작업 구간을 이어 붙일 때 중간 움직임을 봅니다. 보간 횟수는 충돌·시간·움직임의 연속성에 영향을 줍니다.

interpolation(인터폴레이션): 두 상태 사이 중간값 생성

[공식 자료](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/features/imitation-learning/teleop_imitation.html)

## 54. BC · ACT · Diffusion Policy

BC는 시연을 학습합니다. ACT는 행동 묶음을 예측하고, Diffusion Policy는 잡음에서 행동을 복원합니다.

같은 데이터라도 입력·출력·주기·평가를 맞춘 후 비교합니다.
ACT와 Diffusion을 설치 옵션만 다른 동일 알고리즘으로 설명하지 않습니다.

BC(비씨): 행동 복제 · ACT(에이씨티): Action Chunking with Transformers

[공식 자료](https://huggingface.co/docs/lerobot/index)

## 55. Diffusion Policy의 원리

학습 시 정답 행동에 넣은 잡음을 예측하도록 학습하고, 실행 시 잡음에서 행동 시퀀스를 복원합니다.

여러 단계 추론의 지연과 행동 실행 길이를 함께 측정합니다.
미래 동작을 길게 실행하면 최신 관측 반영이 늦어질 수 있습니다.

diffusion(디퓨전): 확산 · denoising(디노이징): 잡음 제거

[공식 자료](https://diffusion-policy.cs.columbia.edu/)

## 56. 공식 자료: GR00T 소개

서로 다른 로봇의 관측·행동을 연결하려는 접근입니다. 새 로봇에는 센서·행동 설정과 자체 데이터가 필요합니다.

embodiment(엠바디먼트): 로봇의 몸체·센서·구동 구조

[공식 자료](https://github.com/NVIDIA/Isaac-GR00T)

## 57. 공식 도식: GR00T 모델 구조

영상·언어·상태가 어디로 들어가고 행동이 어떻게 나오는지 읽습니다. 내 로봇의 행동 표현을 명시해야 합니다.

VLA(브이엘에이): Vision-Language-Action · modality(모달리티): 정보 종류

[공식 자료](https://github.com/NVIDIA/Isaac-GR00T)

## 58. 공식 시연: 언어 조건 로봇 조작

물체가 움직이거나 가려질 때 팔의 반응을 봅니다. 카메라 위치와 작업 지시도 함께 확인합니다.

fine-tuning(파인튜닝): 사전학습 모델을 추가 데이터로 학습

[공식 자료](https://github.com/NVIDIA/Isaac-GR00T)

## 59. 내 로봇으로 VLA 학습하기

영상·관절 상태·행동·언어 지시를 같은 시각 기준으로 저장하고 내 로봇의 형식을 등록합니다.

GR00T는 별도 venv에 고정 revision으로 설치합니다.
modality key, 단위, 상대/절대 행동, 집게 범위, 주기를 맞춥니다.

relative action(렐러티브 액션): 기준 대비 증분 · absolute action: 절대 목표

[공식 자료](https://github.com/NVIDIA/Isaac-GR00T)

## 60. VLA + 강화학습은 확장 연구

시연 기반 모델을 보상으로 추가 개선하는 방법입니다. 기본 PPO와 설치·학습 비용이 다릅니다.

RLinf 예제의 GR00T·transformers 고정 버전을 따로 유지합니다.
최신 N1.7 환경에 기존 N1.5 기반 예제를 합치지 않습니다.

post-training(포스트 트레이닝): 사전학습 이후의 추가 학습

[공식 자료](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/experimental-features/rlinf_vla_posttraining.html)

## 61. 배포 구조: 정책과 명령 선택

정책을 내보낸 뒤 관측 구성과 행동 변환을 실기 쪽에 동일하게 구현합니다.

상태가 오래되거나 NaN이면 유지·정지로 전환합니다.
계획기·Servo·학습 정책 중 하나만 제어권을 갖습니다.

watchdog(워치독): 상태 갱신 감시 · inference(인퍼런스): 모델 추론

[공식 자료](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_deploy/index.html)

## 62. 실습 결과 비교표

같은 초기조건에서 여러 방법의 성공률과 지연을 비교하면 선택 이유를 설명할 수 있습니다.

방법 / 데이터 수 / 성공률 / 충돌 수 / 완료 시간 / 추론 지연
기록은 실제 측정값으로 채웁니다. 공개 영상의 숫자를 옮기지 않습니다.

baseline(베이스라인): 비교 기준 · closed-loop(클로즈드 루프): 피드백 포함 실행

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 63. 문제 진단 순서

입력·좌표·관절·제어·학습을 한꺼번에 바꾸면 원인을 찾기 어렵습니다.

모델 형상 → 한 관절 → 상태 수신 → 작은 명령 → 계획 → 학습 순으로 확인합니다.
학습 전에 실패한 항목을 더 긴 학습으로 해결하려 하지 않습니다.

debugging(디버깅): 오류 원인 확인과 수정

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)

## 64. 완료 기준과 제출물

다른 사람이 내 환경을 재현하고, 실행 방법과 성공·실패 원인을 확인할 수 있어야 합니다.

URDF/USD/SRDF/XRDF, 제어·학습 코드, 버전 기록, checkpoint, 평가표, 실행 영상.
실기 미검증 항목은 그대로 표시합니다. 설치 명령과 세부 기준은 README를 사용합니다.

reproducibility(리프로듀서빌리티): 같은 조건으로 결과를 다시 확인하는 성질

[공식 자료](https://github.com/isaac-sim/IsaacLab/tree/v3.0.0-EA)
