# 실행 순서

각 코드 블록은 표시한 터미널에서 실행합니다. CPU 가상환경, Isaac Lab 호스트 환경, Isaac ROS 컨테이너를 합치지 않습니다.

## 1. URDF 점검 — CPU 가상환경

README의 설치를 마친 뒤 실행합니다.

```bash
python scripts/inspect_urdf.py assets/demo_arm.urdf --output reports/local/urdf.json
```

자기 파일은 `assets/my_arm.urdf`로 복사합니다. xacro는 ROS 환경에서 `xacro robot.urdf.xacro > robot.urdf`로 먼저 펼칩니다. package:// 메시 경로는 변환 환경에서도 해석 가능해야 합니다. 이 점검기는 트리·이름·한계를 검사합니다. 질량 관성의 물리적 정확성, 메시 경로와 충돌은 Sim에서 별도로 확인합니다.

### 로봇 설정과 교차 검사

```bash
python -m arm_practice.preflight assets/demo_arm.urdf configs/demo_robot.json \
  --output reports/local/preflight.json
```

독립 구동 관절의 누락·추가, TCP 링크 존재 여부, URDF보다 큰 위치·속도·힘 한계를 검사합니다. JSON의 관절 순서가 행동 순서이며 XML 선언 순서와 달라도 됩니다. 더 좁은 한계는 허용합니다. mimic 관절은 독립 행동에 포함하지 않습니다.

변환 후에는 같은 명령에 `--require-usd`를 추가합니다. 오류가 있으면 종료 코드 1과 JSON 오류 목록을 반환합니다. USD 파일 존재만 검사하며 내부 articulation·충돌·목표 도달 가능성은 검증하지 않습니다.

## 2. Lab 설치 — 새 호스트 터미널

[설치 가이드](INSTALL.md)의 GPU·uv 준비를 먼저 완료합니다.

```bash
cd ~/manipulator-isaac61-lyrical-practice  # 실제 clone 경로로 변경
export PRACTICE_DIR="$PWD"
export LAB_DIR="$HOME/IsaacLab-3.0-EA"
bash scripts/install_lab.sh
uv pip install --python "$LAB_DIR/.venv/bin/python" --no-deps -e "$PRACTICE_DIR"
bash scripts/lab.sh list_envs
```

성공 기준: `Practice-Reach-OwnArm` 태스크가 목록에 나타납니다. 태스크 목록 로딩은 USD 또는 물리 실행 검증을 대신하지 않습니다. 설치 후 `uv sync`를 다시 실행했다면 editable 패키지도 다시 설치합니다.

## 3. URDF → USD

NVIDIA 이용약관을 읽고 동의한 뒤 Isaac Sim을 시작합니다. 라이선스 확인창은 설치 가이드에 따릅니다.

```bash
bash scripts/convert_urdf.sh assets/demo_arm.urdf models/demo_arm.usd
```

`models/demo_arm.usd`가 생성됩니다. 같은 이름의 기존 USD는 덮어쓰지 않습니다. GUI 확인은 Lab 환경에서 다음과 같이 실행합니다.

```bash
cd "$LAB_DIR"
uv run --frozen --no-sync --extra isaacsim python scripts/tools/convert_urdf.py \
  "$PRACTICE_DIR/assets/demo_arm.urdf" "$PRACTICE_DIR/models/demo_arm_preview.usd" \
  --fix-base --joint-target-type position --viz kit
```

미리보기에서 실물 크기, 고정 베이스, 양의 관절 방향, `tool0`, 바닥과의 접촉, 중력 아래 자세를 확인합니다. 단위는 m·rad·kg입니다. 고정 관절을 병합하면 TCP 링크가 사라질 수 있어 이 예제는 병합하지 않습니다.

## 4. 자기 로봇으로 변경

```bash
cd "$PRACTICE_DIR"
cp configs/demo_robot.json configs/my_robot.json
export ARM_CONFIG="$PRACTICE_DIR/configs/my_robot.json"
```

JSON을 열어 다음을 모두 수정합니다. 파일 경로는 저장소 루트 기준 상대 경로 또는 절대 경로입니다.

| 필드 | 입력할 값 | 확인 기준 |
|---|---|---|
| usd_path | 변환한 USD | 파일이 실제 존재 |
| joint_names | 제어 관절 이름과 순서 | URDF·USD·컨트롤러 동일 |
| tcp_link | 도구의 실제 rigid body 이름 | USD의 body 목록에 존재 |
| home | 시작 관절값 | 충돌 없이 안정적으로 정지 |
| lower / upper | 관절 범위 | URDF·USD와 동일 |
| effort / velocity | 구동기 관절측 한계 | 감속비 반영, 사양과 일치 |
| stiffness / damping | 시뮬레이션 PD 계수 | 발산·진동 없이 유지 |
| action_scale | home 주변 동작 폭 | 단위 rad 또는 m |
| target_range | base 기준 목표 xyz | 실제 도달 가능한 작은 범위 |
| success_distance_m | 성공 거리 | 집기 작업 정밀도에 맞춤 |

3축 예제는 위치만 학습합니다. 모든 로봇이 임의의 6D 자세에 도달할 수 있는 것은 아닙니다. 그리퍼는 별도 행동·관측·성공 기준을 추가해야 합니다.

## 5. 짧은 학습 실행

첫 실행은 제공한 demo 설정으로 설치를 확인한 뒤 자기 설정으로 바꿉니다.

```bash
cd "$PRACTICE_DIR"
export ARM_CONFIG="$PRACTICE_DIR/configs/demo_robot.json"
bash scripts/lab.sh train --rl_library rsl_rl --task Practice-Reach-OwnArm \
  --num_envs 16 --max_iterations 10 physics=isaacsim_physx
```

성공 기준: 환경 16개가 생성되고 관측·행동 차원 오류 없이 iteration이 진행됩니다. 시작 자세가 튀거나 관절이 발산하면 학습을 늘리지 말고 USD·PD·관절 한계부터 수정합니다. 10회 학습은 실행 점검이며 정책 성능을 판단할 분량이 아닙니다.

```bash
export ARM_CONFIG="$PRACTICE_DIR/configs/my_robot.json"
bash scripts/lab.sh train --rl_library rsl_rl --task Practice-Reach-OwnArm \
  --num_envs 64 --max_iterations 500 physics=isaacsim_physx
```

VRAM 부족 시 환경 수부터 줄입니다. 로그는 wrapper가 이동한 `$LAB_DIR/logs` 아래에 저장됩니다. 터미널이 출력한 실제 run 경로와 체크포인트를 기록합니다.

## 6. 학습한 정책 재생

```bash
export CHECKPOINT=/absolute/path/to/model_500.pt  # 실제 생성된 파일명
test -f "$CHECKPOINT"
bash scripts/lab.sh play --rl_library rsl_rl --task Practice-Reach-OwnArm \
  --num_envs 1 --checkpoint "$CHECKPOINT" physics=isaacsim_physx
```

학습 때의 ARM_CONFIG를 유지합니다. 보상 숫자뿐 아니라 목표까지의 거리·성공률·충돌·관절 제한·반복 재현성을 기록합니다. 초기 예제에는 접촉 기반 충돌 감점과 연속 유지 시간 판정이 없습니다. 실제 집기 단계로 확장하기 전에 [코드 가이드](CODE_GUIDE.md)의 항목을 추가합니다.

## 7. 제어와 연결

MoveIt 구성은 [MOVEIT_CUMOTION.md](MOVEIT_CUMOTION.md), ROS 통신은 [LYRICAL_SIM_BRIDGE.md](LYRICAL_SIM_BRIDGE.md)를 진행합니다. 정책 출력을 모터에 바로 쓰지 않습니다. 관절 순서·단위·제한·명령 주기·정지 조건을 확인하고 하나의 제어 모드만 활성화합니다.
