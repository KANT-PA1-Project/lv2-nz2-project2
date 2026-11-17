# v136 runtime 인터페이스와 Isaac 연결

`runtime_owner.py`는 공유 `ObservationOwner`·`NativeMissionCapture`·issuer와 v136 브리지/게시 준비를 조립한다. `isaac_runtime_adapter.py`는 기존 world·robot·controller·실제 센서를 연결한다. import만으로 자식 프로세스를 시작하지 않으며 factory/연결 생성부터 시작한다.

최신 실측 결과와 남은 검증은 [ISAAC_VALIDATION.md](ISAAC_VALIDATION.md)에 있다. 실제 검증 구성은 physics/rendering_dt 모두 1/960, pre-session encoder 960step, 센서 사전 render 2회다. 공유 부모 원본 자체에는 이 센서 매핑과 helper 호출이 없으므로 그대로 실행할 수 없다. `prepare_isaac_validation.py`가 v136 출력 폴더에 연결 구성을 생성한다.

## 신원과 도메인

이 PC(pa33)는 `runtime.local.json`과 `runtime.example.json` 모두 domain33을 사용한다. 기존 범용 테스트의 domain87은 설정 전달 여부를 확인하는 가짜 환경 값이다.

factory는 주어진 번들의 모든 파일 해시와 현재 로드된 adapter 경로를 확인하고, `native_entrypoint_manifest.json`의 SHA-256을 한 번 계산한다. 같은 값을 아래 네 곳에 연결한다.

- `RendererEpochIssuer.source`
- `LivePublicationBridge.source_sha`
- `PriorityFeedbackBridge.source`
- `RuntimePublicationPrepare.source_sha`

따라서 첫 current feedback에서 조립 manifest 해시와 별도 부모 source를 혼용하지 않는다. 모델 신원은 별도로 manifest의 pin 목록을 canonical JSON으로 해시하여 issuer의 model에 넣으며 실제 가중치 해시 검증도 유지한다. 이 source 값이 보증하는 파일 범위는 조립 manifest의 목록이다. 외부 호출자 callback 구현의 소스 기록은 실제 통합 실험에서 별도로 보존해야 한다.

## 호출자가 제공할 RuntimeCallbacks

| 필드 | 호출자가 전달할 값 |
|---|---|
| `rgb`, `depth` | 현재 runtime의 annotator dict. 키는 카메라 스키마와 일치. 각 값은 `get_data`를 제공. |
| `rgb_reference`, `depth_reference` | 같은 센서들의 native reference metadata annotator dict. |
| `read_epoch()` | 현재 `(physics_step, sim_time_s)` 튜플. physics960Hz 기준. |
| `read_joint_positions()` | 현재 native 관절 위치 벡터. 과거 capture q로 대체하지 않음. |
| `arm_indices` | 위 벡터에서 arm_joint1~5에 해당하는 서로 다른 실제 index 5개. |
| `render()` | 소유 스레드의 render callback. 캡처 중 physics를 진행하지 않음. |
| `fk(measured_joints)` | 측정 관절값으로 계산한 카메라별 FK 행렬. 기존 센서 보정/변환 사용. |
| `refresh_packet(sample)` | 촬영 이후 실제 물리·encoder·IMU를 갱신한 NativePacketTimeline packet. 원래 시각과 각 native sample을 보존. |
| `clock()` | `time.monotonic_ns`와 같은 단조 정수 ns clock. 기본값은 실제 monotonic_ns. |

factory 인자는 `root`(새 조립 번들 경로), `out`(실험 출력 경로), `run_id`, `clock_epoch`, `callbacks=RuntimeCallbacks(...)`, `settings=RuntimeSettings.from_env()`다. 반환값은 `ObservationSession`이며 `session.owner`가 공유 코어의 ObservationOwner다. 설정 경로는 `R12_RUNTIME_CONFIG` 또는 명시적 settings로 제공한다. factory는 scene, 퍽 위치, 지지대, 정답 좌표를 만들거나 읽지 않는다. 필수 callback을 비우면 실패한다.

실제 호출자는 매 physics step 뒤 `session.record_physics_step(native_qd=..., same_step_fd=...)`에 동일 step의 두 속도를 전달하고, 관측 시 `session.observe(frame)`를 호출한다. 코어의 0.5초 연속 정착 기준을 유지하며 sparse한 기록이나 가짜0속도를 채워 넣지 않는다. 960Hz 기록 경로에는 manifest 파일 재해시를 추가하지 않았다. manifest 변경 검사는 관측 전 수행한다.

## 상태와 정리

같은 소유 스레드에서 구성·step 기록·관측·종료한다. 구성 도중 epoch가 움직이면 반환된 브리지들을 역순으로 닫고 실패한다. source가 바뀌면 새 관측/feedback을 거부한다. 코어 ObservationOwner의 통신 실패 상태와 pending 원본은 유지하여 다음 capture를 막는다. session.close도 원본을 비우지 않으며, 반환된 child의 비정상 종료 코드와 close 예외를 보고한다.

실제 정상 경로에서 자식 3개의 exit 0을 확인했다. child constructor 도중 실패의 전체 자원 정리 검증과 CPU mock 정리 검사는 별도로 구분한다.


## 제공한 코드와 경계

- `overlay/arm_practice/native_rgbd/isaac_runtime_adapter.py`:
  `IsaacRuntimeAdapter`와 `connect_sensor_runtime`.
- `runtime_owner.py`의 `ObservationSession.record_postcapture_step`:
  현재 owner가 소유한 capture의 촬영 후 16-step 구간에서만 encoder 기록 허용.
- 어댑터는 기존 world/robot/controller/annotator/native IMU를 받는다.
  scene·fixture·정답 좌표·관절 목표·새 Isaac application을 생성하지 않는다.
- 모듈 import는 프로세스를 시작하지 않는다. 어댑터를 **생성하는 호출**부터
  실제 child가 시작된다. 아래 예시는 부모 runtime의 호출 위치를 설명한다.

## 기존 부모 runtime에서의 연결 위치

부모 센서 초기화가 완료되고 원래 `apply_controls_and_step` 함수가 정의된 뒤,
물리/관측 루프를 시작하기 전에 연결한다. 초기화와 호출은 같은 소유 스레드에서
수행하고 child 준비 중에는 physics를 진행하지 않는다.

`connect_sensor_runtime`이 받는 namespace의 실제 항목:

| 이름 | 실제 객체 |
|---|---|
| world, robot | 초기화된 Isaac world와 articulation |
| names | 전체 articulation DOF 이름을 순서대로 담은 목록 |
| prepared_camera_rig | 측정 관절값을 받는 transforms 메서드 제공 |
| native_rgb, native_depth | 스키마와 정확히 같은 키의 8RGB/4depth annotator |
| native_rgb_refs, native_depth_refs | 각 render product에 attach된 ReferenceTime annotator |
| imus | front, rear 순서의 서로 다른 native IMU 두 개 |
| apply_controls_and_step | 기존 제어를 유지하면서 physics를 정확히 1회 진행하는 함수 |
| capture_run_id | 실제 run의 식별자 |

이 항목들은 읽어 본 v135 부모의 변수명이다. helper가 카메라나 annotator를
자동으로 만들지는 않는다. 현재 통합 부모의 객체 이름이 다르면 실제 객체를
해당 키에 명시적으로 매핑한다. 참조용 v135 파일 전체를 복사해 실행하거나,
기존 fixture 전용 producer를 함께 실행하지 않는다.

## 부모 runtime의 호출 예시

새로 조립한 번들의 `src`를 부모가 `arm_practice`를 처음 import하기 전에
Python 경로 맨 앞에 둔다. 과거 번들의 모듈이 이미 import된 프로세스와 섞지 않는다.
부모의 scene/assets 경로와 어댑터의 bundle root는 서로 다른 경로다.

```python
# Isaac 부모의 센서/제어 초기화 이후, 같은 스레드.
from arm_practice.native_rgbd.isaac_runtime_adapter import connect_sensor_runtime

connection = connect_sensor_runtime(
    globals(),
    root=bundle_root,              # 새 조립 결과; 과거 .runtime 번들 재사용 금지
    out=new_run_output,            # 존재하지 않는 새 출력 폴더
    parent_source=__file__,        # 실제 부모 소스 원본의 hash와 사본 보존
    clock_epoch=run_clock_epoch,   # 이 run에서 정한 실제 clock epoch 식별자
    settings=runtime_settings,
)
try:
    # helper는 globals()['apply_controls_and_step']을 connection.step으로 연결한다.
    # 이후 부모의 모든 physics step은 이 경로를 사용한다.
    # 원래 stepper/controller는 내부에서 정확히 한 번 실행된다.
    for sub in range(960):
        apply_controls_and_step(sub)

    for frame in range(requested_observations):
        for sub in range(48):
            apply_controls_and_step(sub)
        result = connection.observe(frame)
        # result['captured']와 실제 ROS/decoder 결과를 상위 runtime에서 확인.
finally:
    connection.close()             # world/SimulationApp을 닫기 전에 수행
```

위 960/48은 연결 위치를 보이는 제한된 관측 예시다. 960Hz의 첫 1초 기록 후
관측 간 48step을 진행하며, 관측 성공을 보장하는 값은 아니다. 정착은 실제
native 속도와 동일 step FD가 모두 0.02rad/s 미만인 0.5초 연속 이력으로 판정한다.
임의 목표 자세나 가짜 0속도를 넣지 않는다. 부모가 physics를 이미 진행하는
루프를 갖고 있다면 같은 step을 이 예시로 다시 진행하지 않는다.

기존 `native_producer`/인지 `worker`와의 이중 실행은 helper가 거부한다.
부모의 `native_producer_enabled`와 `perception_enabled` 경로도 꺼져 있어야 한다.
기존 observer 경로·직접 issuer 기록·별도 current feedback 송신은 함께
호출하지 않는다. v136 검증 구성은 공유 부모에서 파생한 초기화/제어 사본에 이 연결을 적용했다.

## 실제 데이터 경로

1. `step(sub)`은 원래 controller/stepper 앞뒤에서 현재 q/qd와 physics epoch를 읽는다.
   정확히 한 step만 진행됐는지 확인하고, 양자화 전 q 차이/실제 dt로 FD를 계산한다.
   native qd는 step 전후 절댓값의 보수적인 최대를 사용한다.
2. 각 step의 encoder/정착 이력을 owner에 기록한다. 초기 epoch 이후 매 8step마다
   실제 encoder를 4096-count로 양자화해 120Hz 속도를 계산하고 실제 dual IMU를 읽는다.
   반복 epoch의 encoder sample은 기존 read timestamp를 유지한다.
3. `observe(frame)`은 실제 120Hz sample 경계에서만 허용한다.
   카메라 capture/신원 검증은 기존 owner·capture·issuer를 사용한다.
4. `refresh_packet`은 `PostCaptureFeedback`으로 원래 stepper를 16회 실행한다.
   매 8step의 encoder/IMU가 동일 epoch인지 확인하며 pre-capture IMU도 포함한
   timeline과 최신 current feedback을 반환한다.
5. refresh callback 안에서는 priority/ROS를 보내지 않는다.
   `RuntimePublicationPrepare`가 한 번 전송하고 기존 owner가 decoder·ROS·ACK를 처리한다.
6. 실제 센서 time, capture 신원, native IMU history를 다른 시각으로 바꾸지 않는다.
   읽은 timestamp가 기존 native phase/cadence 검사를 만족하지 않으면 실패한다.

어댑터는 소유 스레드 변경, 외부 physics 진행/리셋, 누락된 120Hz sample,
모델/번들 계약 불일치, 부모 소스 변경, 실패한 관측 이후 재사용을 거부한다.
step/관측 도중 실패한 경우 자동 재시작하지 않으며 owner의 pending 증거를 유지한다.

## 기록·자원 제한

- `isaac_connection.json`: 연결 객체, source/schema hash, 실제 arm index와 IMU 경로.
  연결 기록이며 센서 검증 통과 증거가 아니다.
- `parent_runtime_source.py`: 실제 부모 소스의 읽기용 사본.
- `producer_ack.jsonl` 등은 기존 v136 게시 경로에서 생성된다.
- 기존 `NativePacketTimeline`의 pending 상한은 sensor당 240개다.
  초기 대기와 capture 후 2개를 포함해 실제 native IMU가 빠짐없이 들어가야 한다.
  120Hz 이력을 상한 이상 누적하는 긴 관측 간격은 현재 계약으로 지원하지 않는다.
  history를 버리거나 timestamp를 바꿔 우회하지 않으며, 긴 미션 주기 지원은
  별도 packet 소비 설계가 필요하다.
- XYZ/XYZRGB·라이다 등은 상위 runtime의 기존 경로를 유지해야 한다.
  이 어댑터만으로 전체 센서 coverage가 검증됐다고 주장하지 않는다.


## v135 연결 근거

## 읽은 원본

- `~/.cache/isaac-fleet-r12/callback_native_v135/src/arm_practice/native_rgbd/mission_entrypoint.py`
  - SHA-256 `b08b3747cbd130c0a47d61a354a75a2bee54e77b57ad5684bc4fcbc08b80e80f`
- `~/.cache/isaac-fleet-r12/callback_native_v135/scripts/r12_sensor_sim.py`
  - SHA-256 `332f04bddf6573595e9f5f553e560a4a9553d385aa1cacf97719e88b0264c27e`

로컬 읽기용 복사본은 ignore된 `.runtime/reference_v135/`에 있다. 이 두 파일을 v136 실행 번들에 그대로 넣거나 실행하지 않는다. scene/받침대/정답 좌표 설정은 이 대응표의 연결 대상에 포함하지 않는다.

## v135 원본과 RuntimeCallbacks 대응

| v136 필드 | v135 원본과 줄 | 제공할 값/주의점 |
|---|---|---|
| rgb | r12_sensor_sim.py:296, mission_entrypoint.py:52 | native_rgb. 기본 stereo RGB와 extra_cameras를 합친 실제8개 annotator dict. 스키마 키 일치 확인. |
| depth | r12_sensor_sim.py:297, mission_entrypoint.py:52 | native_depth. 기본 depth와 extra_depth의 실제4개 annotator dict. |
| rgb_reference | r12_sensor_sim.py:176,179 | native_rgb_refs. 각 render product에 attach된 ReferenceTime annotator. |
| depth_reference | r12_sensor_sim.py:180,181 | native_depth_refs. 각 left depth에 대응하는 ReferenceTime annotator. |
| read_epoch | mission_entrypoint.py:52 | `(int(world.current_time_step_index), float(world.current_time))`를 읽는 함수. physics960Hz 기준. |
| read_joint_positions | mission_entrypoint.py:52 | robot.get_joint_positions. 과거 capture q나 target q를 대신 주지 않음. |
| arm_indices | r12_sensor_sim.py:240 | 실제 joint names에서 arm_joint1~5를 찾은 index. 0~4로 임의 고정하지 않음. |
| render | mission_entrypoint.py:52 | world.render. 소유 스레드에서 실행하며 이 callback에서 physics step을 추가하지 않음. |
| fk | mission_entrypoint.py:51 | rig.transforms. 측정 encoder 관절값 dict로 카메라 FK 계산. 현재 센서 보정/URDF 사용. |
| refresh_packet | mission_entrypoint.py:87,93,95,113,122 | PostCaptureFeedback, NativePacketTimeline, 현재 feedback을 조합하는 별도 함수. 원본 refresh_after_capture 반환값 자체는 필요한 packet이 아님. |
| clock | mission_entrypoint.py:68 및 NativeMissionCapture 기본 clock | time.monotonic_ns. default 유지 가능. ReferenceTime의 render timestamp나 sim time으로 대체하지 않음. |

ReferenceTime은 렌더 기준 시각이며 실제 노출/벽시계로 간주하지 않는다. 스키마의 카메라·depth 키와 렌더 설정이 맞는지는 실제8RGB4depth runtime에서 다시 검증해야 한다.

## refresh_packet 구현에 필요한 순서

1. `sample['certificate']['body']`의 과거 capture 신원을 보존한다.
2. 원본 `PostCaptureFeedback.run`에 실제 stepper, 현재 physics step 읽기, 실제120Hz encoder sample, 실제 dual-IMU reader를 연결한다.
3. 갱신 과정의 모든 native IMU sample을 NativePacketTimeline에 추가한다. 원본 step·sim time·sensor time을 보존한다. 이전 packet 이후의 pre-capture IMU history도 빠지면 안 된다.
4. 같은 현재 physics step의 position과 velocity를 읽는다. 원본 sample_control_encoder/current_feedback는 mission_entrypoint.py:68,72에 있다. position/velocity의 q는 같은4096-count 양자화를 사용하며, velocity에는 실제 서로 다른120Hz sample이 필요하다. 과거 q나 가짜0속도를 채우지 않는다.
5. `timeline.packet(current_step=read_epoch()[0], capture_body=sample['certificate']['body'])`를 만들고 다음 형태로 반환한다.

```python
return {
    'run_id': sample['certificate']['body']['run_id'],
    'native_packet_timeline': timing,
    'current_feedback': actual_current_feedback,
    'all_IMU_samples': timing['imu'],
}
```

`timing`은5단계에서 얻은 값이고 actual_current_feedback은4단계에서 실제 읽은 값이다. 기존 capture 시각을 최신 시각으로 바꾸지 않는다. 100ms current-feedback/500ms capture 제한은 기존 검증기가 계속 적용한다.

**원본 refresh_after_capture를 통째로 호출하면 안 된다.** mission_entrypoint.py:100,101에서 priority와 ROS current feedback을 이미 보낸다. v136의 RuntimePublicationPrepare도 이를 보내므로 그대로 붙이면 중복 전송·sequence 거부가 발생한다. 새 callback에는 센서 갱신·timeline·feedback 구성만 넣고 전송은 v136 게시 준비에 맡긴다.

원본 stationary bench의 refresh는16 physics steps 동안8step마다 encoder/IMU를2회 샘플링한다. acquire 전에48step/IMU6개를 모았던 설정은 원본 bench의 조건이다. 이 숫자들을 미션의 새 관측 값으로 위조하거나 샘플 수만 맞추지 않는다. 실제 sample identity와 간격을 검증해야 한다.

## 매 physics step에서 전달할 정착 증거

r12_sensor_sim.py:403의 apply_controls_and_step은 실제 world.step(render=False) 전후의 관절값을 읽는다. 같은 step FD는 양자화 전 위치차/실제 dt로 계산하고 native 속도는 전후 절댓값의 보수적인 최대를 쓴다. 해당 자료를 동일 step의 `session.record_physics_step(native_qd=..., same_step_fd=...)`에 전달한다. 반복되는 원본 callback에서 이미 기록한다면 이중 기록을 제거하여 같은 epoch를 두 번 넣지 않는다.

0.5초 동안 두 속도 모두0.02rad/s 미만인 기존 기준을 유지한다. sparse sample이나 마지막 한 번의 속도로 정착을 대신하지 않는다.

## 통합 시 확인할 것

- caller와 factory가 같은 initial physics epoch에서 시작하고, 기동 중 physics가 진행되지 않았는지.
- callback·record·observe·close가 같은 소유 스레드에서 호출되는지.
- 임의 fixture q/좌표/정답을 반환하는 fallback이 없는지.
- source는 factory가 계산한 새 조립 manifest 해시를 issuer/ROS/priority/prepare에 동일하게 쓰는지.
- 조립 파일을 바꾼 뒤 과거 manifest/이미 로드한 번들로 실행하지 않는지.
- 실제 child constructor 도중 실패 정리는 정상 경로의 child exit 0과 구분하여 확인한다.
