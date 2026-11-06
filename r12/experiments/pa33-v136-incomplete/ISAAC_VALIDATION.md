# v136 Isaac 검증 결과 — 2026-11-06

실행은 끝났고, 기록된 자기 PID48개가 모두 종료됐다. 코드·실험 결과는 v136 내부에 있다.

## 최종 판정

최종 실행: .runtime/isaac-validation-20261106-10/

| 확인 항목 | 실제 결과 |
|---|---|
| 고정 phase | 워밍업2회+측정8회, 원래 순서10회 |
| 촬영·원본 인증·decoder500ms 신선도 | 10/10 통과, 최대498.530253ms |
| 기본 ROS | 매 관측31/31 수신, raw12개 hash 확인 |
| 추가 ROS | 매 관측12/12 수신: 4카메라 XYZ4+XYZRGB4, lidar4 |
| owner 거부 | frame/소유권/source/스레드4개 통과, physics 불변 |
| 실제 추가 IPC 거부 | source/hash/nonce replay/100ms 초과 lidar4개 통과, 거부 시 발행0개 |
| child 정상 종료 | decoder0 / ROS0 / priority0 |
| 부모 정상 종료 | 실패: API close 반환 뒤 exit -11(SIGSEGV) |
| 종합 | observation_bench_pass=false, full_pa33_acceptance=false |
| 최소 종료 대조 | scene·모델 없는 Isaac도 close 반환 뒤 exit139(SIGSEGV) |
| v136 CPU | 77/77 통과 |
| 공유 native 계약 CPU | 318개 중314 통과,4 실패. 공유 core 원본에서도 같은4개 실패 재현 |
| 종료 후 정리 | 기록된 PID48개 잔여0, GPU 여유6278MiB 확인 |

추가 ROS의 cloud는 실제 capture depth/RGB와 스키마 K에서 계산한 optical-frame 진단값이다.
lidar는 실제 현재 epoch의4방향 raycast이며 image capture 시각과 별도로 보존한다.
이 결과는 물리 카메라/라이다 보정, 주행·파지·배치·전체 미션 성공을 뜻하지 않는다.
coverage IPC 거부와 기존 capture RPC 전체 오류 주입을 구분한다. 후자의 전체 native 주입 검증은 선언하지 않는다.

## ACK

정의: producer에서 ACK를 받은 직후의 monotonic ns에서 원래 capture lower-bound ns를 뺀 값.
추가12개 ROS는 기본 ACK/decoder collect 이후 전송하므로 기본 ACK 지표에 섞이지 않는다.

| 표본 | p50 ms | p95 ms | 최대 ms |
|---|---:|---:|---:|
| 최종 전체10개 | 344.9775525 | 415.6685907 | 423.826512 |
| 고정 측정8개 | 341.3023485 | 378.83819915 | 390.92231 |

v135와 scene/fixture·태그 recipe·부팅/packet 조건이 달라 위 숫자를 동일 조건 성능 향상/악화로 판정하지 않는다.

## v135에서 확인한 실제 범위

원본: /home/pa33/isaac-fleet-runs/pa33_callback_view_v135_20261105/

- ROS_publication_results.json: 10회 모두31/31·all_received=true.
- live_decoder_results.json: 10회 모두 fresh/authenticated=true, raw_hashes_verified=12.
- shutdown_exit.json과 child exit: runtime/sink 및 child3개0.
- precommitted_phase_plan.json: 워밍업2회+측정8회, 고정 순서.
- ACK 전체10개 p50 313.2897845 / p95 385.37145655 / 최대411.897922ms.
- 지지 fixture 관측 벤치다. 전체 센서/전체 미션/모든 native 오류 주입 검증은 아니다.
- 현재 공유 core의 태그 검출은1·2·3·4배율을 모두 검사한다. v135는1·2·3배율과 조건부 조기 종료를 사용했다.
  이번 v136은 현재 공유 core의 검출 방식을 유지했다.

## 실행 구성

- prepare_isaac_validation.py: 원본 source pin 후 부모 초기화/원래 actuator 본문 파생.
- run_isaac_validation.py: 기본은 CPU 사전 검사. --execute에서만 단일 native 시작/자원 감시.
- isaac_validation_runtime.py: 실제 부모 객체 연결, 거부/관측/추가 ROS/원본/실패/종료 기록.
- verify_isaac_validation.py: phase·인증/hash/신선도·31+12 수신·ACK·거부·실제 종료를 따로 판정.
- probe_isaac_shutdown.py: scene/모델 없는 일반 종료 대조.
- 공용 부모 SHA256: 5bf73b7a50355c4ef709c20999148de3295d3b5c107155666c806307b28baccf.
- 그대로 추출한 actuator 본문338~349줄 SHA256: 360e051b05a1767ed376a20453e69fd0b0724887e9bf8223343db36746e6b776.
- 최종 bundle142파일·overlay20개. bundle source SHA256:
  c2aae2535a0063a9399f4be5cd721de8dc4645cf4c5a8f461174de6c3d2a0f3d.
- scene/robot/vendor USD/정책/센서 helper 등 공유 자산153개 pin. 원본 파일은 읽기 전용.
- 960Hz physics/render timestep, 현재 측정 startup q 유지, vel=0. fixture/정답 기반 controller 추가 없음.
- 실제960step encoder 정착 이력을 child/packet 시작 전에 기록하고 새 issuer에 전달한다.
  과거 archive를 입력으로 사용하지 않으며 pre_session_encoder_history.json에 실제 기록을 보존한다.
- 센서 초기 render2회는 packet/capture 시작 전 실제 callback이다. physics 불변을 검사한다.
- packet 시작 이후의 IMU는 모두 유지한다. 관측 전48step, 촬영 후16step.
- 상단640×480·손목1920×1440의8RGB4depth와 ReferenceTime·실제 dual IMU/encoder/FK 연결.
- 기본 viewport만 비활성화하고 명시적 센서 render product8개는 유지한다.
- ROS domain33 LOCALHOST, headless GPU0, 시간300초·VRAM 여유1024MiB·디스크 여유2048MiB.
  최종 실행에서 관측된 최소 VRAM 여유1135MiB.
- PID/소유 lock 확인, 자기 process group만 정리, source/plan 변경 없음 확인.
- ROS 큐256, 소유·연속·읽기전용 원본의 byte view로 추가 복사 제거. hash/nonce·신선도 기준은 그대로 유지.
- 일반 app.close와 interpreter 종료를 별도 검사한다. 강제 exit로 성공 처리하지 않는다.

## 실패를 포함한 전체 실험 이력

폴더 공통 prefix: .runtime/isaac-validation-20261106-

| 실행 | 결과/후속 수정 |
|---|---|
| 01 | run ID 하이픈으로 ROS topic 생성 실패, 기본 fast shutdown의 최종 기록 누락 확인 |
| 02 | owner 거부4·child0, stale render epoch, 종료 중 VRAM reserve 위반으로 중단 |
| 03 | 렌더 완료 순서 적용. 실제 float32 IMU에서 고정1e-7 검사 오류 발견 |
| 04 | IMU 표현 오차 계약 수정. native float32 encoder와 선변환 float64 값의 exact q 불일치 발견 |
| 05 | q 양자화 수정. 첫 packet ROS271/271 수신, decoder534ms로500ms 거부 |
| 06 | 정착 encoder/packet 시작 구간 분리. 9/10 촬영, 일부 decoder stale |
| 07 | sealed byte view 송신. 9/10 촬영, 일부 ReferenceTime 정렬 실패 |
| 08 | render timestep/정지 GC 정리. 10/10 fresh·ROS31/31, child0, 부모-11 |
| 09 | 추가ROS12개 수신. 첫 워밍업 render timeout과 일부 decoder stale. 첫 성공 capture에서 오류 probe를 실행하도록 보완 |
| 10 | viewport 비활성·센서 사전render2회. 10/10 fresh·31+12 수신·owner4/coverage IPC4 거부·child0. 부모-11 |

최소 일반 종료: .runtime/isaac-shutdown-probe-20261106/ 와 같은 이름의 .log.
API는 반환했지만 실제 exit139. scene/모델 없이 재현되므로 현재 센서 연결 코드만의 문제로 볼 수 없다.

CPU 추가 비교: tag-parallel-parity.json / tag-thread-parity.json.
현 core의4배율 결과를 보존하는 병렬/스레드 비교를 수행했으며 속도 개선이 없어 채택하지 않았다.

## CPU 계약 검사 상세

- v136 자체: .runtime/cpu-tests-isaac-setup.log,77/77.
- 공유 native 계약: .runtime/shared-native-contract-tests-host.log,314/318.
- 남은4개: test_native_rgbd_sensor_join.py의 AdapterHistoricalPoseTests.
  pick_sensor_adapter.transforms를 mock하려는데 해당 attribute가 없어 AttributeError.
- 공유 core 원본으로 같은4개 재현: .runtime/shared-native-contract-tests-baseline.log.
  공유 core/그 테스트를 이번 작업에서 수정하지 않았다.
- 처음 샌드박스에서는 Unix socket/GPU 접근이 제한됐다. 장치와 socket에 접근 가능한 로컬 실행 환경에서 다시 확인했다.

## 완료 기준과 남은 검증

1. 고정 관측 전체의 실제 센서 신원·hash·신선도·필수 ROS 수신을 통과한다.
2. 기존 capture RPC의 잘못된 hash·nonce·지연 메시지 거부를 native 경로에서 검증한다. 현재 추가 coverage IPC의 4개 거부 결과로 대체하지 않는다.
3. producer ACK 시작·수신 완료 시각과 고정 phase별 통계를 보존한다.
4. 부모와 모든 자식이 exit 0으로 정상 종료하고 소유 프로세스가 남지 않는다.
5. 공유 계약 4개 실패의 영향을 확인·처리하고 변경 후 최종 검증을 반복한다.

현재 2·4·5가 남아 있으며 전체 완료 판정은 false다.

## 증거 파일과 재분석

저장소에 포함한 결과는 [reports/verification-20261106.json](reports/verification-20261106.json)이다. 최종 실행의 원본 `verification.json`과 동일한 바이트이며 SHA-256은 `72186c72cbfee18fd79f9bddbbfb4642d1c98bf690e79fdbd822a4f3f8eefd2f`다. 센서 원본과 로그는 로컬 `.runtime/`에 보존하며 Git에는 포함하지 않는다. JSON의 절대 경로는 측정 당시 출처다.

최종 snapshot 안의 주요 증거:

- verification.json: 센서/추가ROS/IPC 거부/child 통과와 부모-11을 분리한 판정.
- supervisor_exit.json: 실제 부모 returncode=-11, source/plan unchanged=true.
- run/observations.json, runtime_rejections.json, coverage_native_rejections.json.
- run/observer/producer_ack.jsonl, ROS_publication_results.json, coverage_results.json.
- run/raw_*.npz, certificate_*.json, reference_readback_*.json, clouds_*.npz.
- run/observer/pre_session_encoder_history.json, run/shutdown.json.
- resource_samples.jsonl, native.log, readiness.json.
- 전체 종료 PID 확인: .runtime/owned-process-audit.json.

~~~bash
cd /home/pa33/git/lv2-nz2-project2/r12/experiments/pa33-v136-incomplete
PYTHONDONTWRITEBYTECODE=1 /home/pa33/.venv-yolo/bin/python .runtime/isaac-validation-20261106-10/verify_isaac_validation.py --plan .runtime/isaac-validation-20261106-10/plan.json
~~~

이 재분석은 부모 종료 오류 때문에2를 반환하는 것이 현재 정상이다.
새 실행은 prepare 도구에 새 output을 지정한다. 기존 snapshot/run은 덮어쓰지 않는다.
공유 core/부모 원본 변경 없음. 부모 일반 종료 문제와 공유 계약4개 실패가 남아 있다.
