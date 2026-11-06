# pa33 v136 관측 adapter

## 현재 상태

2026-11-06 기준 기능 검증과 ACK 실측을 수행했으며 **전체 완료 판정은 미충족**이다.

| 항목 | 결과 |
|---|---|
| 실제 관측 | 고정 워밍업 2회 + 측정 8회, 촬영·인증·500ms 신선도 10/10 통과 |
| ROS 수신 | 매 관측 기본 31개 + 추가 12개 통과 |
| 거부 검증 | owner 4개, 추가 coverage IPC 4개 통과 |
| 종료 | 자식 3개 exit 0, 부모 Isaac exit -11(SIGSEGV) |
| CPU 검사 | v136 77/77, 공유 계약 314/318; 원본에서도 같은 4개 실패 |

남은 일은 부모 정상 종료 문제 처리, 기존 capture RPC의 hash·nonce·지연 메시지 native 주입 검증, 공유 계약 4개 실패의 영향 확인과 최종 재검증이다. 전체 미션·실물 성공을 뜻하지 않는다.

## 파일 구성

| 경로 | 역할 |
|---|---|
| `overlay/arm_practice/native_rgbd/` | owner·실제 센서 콜백·브리지·ACK·추가 센서 전송 |
| `native_live_decoder.py`, `native_ros_publisher.py`, `priority_feedback_consumer.py` | 자식 프로세스 3개 |
| `assemble_bundle.py`, `preflight_runtime.py` | 공유 core + overlay 조립·무결성·로컬 자산 검사 |
| `prepare_isaac_validation.py`, `run_isaac_validation.py` | 불변 실험 구성 생성·단일 실행 감시 |
| `isaac_validation_runtime.py`, `verify_isaac_validation.py` | 실제 Isaac 검증·결과 판정 |
| `probe_isaac_shutdown.py` | scene·모델 없는 종료 오류 진단 |
| `analyze_ack.py`, `tests/` | ACK 분석·CPU 회귀 검사 |
| `reference_configs/`, `runtime.example.json` | 고정 센서/보정 설정과 환경 설정 예시 |
| `reports/verification-20261106.json` | 최종 실행의 기존 검증 JSON 사본; 실패 판정 포함 |

공유 core와 원본 부모는 읽기 전용으로 사용하며, 생성된 번들은 별도 출력 폴더에 둔다.

## 환경 설정

아래 명령은 이 폴더에서 실행한다. Python·ROS·모델 경로는 각 PC에 맞춰 `runtime.local.json`에서 지정한다. 기존 로컬 설정이 있으면 유지한다.

```bash
cp -n runtime.example.json runtime.local.json
```

설정 키는 decoder Python, 모델 루트/manifest, decoder 의존성/스크립트, warmup npz, ROS Python/setup/domain이다. pa33 domain은 33이며 ROS는 localhost로 제한한다. 설정의 상대 경로는 설정 파일 위치 기준이고 `~`를 지원한다. Python symlink 경로를 유지하여 venv를 보존한다.

pa33에서 확인한 외부 자산:

- trainer: `~/.cache/isaac-fleet-r12/mission_native_perception_v23/work/perception_implementation/scripts/train_centerpose.py`
- 모델 manifest: `~/.cache/isaac-fleet-r12/mission_native_perception_v23/perception_model_manifest.json`
- warmup: `~/isaac-fleet-runs/pa33_tagged_puck_fixture_v80_20261105/native_raw_02.npz`

모델 manifest는 캐시의 `model_sha256` 형식과 `r12/MODEL_MANIFEST.json`의 동일한 3개 모델 pin을 지원한다. 실제 가중치 해시 검증은 `python -O`에서도 유지된다. warmup 자산은 초기화 전용이며 live 관측으로 사용하지 않는다.

## 조립·CPU 검사

출력 폴더는 매번 새 이름을 사용한다.

```bash
PYTHONDONTWRITEBYTECODE=1 ~/.venv-yolo/bin/python assemble_bundle.py --output .runtime/local-v136-new
PYTHONDONTWRITEBYTECODE=1 ~/.venv-yolo/bin/python preflight_runtime.py --bundle .runtime/local-v136-new --config runtime.local.json --reference-model-manifest ../../MODEL_MANIFEST.json
PYTHONDONTWRITEBYTECODE=1 ~/.venv-yolo/bin/python -m unittest discover -s tests -p 'test_*.py' -v
```

조립 도구는 core/overlay 경로 충돌, 기존 출력 폴더와 미해결 LFS pointer를 거부하고 읽은 바이트의 해시를 manifest에 기록한다. 사전 검사는 번들·스키마·FK·모델 pin·trainer 구문·warmup shape/dtype·overlay import를 검사하며 Isaac이나 실제 publisher를 시작하지 않는다. CPU mock 검사는 실제 센서 검증과 구분한다.

## 문서와 보존 자료

- [RUNTIME_INTERFACE.md](RUNTIME_INTERFACE.md): callback 계약·부모 연결 예시·v135 대응표.
- [ISAAC_VALIDATION.md](ISAAC_VALIDATION.md): 실제 실행 명령·결과·실패 이력·완료 기준.
- [ACK_ANALYSIS.md](ACK_ANALYSIS.md): ACK 정의·분석 명령·한계.
- [수정.md](수정.md): 날짜별 변경 이력. 과거 검사 수와 manifest는 당시 기록이다.

`.runtime/`, `.serena/`, Python/test cache, `runtime.local.json`은 Git 제외 대상이다. 기존 실험 원본과 정리 전 문서는 `.runtime/`에 로컬 보존한다. `reports/`의 JSON은 기존 결과 사본이며 새 실행 결과가 아니다. 센서 원본과 전체 로그는 이 사본에 포함되지 않는다.
