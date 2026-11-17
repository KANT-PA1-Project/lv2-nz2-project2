# ACK 분석 — v135와 v136의 동일한 endpoint

2026-11-06. 이 도구는 로컬 파일을 읽는 CPU 분석만 수행한다. 실제 ROS/Isaac 기동, 미션 합격 판정, 모델 교체를 수행하지 않는다.

## 고정한 정의

```text
producer_ACK_ms = (ack_receive_end_ns - capture_monotonic_ns) / 1,000,000
```

시작은 certificate의 `capture_monotonic_lower_bound_ns`를 그대로 복사한 정수 ns다. 끝은 producer에서 `receive_packet`이 돌아온 직후의 정수 `ack_receive_end_ns`다. 둘 다 같은 호스트의 monotonic clock이다. `capture_stamp_ns`는 시뮬레이션 시각이므로 이 계산의 시작점으로 사용하지 않는다. `ack_validation_end_ns`도 기존 v135 지표의 끝점으로 사용하지 않는다.

백분위수는 정렬한 값에서 `(n-1)*q` 위치를 선형 보간한다. v135 원본 10개에 대한 결과는 기존 `pa33-v135-comparison.json`과 일치한다.

| 표본 | p50 ms | p95 ms | 최대 ms |
|---|---:|---:|---:|
| v135 10개 | 313.2897845 | 385.37145654999995 | 411.897922 |

v135 원본 파일 SHA-256은 `cd100ceb2569b624ee422dc72f27952bcb1510d1d155d8b664ebe0b791aefd14`다. `tests/data/v135_producer_frames_timing.json`은 이 파일의 분석에 필요한 시각·신원·수신 수량 필드만 보존한 회귀 테스트 자료다. 이미지·정답 좌표는 포함하지 않는다. 출처는 `tests/data/v135_expected.json`에 기록했다.

## 명령

```bash
cd /home/pa33/git/lv2-nz2-project2/r12/experiments/pa33-v136-incomplete
python3 analyze_ack.py --input /home/pa33/isaac-fleet-runs/pa33_callback_view_v135_20261105 --output .runtime/ack-v135.json
```

v136 기록은 해당 run 폴더 또는 정확한 `producer_ack.jsonl`을 `--input`에 준다. v135와 비교하려면 같은 명령에 `--baseline /home/pa33/isaac-fleet-runs/pa33_callback_view_v135_20261105`를 추가한다. 후보에서 baseline을 뺀 p50/p95/max 차이를 기록한다. 같은 데이터로 비교한 단위 검사에서는 차이가0이다. CPU mock 관측을 실제 v136 실측 자료로 취급하지 않는다.

`--output`이 있으면 상세 JSON은 그 파일에 저장하고 stdout에는 요약을 표시한다. 입력 파일과 baseline 파일을 출력으로 덮어쓰는 요청은 거부한다. Python 표준 라이브러리만 사용하므로 학습/ROS 환경이 필요하지 않다.

## v136 기록

`LivePublicationBridge.publish`는 ACK의 capture stamp·도메인을 확인한 뒤, producer 출력 폴더의 `producer_ack.jsonl`에 다음을 같은 행으로 남긴다.

- run_id, clock_epoch, source_sha256, capture physics step과 sim stamp
- 정확한 capture monotonic ns, producer transport timing의 ACK 수신 완료 ns
- 필수 ROS sample expected/received 수, 오류, domain

기록은 ACK 측정 endpoint 이후이면서 decoder collect 이전에 저장된다. 따라서 JSONL 쓰기 비용은 producer_ACK_ms에 들어가지 않고, 이후 decoder collect가 실패해도 이미 확인한 ACK 행은 남는다. JSONL 저장 실패는 예외로 전달되며 코어 ObservationOwner는 실패 상태와 pending 원본을 유지한다. capture/start 신원은 `producer_capture_identity`로 반환 결과에도 포함한다.

`ROS_ack_timeline.jsonl`은 child가 ACK를 보내는 쪽의 부가 기록이다. producer의 수신 완료 endpoint가 없으므로 그것만으로 producer_ACK_ms를 계산하지 않는다. 도구는 JSONL을 읽을 때 새 producer journal을 사용하고, 기존 v135 run 폴더에서는 `native_producer_frames.json`을 사용한다. 둘 다 있으면 producer journal을 우선하며 자동 병합하지 않는다.

## 오류와 세부 구간

필수 시각 누락, 다른 clock, bool/음수 시각, capture step과 stamp 불일치, 수신 미완료, 중복·역순 capture, 여러 run/clock 혼합, source/domain 변경, 잘린 JSONL은 오류다. 오류가 있으면 정상 `summary`는 null이고 CLI는2를 반환한다. 유효한 부분의 수치는 `valid_subset_summary`에만 별도로 보존한다. captured=false 관측은 `skipped_uncaptured`로 집계한다.

blob copy, serialization, Unix send, ACK receive/validation 구간도 기록된 시각에 한해 분석한다. 현재 v136 `timed_send` wrapper에는 serialization_end_ns와 unix_send_start_ns가 없으므로 두 세부 구간은 null과 누락 개수로 표시한다. 합쳐진 serialization_and_send 구간과 전체 producer ACK는 계산할 수 있다. 없는 시각을0이나 다른 clock으로 채우지 않는다.

수치가 일치하거나 단위 검사가 통과해도 전체 미션·인식·파지·센서 coverage가 인증되는 것은 아니다. v136 실제 실측 결과는 [ISAAC_VALIDATION.md](ISAAC_VALIDATION.md)에 있다. v135와 실험 조건이 달라 동일 조건 성능 비교로 해석하지 않는다.
