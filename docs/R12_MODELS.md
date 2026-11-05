# 모델과 실행 준비

## 포함 모델

전체 파일별 크기·SHA-256은 [`r12/MODEL_MANIFEST.json`](../r12/MODEL_MANIFEST.json)에 있다. 파일의 최신성, 학습 완료, 단위 평가, 전체 미션 합격은 서로 다르다. 업로드된 모든 모델을 실물/전체 미션 합격 모델로 취급하지 않는다.

| 모델 | 위치 | 근거·한계 |
|---|---|---|
| YOLO 인식 | `r12/models/perception/yolo/weights/best.pt` | 현재 bench runtime 사용 모델. SHA `e844ec62ac01bb42272ce4a05f2f39128e2d57306b9857d96ef855c10757c473` |
| CenterPose | `r12/models/perception/centerpose/best.pt` | 현재 bench runtime 사용 모델. SHA `3ed5fe5711a5d96298a829a491ab0b1cc33f642434a3e3dc0a166ba60b2d1ae4` |
| 인식 설정 | `r12/models/perception/inference.json` | 원본 유지. `box_dimensions_confirmed=false`; 미확정 상자 치수를 보정된 진실값으로 사용 금지 |
| 스월브 정책 | `r12/models/swerve/` | 기존 버전·deployed 선택 기록을 함께 보존. latest.json은 원본의 선택 기록이며 이번 게시에 의한 새 승격이 아님 |
| 팔 reach 정책 | `r12/models/r12/policy/` | 기존 학습/추론 산출물. 완전한 파지·운반·배치 정책 인증 아님 |
| navigation/scene | `r12/models/navigation/` | 정책·지도·fixture·scene. fixture 좌표를 센서 관측 대신 제어에 주입하지 않음 |
| CAD/URDF/USD | `r12/models/r12/`, `r12/assets/vendor/` | 개발용 참조 자산. 설치된 Isaac/ROS 런타임과 상대/절대 참조 해석 필요 |

AprilTag 해독은 학습 모델 대신 tag36h11 패턴 검출·해독을 사용한다. 색/형상 인식 성능, 태그 해독, 보정된 3D 위치 추정은 별도 검증한다. 추가 학습이 영구적으로 불필요하다고 결론내린 상태가 아니다.

## 다운로드

큰 파일은 Git LFS로 보존했다. 일반 Git pointer만 받은 상태에서 실행하지 않는다.

```bash
git lfs install
git clone --branch tutor https://github.com/KANT-PA1-Project/lv2-nz2-project2.git
cd lv2-nz2-project2
git lfs pull
python3 r12/tools/verify_model_manifest.py
```

## 실행 준비 한계

- Python ≥3.12 코어 의존성은 `r12/pyproject.toml`에 있다. Isaac Sim/Isaac Lab/ROS는 별도 설치 의존성이며 이 저장소가 포함하지 않는다.
- 환경을 새로 만들 때 원본 실험의 런타임 버전과 manifest를 확인한다. 타 프로젝트 환경을 업그레이드하지 않는다.
- `r12/experiments/`는 재현용 소스 보관이며 호스트 고정 경로가 남아 있다. local 경로/모델/도메인/port를 확인하고 단위 CPU 시험, 짧은 headless 기동·정상 종료 후 평가한다.
- 현재 원격 실행은 금지다. 아래 CPU 검증 예시에는 로봇/Isaac 실행이 없다.

```bash
cd r12
python3 -m venv .venv
.venv/bin/pip install -e . pytest
.venv/bin/python -m pytest tests/test_capture_lifecycle.py tests/test_observation_owner.py tests/test_native_stationary_gc.py
```

전체 미션 실행·센서 수신·GPU 물리·추론 검증은 위 CPU 시험으로 대체할 수 없다. 모델 권리/출처는 기존 소스 LICENSE와 vendor/학습 모델의 원래 조건을 따른다. 이 스냅샷은 새로운 사용권을 부여하지 않는다.

## 최신 팔 평가 후보

`models/candidates/local-arm-gamma999/`와 `models/candidates/pa28-arm-v145/`에 최신 평가 checkpoint·export·검증 결과를 별도 포함했다. 기존 호환 모델을 교체하지 않았으며 자동 승격 금지다.
