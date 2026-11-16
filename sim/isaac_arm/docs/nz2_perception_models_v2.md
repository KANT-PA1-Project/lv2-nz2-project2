# NZ2 새 인식 모델 (튜터님 제공, 11/11) — 보류, 원래 모델로 복구

## 결정 (11/11 저녁)
- 튜터님 모델은 **보류**. 원래 9클래스 `models/nz2_puck_best.pt` (md5 3aaedb9e, PR #26 과 같은 모델) 로 복구
- 튜터님 모델은 저장소에 올리지 않음 (개발 PC `models/tutor_1111/` 보관: nz2_yolo6_best.pt 82e841ad, nz2_centerpose_best.pt 4daa2e74)
- 재개 조건: 튜터님께 `train_centerpose.py` 를 받고 시간이 남을 때

## 모델 정보
| 파일 | md5 | 내용 |
|---|---|---|
| nz2_yolo6_best.pt (18.3 MB) | 82e841ad | ultralytics 8.3.221 YOLOv8 (C2f), 6 클래스: 0 red_cylinder, 1 red_cuboid, 2 green_cylinder, 3 green_cuboid, 4 box, 5 apriltag. imgsz 800, P 0.976 / R 0.940 / mAP50 0.975 / mAP50-95 0.856 |
| nz2_centerpose_best.pt (75.5 MB) | 4daa2e74 | CenterPose DLA-34 (DCN 없이 일반 conv), 헤드 hm 1, wh 2, hps 16(8 꼭짓점), reg 2, hm_hp 8, hp_offset 2, scale 3. symmetry_aware, metric_depth_head. 퍽 0.03×0.03×0.06 |
- YOLO 에 세움/누움 클래스 없음 → 자세는 CenterPose 가 담당

## 추론 코드 위치 (11/11 GitHub 확인)
- tutor 브랜치에 사용 예만 있음:
  - `r12/scripts/r12_perception_worker.py`: `from train_centerpose import load_model, infer` → `infer(pose, image_bgr, yolo_xyxy, K, device='cpu', dimensions_m=(.03,.03,.06), views='scale3_median')`, YOLO 클래스 0~3(퍽)만 자세 추정, 반환 `center_camera_m`, `translation_valid` 등. 깊이 일치 검사 = 박스 중심 5×5 깊이 중앙값과 |차| < 0.1 m
  - `r12/scripts/export_r12_perception_onnx.py`: CenterPose 입력 128×128 크롭, 3장 배치(scale3), 출력 hm·hps·reg·scale
  - `r12/src/arm_practice/trt_perception.py`: YOLO letterbox 800, NMS iou 0.7
  - `r12/models/perception/inference.json`: imgsz 800, pose_views scale3_median, detector_conf 0.25, pose_conf 0.25
- `train_centerpose.py` 는 공개 저장소 어디에도 없음 (전 브랜치 검색). 튜터님 로컬 `turtlebot-swerve-cuvslam/perception/scripts/`

## 시뮬 카메라 빈 화면 진단 (보류, 원인 미해결)
| 실험 | 조건 | 결과 |
|---|---|---|
| run.sh cam, A·B | Fixed 작업, zero_agent, 4 env, 팔 바닥(z 0), STAGE 1 | 통과 |
| 040 | percep_run.sh: Residual 작업, NZ2_PLAY=1, 8 env, r20d(팔 0.209), STAGE 3, WAIT 2 s | wrist·top·percep 전부 빈 화면 1~5 s 내내 |
| 041 | 040 에서 r20d 만 뺌 | 똑같이 전부 빈 화면 → 팔 높이·예열 문제 아님 |
- 남은 차이: 작업(Residual vs Fixed), NZ2_PLAY=1, env 수 8 vs 4, STAGE 3 vs 1
- Residual 작업의 정책 영상 입력(wrist/top)도 빈 화면 → 잔차 RL 학습 때 영상 정보가 없었을 가능성 높음
