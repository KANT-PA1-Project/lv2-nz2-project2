// =============================================================
// 패키지 : perception (nz2_perception 과 같은 코드)
// 파일   : include/nz2_perception/yolo_engine.hpp
// 역할   : YOLO 추론 엔진 공통 인터페이스 (ROS 와 무관한 순수 C++)
//          - TensorRT 백엔드 (.engine, Jetson 본선)   → yolo_engine_tensorrt.cpp
//          - OpenCV DNN 백엔드 (.onnx, 노트북 CPU 테스트) → yolo_engine_opencv.cpp
//          전처리(레터박스)·후처리(디코드+NMS)는 두 백엔드가 같이 쓴다 → yolo_engine.cpp
// 출력 형식 자동 판별
//   - YOLOv8/11 : [1, 4+nc, N]  (cx, cy, w, h, 클래스별 점수)  → NMS 필요
//   - end2end(YOLO26 등) : [1, N, 6]  (x1, y1, x2, y2, score, class) → NMS 불필요
// =============================================================
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace nz2_perception
{

// 검출 1개 (원본 영상 픽셀 좌표)
struct Detection
{
  int class_id = -1;
  float score = 0.0f;
  cv::Rect2f box;
};

// 레터박스 변환 정보 (원본 → 모델 입력). 결과 좌표를 원본으로 되돌릴 때 쓴다
struct LetterboxInfo
{
  float scale = 1.0f;
  float pad_x = 0.0f;
  float pad_y = 0.0f;
};

// 원본 BGR → in_w x in_h 레터박스 (비율 유지 축소 + 가운데 정렬, 회색 114 패딩: ultralytics 와 동일)
cv::Mat letterbox(const cv::Mat & bgr, int in_h, int in_w, LetterboxInfo & info);

// 레터박스 영상 → NCHW float32, RGB, 0~1
cv::Mat make_blob(const cv::Mat & letterboxed);

// 모델 출력 → 원본 좌표 검출 목록. 형식을 모르면 false + err
bool decode_output(
  const float * data, const std::vector<int64_t> & shape, int num_classes,
  float conf, float iou, const LetterboxInfo & info, const cv::Size & orig,
  std::vector<Detection> & out, std::string & err);

class YoloEngine
{
public:
  virtual ~YoloEngine() = default;
  // BGR 1장 → 검출 목록 (실패 시 예외 대신 빈 목록 + last_error())
  virtual std::vector<Detection> infer(const cv::Mat & bgr, float conf, float iou) = 0;
  virtual std::string name() const = 0;
  const std::string & last_error() const {return last_error_;}

protected:
  std::string last_error_;
};

// backend: "auto"(확장자로 결정: .engine → tensorrt, .onnx → opencv) / "tensorrt" / "opencv"
// 실패하면 nullptr + error 에 이유
std::unique_ptr<YoloEngine> create_yolo_engine(
  const std::string & backend, const std::string & model_path,
  int in_h, int in_w, int num_classes, std::string & error);

// 이 빌드에 TensorRT 백엔드가 들어 있는지
bool tensorrt_available();

}  // namespace nz2_perception
