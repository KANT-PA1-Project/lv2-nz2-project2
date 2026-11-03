// =============================================================
// 패키지 : perception (nz2_perception 과 같은 코드)
// 파일   : src/yolo_engine.cpp
// 역할   : YOLO 공통 전처리/후처리 + 백엔드 선택
// =============================================================
#include "nz2_perception/yolo_engine.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace nz2_perception
{

// 백엔드별 생성 함수 (각 cpp 에 구현)
std::unique_ptr<YoloEngine> create_opencv_engine(
  const std::string & path, int in_h, int in_w, int num_classes, std::string & error);
#ifdef NZ2_WITH_TENSORRT
std::unique_ptr<YoloEngine> create_tensorrt_engine(
  const std::string & path, int in_h, int in_w, int num_classes, std::string & error);
#endif

bool tensorrt_available()
{
#ifdef NZ2_WITH_TENSORRT
  return true;
#else
  return false;
#endif
}

cv::Mat letterbox(const cv::Mat & bgr, int in_h, int in_w, LetterboxInfo & info)
{
  const float s = std::min(static_cast<float>(in_w) / bgr.cols, static_cast<float>(in_h) / bgr.rows);
  const int nw = static_cast<int>(std::round(bgr.cols * s));
  const int nh = static_cast<int>(std::round(bgr.rows * s));
  info.scale = s;
  info.pad_x = (in_w - nw) / 2.0f;
  info.pad_y = (in_h - nh) / 2.0f;

  cv::Mat resized;
  if (nw != bgr.cols || nh != bgr.rows) {
    cv::resize(bgr, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
  } else {
    resized = bgr;
  }
  const int top = static_cast<int>(std::round(info.pad_y - 0.1f));
  const int left = static_cast<int>(std::round(info.pad_x - 0.1f));
  cv::Mat out;
  cv::copyMakeBorder(
    resized, out, top, in_h - nh - top, left, in_w - nw - left,
    cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  info.pad_x = static_cast<float>(left);
  info.pad_y = static_cast<float>(top);
  return out;
}

cv::Mat make_blob(const cv::Mat & letterboxed)
{
  // 1/255 정규화 + BGR→RGB + HWC→NCHW (blobFromImage 가 한 번에 처리)
  return cv::dnn::blobFromImage(letterboxed, 1.0 / 255.0, cv::Size(), cv::Scalar(), true, false, CV_32F);
}

namespace
{
// 모델 입력 좌표 → 원본 좌표
cv::Rect2f unletterbox(float x1, float y1, float x2, float y2, const LetterboxInfo & li, const cv::Size & orig)
{
  auto fx = [&](float v) {return std::clamp((v - li.pad_x) / li.scale, 0.0f, static_cast<float>(orig.width - 1));};
  auto fy = [&](float v) {return std::clamp((v - li.pad_y) / li.scale, 0.0f, static_cast<float>(orig.height - 1));};
  const float a = fx(x1), b = fy(y1), c = fx(x2), d = fy(y2);
  return cv::Rect2f(a, b, std::max(0.0f, c - a), std::max(0.0f, d - b));
}
}  // namespace

bool decode_output(
  const float * data, const std::vector<int64_t> & shape, int num_classes,
  float conf, float iou, const LetterboxInfo & info, const cv::Size & orig,
  std::vector<Detection> & out, std::string & err)
{
  out.clear();
  if (shape.size() != 3 || shape[0] != 1) {
    err = "출력 차원이 [1, A, B] 가 아님";
    return false;
  }
  const int64_t A = shape[1], B = shape[2];
  const int64_t C = 4 + num_classes;

  // ---- end2end: [1, N, 6] (x1 y1 x2 y2 score cls), 이미 NMS 됨 ----
  if (B == 6 && A != C) {
    for (int64_t i = 0; i < A; ++i) {
      const float * r = data + i * 6;
      if (r[4] < conf) {continue;}
      const int cls = static_cast<int>(std::lround(r[5]));
      if (cls < 0 || cls >= num_classes) {continue;}
      out.push_back({cls, r[4], unletterbox(r[0], r[1], r[2], r[3], info, orig)});
    }
    return true;
  }

  // ---- YOLOv8/11: [1, C, N] 또는 [1, N, C] ----
  bool chan_first;
  int64_t N;
  if (A == C) {chan_first = true; N = B;}
  else if (B == C) {chan_first = false; N = A;}
  else {
    err = "출력 [1," + std::to_string(A) + "," + std::to_string(B) + "] 가 클래스 수(" +
      std::to_string(num_classes) + ")와 안 맞음 → class_names 와 모델 클래스 확인";
    return false;
  }
  auto at = [&](int64_t c, int64_t i) {return chan_first ? data[c * N + i] : data[i * C + c];};

  std::vector<cv::Rect2d> boxes;      // NMS 용 (클래스별로 좌표를 띄워서 한 번에 처리)
  std::vector<float> scores;
  std::vector<int> classes;
  std::vector<cv::Rect2f> raw;
  for (int64_t i = 0; i < N; ++i) {
    int best = -1;
    float bs = conf;
    for (int c = 0; c < num_classes; ++c) {
      const float s = at(4 + c, i);
      if (s >= bs) {bs = s; best = c;}
    }
    if (best < 0) {continue;}
    const float cx = at(0, i), cy = at(1, i), w = at(2, i), h = at(3, i);
    const float x1 = cx - w / 2, y1 = cy - h / 2;
    const double off = best * 4096.0;   // 클래스가 다르면 겹치지 않게
    boxes.emplace_back(x1 + off, y1 + off, w, h);
    scores.push_back(bs);
    classes.push_back(best);
    raw.emplace_back(x1, y1, w, h);
  }
  std::vector<int> keep;
  cv::dnn::NMSBoxes(boxes, scores, conf, iou, keep);
  for (int k : keep) {
    const auto & r = raw[k];
    out.push_back({classes[k], scores[k], unletterbox(r.x, r.y, r.x + r.width, r.y + r.height, info, orig)});
  }
  return true;
}

std::unique_ptr<YoloEngine> create_yolo_engine(
  const std::string & backend, const std::string & model_path,
  int in_h, int in_w, int num_classes, std::string & error)
{
  std::string b = backend;
  if (b == "auto") {
    auto ends = [&](const std::string & s) {
        return model_path.size() >= s.size() &&
               model_path.compare(model_path.size() - s.size(), s.size(), s) == 0;
      };
    b = ends(".onnx") ? "opencv" : "tensorrt";
  }
  if (b == "tensorrt") {
#ifdef NZ2_WITH_TENSORRT
    return create_tensorrt_engine(model_path, in_h, in_w, num_classes, error);
#else
    error = "이 빌드에는 TensorRT 백엔드가 없음 (CUDA/TensorRT 를 못 찾고 빌드됨). "
      ".onnx 모델 + backend:=opencv 를 쓰거나 Jetson 에서 다시 빌드";
    return nullptr;
#endif
  }
  if (b == "opencv") {
    return create_opencv_engine(model_path, in_h, in_w, num_classes, error);
  }
  error = "backend 는 auto / tensorrt / opencv 중 하나: " + backend;
  return nullptr;
}

}  // namespace nz2_perception
