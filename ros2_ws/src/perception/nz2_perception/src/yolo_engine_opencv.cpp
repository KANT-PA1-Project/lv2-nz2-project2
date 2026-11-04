// =============================================================
// 패키지 : perception (nz2_perception 과 같은 코드)
// 파일   : src/yolo_engine_opencv.cpp
// 역할   : OpenCV DNN 백엔드 (.onnx, CPU). 노트북에서 TensorRT 없이 노드를 시험할 때 사용
//          ONNX 만들기 (노트북, yolo_env):
//            yolo export model=models/current.pt format=onnx imgsz=384,640 opset=12
// =============================================================
#include <fstream>

#include <opencv2/dnn.hpp>

#include "nz2_perception/yolo_engine.hpp"

namespace nz2_perception
{

class OpenCvYoloEngine : public YoloEngine
{
public:
  OpenCvYoloEngine(cv::dnn::Net net, int in_h, int in_w, int nc)
  : net_(std::move(net)), in_h_(in_h), in_w_(in_w), nc_(nc) {}

  std::vector<Detection> infer(const cv::Mat & bgr, float conf, float iou) override
  {
    std::vector<Detection> dets;
    LetterboxInfo li;
    const cv::Mat lb = letterbox(bgr, in_h_, in_w_, li);
    net_.setInput(make_blob(lb));
    cv::Mat out = net_.forward();
    if (out.dims != 3) {
      last_error_ = "ONNX 출력 차원이 3 이 아님";
      return dets;
    }
    if (out.type() != CV_32F) {out.convertTo(out, CV_32F);}
    const std::vector<int64_t> shape = {out.size[0], out.size[1], out.size[2]};
    if (!decode_output(out.ptr<float>(), shape, nc_, conf, iou, li, bgr.size(), dets, last_error_)) {
      dets.clear();
    }
    return dets;
  }

  std::string name() const override {return "opencv-dnn(cpu)";}

private:
  cv::dnn::Net net_;
  int in_h_, in_w_, nc_;
};

std::unique_ptr<YoloEngine> create_opencv_engine(
  const std::string & path, int in_h, int in_w, int num_classes, std::string & error)
{
  if (!std::ifstream(path).good()) {
    error = "모델 파일 없음: " + path;
    return nullptr;
  }
  try {
    cv::dnn::Net net = cv::dnn::readNet(path);
    if (net.empty()) {
      error = "OpenCV DNN 이 모델을 못 읽음: " + path;
      return nullptr;
    }
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    return std::make_unique<OpenCvYoloEngine>(std::move(net), in_h, in_w, num_classes);
  } catch (const cv::Exception & e) {
    error = std::string("OpenCV DNN 로딩 실패: ") + e.what();
    return nullptr;
  }
}

}  // namespace nz2_perception
