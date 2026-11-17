// =============================================================
// 패키지 : perception (nz2_perception 과 같은 코드)
// 파일   : src/yolo_engine_tensorrt.cpp
// 역할   : TensorRT 백엔드 (.engine, FP16). Jetson Orin Nano 본선용
//          엔진 만들기 (Jetson): bash jetson/build_engine.sh models/current.pt 384 640
// 주의
//   - 엔진은 그걸 만든 TensorRT 버전·GPU 에서만 열린다 → 반드시 Jetson 에서 빌드
//   - ultralytics 가 만든 .engine 은 앞에 [4바이트 길이 + JSON 메타데이터]가 붙어 있다 → 건너뛰고 읽음
//   - TensorRT 8.5+ / 10.x 공통 API(enqueueV3, setTensorAddress) 사용 (JetPack 6.x 대응)
// =============================================================
#include <cuda_runtime_api.h>
#include <NvInfer.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <numeric>

#include "nz2_perception/yolo_engine.hpp"

namespace nz2_perception
{

namespace
{

class TrtLogger : public nvinfer1::ILogger
{
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {std::cerr << "[TensorRT] " << msg << std::endl;}
  }
};

TrtLogger g_logger;

size_t elem_size(nvinfer1::DataType t)
{
  switch (t) {
    case nvinfer1::DataType::kFLOAT: return 4;
    case nvinfer1::DataType::kHALF: return 2;
    case nvinfer1::DataType::kINT32: return 4;
    case nvinfer1::DataType::kINT8: return 1;
    default: return 4;
  }
}

int64_t volume(const nvinfer1::Dims & d)
{
  int64_t v = 1;
  for (int i = 0; i < d.nbDims; ++i) {v *= std::max<int64_t>(d.d[i], 1);}
  return v;
}

#define CUDA_OK(call) ((call) == cudaSuccess)

}  // namespace

class TrtYoloEngine : public YoloEngine
{
public:
  ~TrtYoloEngine() override
  {
    if (d_in_) {cudaFree(d_in_);}
    if (d_out_) {cudaFree(d_out_);}
    if (stream_) {cudaStreamDestroy(stream_);}
  }

  bool init(const std::string & path, int in_h, int in_w, int nc, std::string & err)
  {
    in_h_ = in_h;
    in_w_ = in_w;
    nc_ = nc;

    // ---- 파일 읽기 (+ ultralytics 메타데이터 헤더 건너뛰기) ----
    std::ifstream f(path, std::ios::binary);
    if (!f) {err = "엔진 파일 없음: " + path; return false;}
    std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    size_t off = 0;
    if (buf.size() > 8) {
      int32_t meta_len = 0;
      std::memcpy(&meta_len, buf.data(), 4);
      if (meta_len > 0 && static_cast<size_t>(meta_len) + 4 < buf.size() && buf[4] == '{') {
        off = 4 + static_cast<size_t>(meta_len);
      }
    }

    runtime_.reset(nvinfer1::createInferRuntime(g_logger));
    if (!runtime_) {err = "TensorRT 런타임 생성 실패"; return false;}
    engine_.reset(runtime_->deserializeCudaEngine(buf.data() + off, buf.size() - off));
    if (!engine_) {
      err = "엔진 역직렬화 실패 (다른 TensorRT 버전/기기에서 만든 엔진일 가능성 → Jetson 에서 다시 빌드)";
      return false;
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {err = "실행 컨텍스트 생성 실패"; return false;}

    // ---- 입출력 텐서 찾기 (입력 1개, 출력 1개 가정: ultralytics detect 모델) ----
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * name = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {in_name_ = name;}
      else if (out_name_.empty()) {out_name_ = name;}
    }
    if (in_name_.empty() || out_name_.empty()) {err = "입출력 텐서를 못 찾음"; return false;}

    nvinfer1::Dims in_d = engine_->getTensorShape(in_name_.c_str());
    if (in_d.nbDims != 4) {err = "입력이 4차원(NCHW)이 아님"; return false;}
    if (in_d.d[2] < 0 || in_d.d[3] < 0) {          // 동적 입력이면 지정 크기로 고정
      nvinfer1::Dims4 want{1, 3, in_h_, in_w_};
      if (!context_->setInputShape(in_name_.c_str(), want)) {err = "동적 입력 크기 설정 실패"; return false;}
    } else if (in_d.d[2] != in_h_ || in_d.d[3] != in_w_) {
      err = "엔진 입력 " + std::to_string(in_d.d[2]) + "x" + std::to_string(in_d.d[3]) +
        " ≠ 파라미터 input_height x input_width " + std::to_string(in_h_) + "x" + std::to_string(in_w_);
      return false;
    }
    in_type_ = engine_->getTensorDataType(in_name_.c_str());
    out_type_ = engine_->getTensorDataType(out_name_.c_str());
    if ((in_type_ != nvinfer1::DataType::kFLOAT && in_type_ != nvinfer1::DataType::kHALF) ||
      (out_type_ != nvinfer1::DataType::kFLOAT && out_type_ != nvinfer1::DataType::kHALF))
    {
      err = "입출력 자료형은 FP32/FP16 만 지원";
      return false;
    }
    const nvinfer1::Dims out_d = context_->getTensorShape(out_name_.c_str());
    out_shape_.assign(out_d.d, out_d.d + out_d.nbDims);
    in_bytes_ = static_cast<size_t>(1 * 3 * in_h_ * in_w_) * elem_size(in_type_);
    out_count_ = static_cast<size_t>(volume(out_d));
    out_bytes_ = out_count_ * elem_size(out_type_);

    if (!CUDA_OK(cudaStreamCreate(&stream_)) ||
      !CUDA_OK(cudaMalloc(&d_in_, in_bytes_)) || !CUDA_OK(cudaMalloc(&d_out_, out_bytes_)))
    {
      err = "CUDA 메모리 할당 실패";
      return false;
    }
    context_->setTensorAddress(in_name_.c_str(), d_in_);
    context_->setTensorAddress(out_name_.c_str(), d_out_);
    host_out_.resize(out_bytes_);
    return true;
  }

  std::vector<Detection> infer(const cv::Mat & bgr, float conf, float iou) override
  {
    std::vector<Detection> dets;
    LetterboxInfo li;
    cv::Mat blob = make_blob(letterbox(bgr, in_h_, in_w_, li));   // NCHW float32
    if (in_type_ == nvinfer1::DataType::kHALF) {blob.convertTo(blob, CV_16F);}
    if (!blob.isContinuous()) {blob = blob.clone();}

    if (!CUDA_OK(cudaMemcpyAsync(d_in_, blob.data, in_bytes_, cudaMemcpyHostToDevice, stream_)) ||
      !context_->enqueueV3(stream_) ||
      !CUDA_OK(cudaMemcpyAsync(host_out_.data(), d_out_, out_bytes_, cudaMemcpyDeviceToHost, stream_)) ||
      !CUDA_OK(cudaStreamSynchronize(stream_)))
    {
      last_error_ = "TensorRT 추론 실패";
      return dets;
    }

    const float * out = reinterpret_cast<const float *>(host_out_.data());
    cv::Mat tmp;
    if (out_type_ == nvinfer1::DataType::kHALF) {
      cv::Mat half(1, static_cast<int>(out_count_), CV_16F, host_out_.data());
      half.convertTo(tmp, CV_32F);
      out = tmp.ptr<float>();
    }
    if (!decode_output(out, out_shape_, nc_, conf, iou, li, bgr.size(), dets, last_error_)) {
      dets.clear();
    }
    return dets;
  }

  std::string name() const override {return "tensorrt";}

private:
  int in_h_ = 384, in_w_ = 640, nc_ = 9;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::string in_name_, out_name_;
  nvinfer1::DataType in_type_ = nvinfer1::DataType::kFLOAT, out_type_ = nvinfer1::DataType::kFLOAT;
  std::vector<int64_t> out_shape_;
  size_t in_bytes_ = 0, out_bytes_ = 0, out_count_ = 0;
  void * d_in_ = nullptr;
  void * d_out_ = nullptr;
  cudaStream_t stream_ = nullptr;
  std::vector<uint8_t> host_out_;
};

std::unique_ptr<YoloEngine> create_tensorrt_engine(
  const std::string & path, int in_h, int in_w, int num_classes, std::string & error)
{
  auto e = std::make_unique<TrtYoloEngine>();
  if (!e->init(path, in_h, in_w, num_classes, error)) {return nullptr;}
  return e;
}

}  // namespace nz2_perception
