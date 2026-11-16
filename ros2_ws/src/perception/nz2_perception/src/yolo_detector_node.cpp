// =====================================================================
// yolo_detector_node  (NZ2 인지)
//   역할: 전면/그리퍼 D435 컬러 영상 → YOLO 추론 → 2D 검출 발행
//   입력: /front/color/image_raw        (sensor_msgs/Image)  주행 중 신호등·원거리 퍽
//         /gripper/color/image_raw      (sensor_msgs/Image)  PICK/PLACE 단계
//   출력: /perception/detections        (vision_msgs/Detection2DArray)
//         → traffic_light_filter, puck_localizer 가 구독 (header.frame_id 로 전면/그리퍼 구분)
//
//   [수정 2026-10-24]
//     - 스텁이던 추론을 실제 엔진으로 연결 (include/nz2_perception/yolo_engine.hpp)
//         .engine → TensorRT (Jetson, FP16)   /   .onnx → OpenCV DNN (노트북 CPU 테스트)
//         엔진: bash jetson/build_engine.sh models/current.pt 384 640   (Jetson 에서)
//         ONNX: yolo export model=models/current.pt format=onnx imgsz=384,640 opset=12  (노트북 yolo_env)
//     - 처리 속도 제한: 영상은 최신 1장만 보관 → process_hz 타이머가 카메라마다 1장씩 추론 (11-03 에 바뀜, 아래)
//       (예전: 카메라 2대 × 30fps = 초당 60번 추론 → Orin Nano 과부하)
//     - engine_path 의 ~ 를 홈 폴더로 펼침 (예전 상대경로는 실행 폴더 기준이라 못 찾음)
//     - 엔진 로딩 실패 시 ERROR 로그를 계속 남김 (조용히 빈 결과만 내던 문제)
//
//   [수정 2026-11-03] 타이머 방식 → 영상 도착 즉시 추론 (process_hz 는 상한으로만)
//     - 예전: process_hz 타이머가 돌 때마다 최신 영상 1장. 카메라 30 Hz + 타이머 30 Hz 가 박자가 어긋나
//       새 영상이 없는 차례는 건너뜀 → 30 이 안 나옴 (15 면 정확히 1장 걸러 1장)
//     - 지금: 영상 콜백에서 바로 추론. 직전 처리 후 (1/process_hz × (1 - rate_tolerance)) 가 안 지났으면 건너뜀
//       → 카메라 30 Hz + process_hz 30 이면 프레임 간격이 조금 짧아도(카메라 지터) 버리지 않고 약 30 Hz
//       → process_hz 15 면 약 15 Hz (Orin Nano 부하를 줄이고 싶을 때)
//     - 추론이 프레임 간격보다 오래 걸리면 구독 큐(최신 1장)가 밀린 영상을 버림 → 지연이 쌓이지 않음
//     - 10 s 마다 실제 처리 Hz 를 로그에 같이 표시
// =====================================================================


#include <algorithm>  // std::max
#include <chrono>   // 타이머 주기
#include <cstdio>   // std::snprintf
#include <cstdlib>  // std::getenv (홈 폴더)
#include <memory>   // std::make_shared, std::shared_ptr (스마트 포인터)
#include <string>   // std::string (문자열)
#include <vector>   // std::vector (가변 길이 배열/리스트)

#include <opencv2/opencv.hpp>                       // OpenCV (영상 처리, cv::Mat)
#include <rclcpp/rclcpp.hpp>                         // ROS 2 C++ 클라이언트 (노드/퍼블리셔 등)
#include <sensor_msgs/msg/image.hpp>                 // 카메라 영상 메시지 타입
#include <vision_msgs/msg/detection2_d_array.hpp>    // 2D 검출 결과 메시지 타입

// cv_bridge(ROS 영상 ↔ OpenCV 변환) 헤더 이름이 배포판마다 .hpp/.h 로 달라서 둘 다 대비
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include "nz2_perception/yolo_engine.hpp"            // YOLO 엔진 (TensorRT / OpenCV DNN)

namespace nz2_perception
{

// "~/..." → "/home/<사용자>/..."
static std::string expand_home(const std::string & p)
{
  if (p.rfind("~/", 0) == 0) {
    const char * h = std::getenv("HOME");
    if (h) {return std::string(h) + p.substr(1);}
  }
  return p;
}

// 이 클래스가 곧 ROS 노드다 (rclcpp::Node 를 상속).
class YoloDetectorNode : public rclcpp::Node
{
public:
  YoloDetectorNode()
  : rclcpp::Node("yolo_detector")
  {
    // --- 파라미터 (읽기전용: 시작할 때 한 번만 읽음) ---
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    engine_path_ = expand_home(
      declare_parameter<std::string>("engine_path", "~/nz2_models/current.engine", ro));
    backend_ = declare_parameter<std::string>("backend", "auto", ro);   // auto: 확장자로 결정
    conf_threshold_ = declare_parameter<double>("conf_threshold", 0.25, ro);
    iou_threshold_ = declare_parameter<double>("iou_threshold", 0.45, ro);
    const auto in_h = declare_parameter<int>("input_height", 384, ro);
    const auto in_w = declare_parameter<int>("input_width", 640, ro);
    const double hz = declare_parameter<double>("process_hz", 30.0, ro);   // 카메라별 초당 최대 추론 수 (상한)
    // [2026-11-03] 카메라 지터 허용: 최소 간격 = 1/hz × (1 - rate_tolerance). 0.3 이면 30 Hz 에서 23 ms 이상이면 처리
    const double tol = declare_parameter<double>("rate_tolerance", 0.3, ro);
    enable_front_ = declare_parameter<bool>("enable_front", false, ro);   // [2026-10-28] 전면 카메라 삭제 → 기본 끔 (모든 인지 = 그리퍼)
    enable_gripper_ = declare_parameter<bool>("enable_gripper", true, ro);
    // 디버그 영상: 박스+클래스+점수를 그린 영상 → /perception/<front|gripper>/debug_image (RViz2 Image 로 봄)
    //   그리기·발행 비용이 있으니 Jetson 본선에서는 false 권장
    debug_image_ = declare_parameter<bool>("publish_debug_image", true, ro);
    // 클래스 순서 = 학습 config.yaml 과 동일해야 한다 (tower_light 가 0번)
    class_names_ = declare_parameter<std::vector<std::string>>(
      "class_names",
      {"tower_light",
       "cyl_red_up", "cyl_red_down", "cyl_green_up", "cyl_green_down",
       "box_red_up", "box_red_down", "box_green_up", "box_green_down"},
      ro);

    // --- 검증: 설정이 말이 안 되면 예외를 던져 즉시 종료 ---
    if (conf_threshold_ < 0.0 || conf_threshold_ > 1.0) {
      throw std::invalid_argument("conf_threshold must be in [0, 1]");
    }
    if (iou_threshold_ < 0.0 || iou_threshold_ > 1.0) {
      throw std::invalid_argument("iou_threshold must be in [0, 1]");
    }
    if (in_h <= 0 || in_w <= 0) {throw std::invalid_argument("input_height/width must be positive");}
    if (class_names_.empty()) {throw std::invalid_argument("class_names must not be empty");}
    if (!(0.0 < hz && hz <= 30.0)) {throw std::invalid_argument("process_hz must be in (0, 30]");}
    if (!(0.0 <= tol && tol < 1.0)) {throw std::invalid_argument("rate_tolerance must be in [0, 1)");}
    min_period_s_ = (1.0 / hz) * (1.0 - tol);

    // --- 엔진 로딩. 실패해도 노드는 살려 둔다(다른 인지 노드 시험용) — 대신 ERROR 를 계속 출력 ---
    std::string err;
    engine_ = create_yolo_engine(
      backend_, engine_path_, static_cast<int>(in_h), static_cast<int>(in_w),
      static_cast<int>(class_names_.size()), err);
    if (!engine_) {
      load_error_ = err;
      RCLCPP_ERROR(get_logger(), "YOLO 엔진 로딩 실패 → 검출은 빈 결과로 발행: %s", err.c_str());
    }

    // --- QoS: 영상 BEST_EFFORT(SensorDataQoS), 검출 결과 RELIABLE ---
    // 영상 구독 QoS: 기본 RELIABLE (depth 1). BEST_EFFORT 로 큰 영상을 받으면 DDS 조각 손실로 프레임이 대부분
    //   버려질 수 있음 (노트북 Python 노드에서 15Hz → 1Hz 실측). image_reliable:=false 로 예전 방식
    const bool image_reliable = declare_parameter<bool>("image_reliable", true, ro);
    const rclcpp::QoS sensor_qos = image_reliable ? rclcpp::QoS(rclcpp::KeepLast(1)).reliable()
                                                  : rclcpp::SensorDataQoS();
    const rclcpp::QoS det_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("/perception/detections", det_qos);
    if (debug_image_) {
      // RELIABLE 로 발행해야 RViz2 Image(기본 Reliable)·rqt_image_view 둘 다 붙음
      const auto dbg_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
      // [2026-10-30] 켠 카메라 것만 만듦 (전면 카메라 삭제 → /perception/front/debug_image 안 생김)
      if (enable_front_) {
        dbg_front_pub_ = create_publisher<sensor_msgs::msg::Image>("/perception/front/debug_image", dbg_qos);
      }
      if (enable_gripper_) {
        dbg_gripper_pub_ = create_publisher<sensor_msgs::msg::Image>("/perception/gripper/debug_image", dbg_qos);
      }
    }

    // [2026-11-03] 영상 도착 즉시 추론 (process_hz 는 상한). 큐는 최신 1장 → 추론이 밀리면 옛 영상은 버려짐
    if (enable_front_) {
      front_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/front/color/image_raw", sensor_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_frame(msg, front_rate_, "front");});
    }
    if (enable_gripper_) {
      gripper_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/gripper/color/image_raw", sensor_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_frame(msg, gripper_rate_, "gripper");});
    }

    RCLCPP_INFO(
      get_logger(),
      "yolo_detector 시작: %s '%s', 입력 %dx%d, conf %.2f, iou %.2f, 클래스 %zu개, 영상 도착 즉시 추론 "
      "(상한 %.0f Hz, 최소 간격 %.1f ms), front=%s gripper=%s → /perception/detections",
      engine_ ? engine_->name().c_str() : "(엔진 없음)", engine_path_.c_str(),
      static_cast<int>(in_w), static_cast<int>(in_h), conf_threshold_, iou_threshold_,
      class_names_.size(), hz, min_period_s_ * 1000.0,
      enable_front_ ? "on" : "off", enable_gripper_ ? "on" : "off");
  }

private:
  // 카메라별 처리 속도 기록 (상한 판정 + 실제 Hz 로그)
  struct RateState
  {
    bool started = false;
    std::chrono::steady_clock::time_point last;      // 마지막으로 추론한 시각
    std::chrono::steady_clock::time_point win_start;  // Hz 계산 구간 시작
    int win_count = 0;
    double hz = 0.0;                                  // 최근 구간 실제 처리 Hz
  };

  // [2026-11-03] 영상 콜백: 상한(최소 간격)만 확인하고 바로 추론
  void on_frame(const sensor_msgs::msg::Image::ConstSharedPtr & msg, RateState & r, const char * camera)
  {
    const auto now = std::chrono::steady_clock::now();
    if (r.started && std::chrono::duration<double>(now - r.last).count() < min_period_s_) {
      return;   // 상한보다 빨리 온 영상 → 건너뜀
    }
    if (!r.started) {r.win_start = now;}
    r.started = true;
    r.last = now;
    ++r.win_count;
    const double win = std::chrono::duration<double>(now - r.win_start).count();
    if (win >= 2.0) {
      r.hz = r.win_count / win;
      r.win_count = 0;
      r.win_start = now;
    }
    on_image(msg, camera, r.hz);
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const char * camera, double rate_hz)
  {
    vision_msgs::msg::Detection2DArray out;
    out.header = msg->header;  // 스탬프/frame_id 그대로 → 하위 노드가 카메라 구분
    if (!engine_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "YOLO 엔진 없음: %s", load_error_.c_str());
      det_pub_->publish(out);
      return;
    }
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image;   // ROS Image → OpenCV BGR 영상
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "[%s] 영상 변환 실패: %s", camera, e.what());
      return;
    }
    if (bgr.empty()) {return;}

    const auto t0 = std::chrono::steady_clock::now();
    const auto dets = engine_->infer(bgr, static_cast<float>(conf_threshold_),
      static_cast<float>(iou_threshold_));
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (!engine_->last_error().empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "[%s] 추론 오류: %s", camera,
        engine_->last_error().c_str());
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "[%s] 추론 %.1f ms, 검출 %zu개, 처리 %.1f Hz",
      camera, ms, dets.size(), rate_hz);

    out.detections.reserve(dets.size());
    for (const auto & d : dets) {
      vision_msgs::msg::Detection2D det;
      det.header = msg->header;
      det.bbox.center.position.x = d.box.x + d.box.width * 0.5;
      det.bbox.center.position.y = d.box.y + d.box.height * 0.5;
      det.bbox.size_x = d.box.width;
      det.bbox.size_y = d.box.height;
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id =
        (d.class_id >= 0 && d.class_id < static_cast<int>(class_names_.size()))
        ? class_names_[d.class_id] : std::to_string(d.class_id);
      hyp.hypothesis.score = d.score;
      det.results.push_back(hyp);
      out.detections.push_back(det);
    }
    det_pub_->publish(out);
    if (debug_image_) {publish_debug(msg, bgr, dets, ms, camera);}
  }

  // 박스·클래스·점수를 그린 영상 발행 (색: 경광등=노랑, 빨강 퍽=빨강, 초록 퍽=초록)
  void publish_debug(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg, const cv::Mat & bgr,
    const std::vector<Detection> & dets, double ms, const char * camera)
  {
    auto & pub = std::string(camera) == "front" ? dbg_front_pub_ : dbg_gripper_pub_;
    if (!pub || pub->get_subscription_count() == 0) {return;}   // 보는 사람이 없으면 그리지 않음
    cv::Mat img = bgr.clone();
    for (const auto & d : dets) {
      const std::string name =
        (d.class_id >= 0 && d.class_id < static_cast<int>(class_names_.size())) ? class_names_[d.class_id] : "?";
      const cv::Scalar col = name == "tower_light" ? cv::Scalar(0, 220, 255)
        : name.find("_red") != std::string::npos ? cv::Scalar(40, 40, 255) : cv::Scalar(60, 220, 60);
      cv::rectangle(img, d.box, col, 2);
      char txt[64];
      std::snprintf(txt, sizeof(txt), "%s %.2f", name.c_str(), d.score);
      const cv::Point org(static_cast<int>(d.box.x), std::max(14, static_cast<int>(d.box.y) - 4));
      cv::putText(img, txt, org, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 3);
      cv::putText(img, txt, org, cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1);
    }
    char hud[64];
    std::snprintf(hud, sizeof(hud), "%s  %.0f ms  %zu det", camera, ms, dets.size());
    cv::putText(img, hud, {8, 22}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 3);
    cv::putText(img, hud, {8, 22}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
    pub->publish(*cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg());
  }

  // ----- 멤버 변수 -----
  std::string engine_path_, backend_, load_error_;
  double conf_threshold_ = 0.25;
  double iou_threshold_ = 0.45;
  bool enable_front_ = true;
  bool enable_gripper_ = true;
  bool debug_image_ = true;
  std::vector<std::string> class_names_;

  std::unique_ptr<YoloEngine> engine_;
  double min_period_s_ = 0.0;              // [2026-11-03] 추론 최소 간격 (상한)
  RateState front_rate_, gripper_rate_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dbg_front_pub_, dbg_gripper_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr gripper_sub_;
};

}  // namespace nz2_perception

// 프로그램 시작점. (ros2 run 하면 여기부터 실행)
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<nz2_perception::YoloDetectorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("yolo_detector"), "시작 실패: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
