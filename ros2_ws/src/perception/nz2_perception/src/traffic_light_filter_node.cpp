// =====================================================================
// traffic_light_filter_node  (NZ2 인지)
//   역할: YOLO 경광등(tower_light) 박스 + 그리퍼 영상 → 켜진 칸(빨/노/초) 판정 → 확정
//   입력: /perception/detections    (vision_msgs/Detection2DArray)  YOLO 결과
//         image_topic               (sensor_msgs/Image, 기본 /gripper/color/image_raw)  렌즈 밝기 판정용
//   출력: /perception/traffic_light (nz2_interfaces/TrafficLight)
//         → perception_master, signal_recorder (12회 점멸 기록)
//
//   [수정 2026-10-24] 모델이 tower_light 1클래스라, 박스를 세로 3칸(위=빨, 가운데=노, 아래=초)으로 나눠 밝기로 판정.
//   [수정 2026-10-28] 미션 v5 + 전면 카메라 삭제(그리퍼 카메라가 신호등도 봄)
//     - 그리퍼 프레임을 버리던 줄 삭제 (예전 L104). 이제 들어오는 검출은 전부 그리퍼 카메라
//     - "꺼짐"(STATE_OFF: 경광등은 보이는데 세 칸 다 꺼짐)과 "안 보임"(STATE_UNKNOWN)을 구분
//       → 팔이 흔들려 경광등이 화면에서 빠진 순간을 "꺼짐"으로 세지 않게
//     - 미션: 1회 점멸 = 켜짐 1 s + 꺼짐 1 s. 예전 창(8프레임/2회)+확정 3프레임은 꺼짐 확정까지 약 1 s 걸려
//       같은 색 연속 점멸(R R)이 1번으로 합쳐질 수 있었음 → 기본값을 창 3 / 1회, 확정 2 로 줄임
//     - 경광등이 안 보이면 밝기 창(history_)을 비움 (다시 보일 때 예전 켜짐이 남지 않게)
//     - 영상 구독 QoS 기본 RELIABLE (BEST_EFFORT 는 큰 영상이 조각 손실로 버려져 같은 시각 영상을 못 찾음)
//   판정 순서
//     1) 칸마다 V(밝기) > v_threshold 픽셀 비율 >= on_ratio 면 이번 프레임 "켜짐"
//     2) 최근 window_frames 프레임 중 window_min_on 번 이상 켜지면 그 칸 켜짐 (LED 깜빡임 대응)
//     3) 여러 칸이 켜지면 빨강 > 노랑 > 초록 우선. 다 꺼지면 OFF
//     4) raw_state 가 confirm_frames 번 연속이어야 state 확정
//   - 그리퍼 카메라는 노출 고정 필수 (자동 노출이면 켜진 램프가 하얗게 날아가거나 v_threshold 가 안 맞음)
//
//   ※ C++ 문법 기초는 yolo_detector_node.cpp 맨 위 주석을 먼저 보세요.
// =====================================================================
#include <array>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <nz2_interfaces/msg/traffic_light.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace nz2_perception
{

class TrafficLightFilterNode : public rclcpp::Node
{
public:
  TrafficLightFilterNode()
  : rclcpp::Node("traffic_light_filter")
  {
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    confirm_frames_ = static_cast<int>(declare_parameter<int>("confirm_frames", 2, ro));
    min_score_ = declare_parameter<double>("min_score", 0.4, ro);
    const auto image_topic = declare_parameter<std::string>("image_topic", "/gripper/color/image_raw", ro);
    const bool image_reliable = declare_parameter<bool>("image_reliable", true, ro);
    tower_class_ = declare_parameter<std::string>("tower_class", "tower_light", ro);
    v_thr_ = static_cast<int>(declare_parameter<int>("v_threshold", 200, ro));
    on_ratio_ = declare_parameter<double>("on_ratio", 0.15, ro);
    window_ = static_cast<size_t>(declare_parameter<int>("window_frames", 3, ro));
    window_min_on_ = static_cast<int>(declare_parameter<int>("window_min_on", 1, ro));
    match_tol_ = declare_parameter<double>("match_tolerance_s", 0.005, ro);
    // (선택) 색별 클래스를 내는 모델용: 이름 → 상태
    const auto keys = declare_parameter<std::vector<std::string>>(
      "state_class_names", {"light_red", "light_yellow", "light_green"}, ro);
    using T = nz2_interfaces::msg::TrafficLight;
    const std::vector<uint8_t> vals = {T::STATE_RED, T::STATE_YELLOW, T::STATE_GREEN};
    for (size_t i = 0; i < keys.size() && i < vals.size(); ++i) {name_to_state_[keys[i]] = vals[i];}

    if (confirm_frames_ < 1) {throw std::invalid_argument("confirm_frames must be >= 1");}
    if (min_score_ < 0.0 || min_score_ > 1.0) {throw std::invalid_argument("min_score must be in [0,1]");}
    if (v_thr_ < 0 || v_thr_ > 255) {throw std::invalid_argument("v_threshold must be in [0,255]");}
    if (!(0.0 < on_ratio_ && on_ratio_ < 1.0)) {throw std::invalid_argument("on_ratio must be in (0,1)");}
    if (window_ < 1 || window_min_on_ < 1 || window_min_on_ > static_cast<int>(window_)) {
      throw std::invalid_argument("1 <= window_min_on <= window_frames");
    }
    if (!(0.0 < match_tol_ && match_tol_ < 0.1)) {throw std::invalid_argument("match_tolerance_s must be in (0,0.1)");}

    pub_ = create_publisher<nz2_interfaces::msg::TrafficLight>(
      "/perception/traffic_light", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    // 검출과 같은 시각의 영상을 찾으려고 최근 몇 장을 보관
    auto img_qos = rclcpp::QoS(rclcpp::KeepLast(5));
    if (image_reliable) {img_qos.reliable();} else {img_qos.best_effort();}
    img_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, img_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {
        frames_.push_back(m);
        while (frames_.size() > 8) {frames_.pop_front();}
        // 영상보다 검출이 먼저 도착해 보류된 게 있으면 지금 처리
        if (pending_ && same_time(pending_->header.stamp, m->header.stamp)) {
          auto d = pending_;
          pending_.reset();
          process(d, m);
        }
      });
    sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/perception/detections", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&TrafficLightFilterNode::on_detections, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "traffic_light_filter 시작: %s(%s) + '%s' 박스, V>%d 비율>=%.2f, 창 %zu/%d, %d프레임 확정 "
      "→ /perception/traffic_light", image_topic.c_str(), image_reliable ? "RELIABLE" : "BEST_EFFORT",
      tower_class_.c_str(), v_thr_, on_ratio_, window_, window_min_on_, confirm_frames_);
  }

private:
  bool same_time(const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b) const
  {
    return std::abs((rclcpp::Time(a) - rclcpp::Time(b)).seconds()) < match_tol_;
  }

  void on_detections(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & dets)
  {
    // [2026-10-28] 전면 카메라 삭제 → 그리퍼 프레임도 처리 (예전엔 여기서 그리퍼 프레임을 버렸음)
    // 같은 시각 영상 찾기. 아직 안 왔으면 보류했다가 영상 콜백에서 처리
    sensor_msgs::msg::Image::ConstSharedPtr img;
    for (const auto & f : frames_) {
      if (same_time(f->header.stamp, dets->header.stamp)) {img = f;}
    }
    if (!img && has_tower(*dets)) {
      if (pending_) {   // 이전 보류분은 영상이 끝내 안 왔으니 영상 없이 처리 (→ 안 보임으로 기록)
        auto old = pending_;
        pending_.reset();
        process(old, nullptr);
      }
      pending_ = dets;
      return;
    }
    process(dets, img);
  }

  bool has_tower(const vision_msgs::msg::Detection2DArray & dets) const
  {
    for (const auto & d : dets.detections) {
      if (!d.results.empty() && d.results[0].hypothesis.class_id == tower_class_) {return true;}
    }
    return false;
  }

  void process(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & dets, const sensor_msgs::msg::Image::ConstSharedPtr & img)
  {
    using T = nz2_interfaces::msg::TrafficLight;

    uint8_t raw = T::STATE_UNKNOWN;
    bool visible = false;
    float conf = 0.0f;
    const vision_msgs::msg::Detection2D * tower = nullptr;
    float best_named = static_cast<float>(min_score_), best_tower = static_cast<float>(min_score_);
    for (const auto & d : dets->detections) {
      if (d.results.empty()) {continue;}
      const auto & name = d.results[0].hypothesis.class_id;
      const float score = static_cast<float>(d.results[0].hypothesis.score);
      auto it = name_to_state_.find(name);
      if (it != name_to_state_.end() && score >= best_named) {   // 색별 클래스 모델
        best_named = score;
        raw = it->second;
        conf = score;
        visible = true;
      }
      if (name == tower_class_ && score >= best_tower) {best_tower = score; tower = &d;}
    }
    // 색별 클래스가 없으면 tower_light 박스 밝기로 판정
    if (raw == T::STATE_UNKNOWN && tower) {raw = judge_lamp(*tower, img, conf, visible);}
    if (!visible) {history_.clear();}   // 안 보이는 동안의 기록은 버림 (다시 보일 때 예전 켜짐이 남지 않게)

    if (raw == candidate_) {
      if (count_ < 255) {++count_;}
    } else {
      candidate_ = raw;
      count_ = 1;
    }
    if (count_ >= confirm_frames_) {confirmed_ = candidate_; last_score_ = conf;}

    T msg;
    msg.header = dets->header;
    msg.state = confirmed_;
    msg.confidence = last_score_;
    msg.confirmed_frames = static_cast<uint8_t>(std::min<int>(count_, 255));
    msg.visible = visible;
    msg.raw_state = raw;
    pub_->publish(msg);
  }

  // tower_light 박스 → 켜진 칸 (창 + 우선순위 적용).
  //   같은 시각 영상이 없거나 박스가 너무 작으면 UNKNOWN(visible=false), 세 칸 다 꺼지면 OFF(visible=true)
  uint8_t judge_lamp(
    const vision_msgs::msg::Detection2D & d, const sensor_msgs::msg::Image::ConstSharedPtr & img, float & conf,
    bool & visible)
  {
    using T = nz2_interfaces::msg::TrafficLight;
    visible = false;
    if (!img) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "검출과 같은 시각의 영상 없음 (image_topic·QoS 확인)");
      return T::STATE_UNKNOWN;
    }
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvShare(img, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "영상 변환 실패: %s", e.what());
      return T::STATE_UNKNOWN;
    }
    // 박스 가로 가운데 60% (테두리·반사 제외)
    const double w = d.bbox.size_x, h = d.bbox.size_y;
    cv::Rect r(
      static_cast<int>(d.bbox.center.position.x - w * 0.3), static_cast<int>(d.bbox.center.position.y - h / 2),
      static_cast<int>(w * 0.6), static_cast<int>(h));
    r &= cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (r.width < 3 || r.height < 9) {return T::STATE_UNKNOWN;}
    visible = true;

    cv::Mat hsv;
    cv::cvtColor(bgr(r), hsv, cv::COLOR_BGR2HSV);
    std::array<bool, 3> on{};
    std::array<double, 3> ratio{};
    const int h3 = hsv.rows / 3;
    for (int i = 0; i < 3; ++i) {
      std::vector<cv::Mat> ch;
      cv::split(hsv(cv::Rect(0, i * h3, hsv.cols, h3)), ch);
      ratio[i] = static_cast<double>(cv::countNonZero(ch[2] > v_thr_)) / std::max(1, ch[2].rows * ch[2].cols);
      on[i] = ratio[i] >= on_ratio_;
    }
    history_.push_back(on);
    while (history_.size() > window_) {history_.pop_front();}
    std::array<int, 3> cnt{};
    for (const auto & hh : history_) {for (int i = 0; i < 3; ++i) {cnt[i] += hh[i] ? 1 : 0;}}

    const uint8_t states[3] = {T::STATE_RED, T::STATE_YELLOW, T::STATE_GREEN};
    for (int i = 0; i < 3; ++i) {     // 위(빨강)부터 → 빨강 우선
      if (cnt[i] >= window_min_on_) {conf = static_cast<float>(ratio[i]); return states[i];}
    }
    conf = 1.0f;
    return T::STATE_OFF;              // 보이는데 전부 꺼짐
  }

  int confirm_frames_ = 2;
  double min_score_ = 0.4, on_ratio_ = 0.15, match_tol_ = 0.005;
  int v_thr_ = 200, window_min_on_ = 1;
  size_t window_ = 3;
  std::string tower_class_;
  std::map<std::string, uint8_t> name_to_state_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> frames_;
  std::deque<std::array<bool, 3>> history_;
  vision_msgs::msg::Detection2DArray::ConstSharedPtr pending_;   // 영상을 기다리는 검출

  uint8_t candidate_ = 0;
  uint8_t confirmed_ = 0;
  int count_ = 0;
  float last_score_ = 0.0f;

  rclcpp::Publisher<nz2_interfaces::msg::TrafficLight>::SharedPtr pub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

}  // namespace nz2_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<nz2_perception::TrafficLightFilterNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("traffic_light_filter"), "시작 실패: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
