// =====================================================================
// puck_localizer_node  (NZ2 인지)
//   역할: YOLO 퍽 bbox + 그리퍼 정렬 depth → 퍽 3D pose (base_link)
//   입력: /perception/detections                      (vision_msgs/Detection2DArray)
//         /gripper/aligned_depth_to_color/image_raw    (sensor_msgs/Image, 16UC1 mm 또는 32FC1 m)
//         /gripper/color/camera_info                   (sensor_msgs/CameraInfo)
//         /tf  base_link→gripper_cam                   (tf2)
//   출력: /perception/pucks                            (nz2_interfaces/PuckArray, frame_id base_link)
//   양식: xmen_tracker tracker_node 의 "최신 입력 보관 + 역투영 + 검증" 패턴을 C++로.
//
//   [초안] 자세(up/down)·yaw 는 미계산(NaN/UNKNOWN). bbox 중심 depth 중앙값으로 위치만 추정.
//          실제 구현: ROI median depth, 크기 게이트(지름 3cm/높이 6cm), 자세는 depth 높이/박스비율.
//
//   ※ C++ 기본 문법 설명은 yolo_detector_node.cpp 상단 주석을 먼저 보세요.
//     이 파일은 이 노드에서 처음 나오는 개념(역투영/tf/깊이 중앙값)만 집중 설명합니다.
// =====================================================================
#include <algorithm>   // std::max, std::min, std::nth_element
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>   // 카메라 내부 파라미터(초점거리 등)
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <nz2_interfaces/msg/puck.hpp>        // 우리가 정의한 커스텀 메시지(빌드하면 헤더 생성)
#include <nz2_interfaces/msg/puck_array.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
// tf2_ros 헤더: 최신 배포판(Lyrical 등)은 .hpp, 구버전은 .h 만 있다.
// .h 를 쓰면 'DEPRECATION' 빌드 경고가 나므로, .hpp 가 있으면 그걸 쓰도록 분기한다.
#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>                 // tf2: 좌표계(프레임) 간 변환 보관소
#include <tf2_ros/transform_listener.hpp>     //      TF 를 계속 수신하는 리스너
#else
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#endif
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // PointStamped 등을 tf 로 변환 가능하게 함

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

namespace nz2_perception
{

// 클래스 밖의 '자유 함수'(어디서나 쓰는 도우미). static = 이 파일 안에서만 보이게.
// color, shape 를 '참조(&)'로 받아 함수 안에서 채워 돌려준다(출력 인자). 반환 bool 은 퍽이냐 여부.
// posture 도 같은 방식(출력 인자). 클래스 이름 끝이 _up/_down 이면 자세, 없으면 UNKNOWN(예전 7클래스 모델).
static bool parse_puck_class(
  const std::string & name, uint8_t & color, uint8_t & shape, uint8_t & posture)
{
  using P = nz2_interfaces::msg::Puck;   // using = 긴 이름에 짧은 별명 P 부여
  // name.rfind("cyl_", 0) == 0 : 문자열이 "cyl_" 로 '시작'하는지 검사하는 관용구.
  if (name.rfind("cyl_", 0) == 0) {shape = P::SHAPE_CYLINDER;}   // P::SHAPE_... = msg 안 상수
  else if (name.rfind("box_", 0) == 0) {shape = P::SHAPE_BOX;}
  else {return false;}  // tower_light 등 → 퍽이 아님
  // find(...) != npos : 부분 문자열이 들어있는지 (npos = "못 찾음" 표식)
  if (name.find("_red") != std::string::npos) {color = P::COLOR_RED;}
  else if (name.find("_green") != std::string::npos) {color = P::COLOR_GREEN;}
  else if (name.find("_blue") != std::string::npos) {color = P::COLOR_BLUE;}
  else {color = P::COLOR_UNKNOWN;}
  // ends_with : 문자열이 s 로 '끝나는지' 검사하는 작은 람다(이름 없는 함수)
  auto ends_with = [&name](const std::string & s) {
      return name.size() >= s.size() && name.compare(name.size() - s.size(), s.size(), s) == 0;
    };
  if (ends_with("_up")) {posture = P::POSTURE_UP;}            // 서 있음
  else if (ends_with("_down")) {posture = P::POSTURE_DOWN;}   // 누움
  else {posture = P::POSTURE_UNKNOWN;}
  return true;
}

class PuckLocalizerNode : public rclcpp::Node
{
public:
  PuckLocalizerNode()
  : rclcpp::Node("puck_localizer")
  {
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link", ro);
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.10, ro);   // 이보다 가까운 depth 는 버림
    max_depth_m_ = declare_parameter<double>("max_depth_m", 1.50, ro);   // 이보다 먼 depth 는 버림
    roi_ratio_ = declare_parameter<double>("roi_ratio", 0.4, ro);        // bbox 중앙 ROI 비율
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1, ro);

    if (!(0.0 < min_depth_m_ && min_depth_m_ < max_depth_m_)) {
      throw std::invalid_argument("min_depth_m must be >0 and < max_depth_m");
    }
    if (!(0.0 < roi_ratio_ && roi_ratio_ <= 1.0)) {
      throw std::invalid_argument("roi_ratio must be in (0, 1]");
    }

    // tf2 준비: Buffer(변환 저장소) + Listener(계속 /tf 를 받아 Buffer 에 채움).
    // make_unique = 단독 소유 스마트 포인터(이 노드만 소유).
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    // [수정 2026-10-24] camera_info 는 SensorDataQoS(best_effort, volatile)로 받는다.
    //   예전 transient_local 구독은 RealSense 의 volatile 발행과 DDS 호환이 안 돼서 메시지가 0개 들어왔다.
    //   (best_effort·volatile 구독은 어떤 발행 QoS 와도 연결된다)
    const auto info_qos = rclcpp::SensorDataQoS();
    const rclcpp::QoS pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    pucks_pub_ = create_publisher<nz2_interfaces::msg::PuckArray>("/perception/pucks", pub_qos);

    // depth 와 camera_info 는 '최신 것만 보관'해 두었다가, 검출이 올 때 꺼내 쓴다(아래 콜백).
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/gripper/aligned_depth_to_color/image_raw", sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {latest_depth_ = m;});
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/gripper/color/camera_info", info_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {latest_info_ = m;});
    // 검출이 올 때마다 on_detections 실행. std::bind 로 멤버함수를 콜백으로 등록(_1 = 첫 인자 자리).
    det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/perception/detections", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&PuckLocalizerNode::on_detections, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "puck_localizer 시작: base=%s, depth %.2f~%.2f m, ROI %.0f%% → /perception/pucks",
      base_frame_.c_str(), min_depth_m_, max_depth_m_, roi_ratio_ * 100.0);
  }

private:
  void on_detections(const vision_msgs::msg::Detection2DArray::ConstSharedPtr & dets)
  {
    nz2_interfaces::msg::PuckArray out;
    out.header.stamp = dets->header.stamp;
    out.header.frame_id = base_frame_;

    // 그리퍼 카메라 검출만 측위 (frame_id 로 전면/그리퍼 구분). 전면 검출은 신호등용이라 무시.
    const bool is_gripper = dets->header.frame_id.find("gripper") != std::string::npos;
    // 전면 카메라 검출이면 발행 자체를 안 한다. 여기서 빈 목록을 내보내면 perception_master 의
    // 퍽 목록이 그리퍼 결과 ↔ 빈 목록으로 계속 깜빡이기 때문(두 카메라가 같은 토픽을 씀).
    if (!is_gripper) {return;}
    if (!latest_depth_ || !latest_info_) {   // !latest_depth_ = 아직 depth 안 들어옴
      pucks_pub_->publish(out);  // 빈 목록이라도 주기적으로 발행
      return;
    }

    cv::Mat depth;
    try {
      depth = cv_bridge::toCvShare(latest_depth_)->image;   // depth 영상 → OpenCV 행렬
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "depth 변환 실패: %s", e.what());
      pucks_pub_->publish(out);
      return;
    }
    // 카메라 내부 파라미터: k 는 3x3 행렬을 1줄(9개)로 편 것. fx,fy=초점거리, cx,cy=영상 중심.
    const double fx = latest_info_->k[0], fy = latest_info_->k[4];
    const double cx = latest_info_->k[2], cy = latest_info_->k[5];
    if (fx <= 0 || fy <= 0) {
      pucks_pub_->publish(out);
      return;
    }
    const std::string cam_frame =
      !dets->header.frame_id.empty() ? dets->header.frame_id : latest_info_->header.frame_id;

    for (const auto & d : dets->detections) {   // 검출마다 반복
      if (d.results.empty()) {continue;}         // continue = 이번 것 건너뛰고 다음으로
      uint8_t color, shape, posture;
      if (!parse_puck_class(d.results[0].hypothesis.class_id, color, shape, posture)) {continue;}  // 퍽 아니면 skip

      const double z = sample_depth(depth, d.bbox);   // 박스 중앙의 거리(m) 추정
      if (!(min_depth_m_ <= z && z <= max_depth_m_)) {continue;}  // depth 없음/범위 밖이면 skip

      // 핀홀(바늘구멍) 카메라 역투영: 픽셀(u,v)+거리 z → 카메라 기준 3D 좌표(X,Y,Z).
      //   X = (u - cx)/fx * z,  Y = (v - cy)/fy * z,  Z = z   (카메라 광학 좌표계)
      geometry_msgs::msg::PointStamped p_cam;
      p_cam.header.frame_id = cam_frame;        // 이 좌표가 "어느 프레임 기준"인지 꼭 적어야 tf 가능
      p_cam.header.stamp = dets->header.stamp;
      p_cam.point.x = (d.bbox.center.position.x - cx) / fx * z;
      p_cam.point.y = (d.bbox.center.position.y - cy) / fy * z;
      p_cam.point.z = z;

      nz2_interfaces::msg::Puck puck;
      puck.header = out.header;
      puck.color = color;
      puck.shape = shape;
      puck.posture = posture;   // YOLO 클래스 이름의 _up/_down (없으면 UNKNOWN)
      puck.yaw = std::numeric_limits<float>::quiet_NaN();          // TODO: 접근각 (NaN = "값 없음")
      puck.confidence = d.results[0].hypothesis.score;
      puck.track_id = -1;                                          // TODO: 프레임 간 추적

      // 카메라 좌표 → base_link(차체) 좌표로 변환. tf 가 아직 없으면 예외 → 카메라 좌표 그대로 발행.
      try {
        auto p_base = tf_buffer_->transform(
          p_cam, base_frame_, tf2::durationFromSec(tf_timeout_));
        puck.position = p_base.point;
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "tf %s→%s 실패: %s (카메라 프레임 좌표로 발행)",
          cam_frame.c_str(), base_frame_.c_str(), e.what());
        // [수정 2026-10-24] 좌표가 카메라 기준이면 frame_id 도 카메라로 적는다.
        //   예전엔 header 가 base_link 인 채로 카메라 좌표가 들어가서, 받는 쪽이 차체 좌표로 오해했다.
        puck.position = p_cam.point;
        puck.header.frame_id = cam_frame;
      }
      out.pucks.push_back(puck);
    }
    pucks_pub_->publish(out);
  }

  // bbox 중앙 ROI 에서 유효 depth 의 '중앙값(median)'을 구한다(m). 값 없으면 -1.
  //   중앙값을 쓰는 이유: 박스 안에 배경/노이즈가 섞여도 가운데 대표값이 안정적이라서.
  double sample_depth(const cv::Mat & depth, const vision_msgs::msg::BoundingBox2D & box) const
  {
    const int w = depth.cols, h = depth.rows;        // cols=가로픽셀수, rows=세로
    const double rw = box.size_x * roi_ratio_ * 0.5; // ROI 반폭
    const double rh = box.size_y * roi_ratio_ * 0.5; // ROI 반높이
    // ROI 범위를 영상 안쪽으로 자르기(clamp). max/min 으로 0~(w-1), 0~(h-1) 벗어나지 않게.
    const int x0 = std::max(0, static_cast<int>(box.center.position.x - rw));
    const int x1 = std::min(w - 1, static_cast<int>(box.center.position.x + rw));
    const int y0 = std::max(0, static_cast<int>(box.center.position.y - rh));
    const int y1 = std::min(h - 1, static_cast<int>(box.center.position.y + rh));
    if (x1 < x0 || y1 < y0) {return -1.0;}

    std::vector<double> vals;
    vals.reserve((x1 - x0 + 1) * (y1 - y0 + 1));
    const bool is_u16 = (depth.type() == CV_16UC1);   // depth 가 16비트 정수(mm)인지 32비트 실수(m)인지
    for (int y = y0; y <= y1; ++y) {
      for (int x = x0; x <= x1; ++x) {
        double m;
        // depth.at<타입>(y, x) : (y행, x열) 픽셀 값 읽기. 16UC1 은 mm 라 0.001 곱해 m 로.
        if (is_u16) {m = depth.at<uint16_t>(y, x) * 0.001;}       // mm → m
        else if (depth.type() == CV_32FC1) {m = depth.at<float>(y, x);}  // 이미 m
        else {return -1.0;}
        if (std::isfinite(m) && m > 0.0) {vals.push_back(m);}     // 유효한 값만 수집
      }
    }
    if (vals.empty()) {return -1.0;}
    // nth_element : 전체 정렬 없이 "가운데 번째 값"만 제자리에 놓는 빠른 방법 → 중앙값.
    std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
    return vals[vals.size() / 2];
  }

  // ----- 멤버 변수 -----
  std::string base_frame_;
  double min_depth_m_, max_depth_m_, roi_ratio_, tf_timeout_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                 // 단독 소유
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;    // 공유 소유
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;       // 최신 depth 영상 보관
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;   // 최신 camera_info 보관

  rclcpp::Publisher<nz2_interfaces::msg::PuckArray>::SharedPtr pucks_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
};

}  // namespace nz2_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<nz2_perception::PuckLocalizerNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("puck_localizer"), "시작 실패: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
