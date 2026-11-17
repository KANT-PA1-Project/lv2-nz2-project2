// =====================================================================
// tag_reader_node  (NZ2 인지)
//   역할: 그리퍼 카메라 영상 → AprilTag(tag36h11) ID + 3D 자세 → /nz2/tags
//   입력: image_topic (sensor_msgs/Image, 기본 /gripper/color/image_raw)
//         info_topic  (sensor_msgs/CameraInfo, 기본 /gripper/color/camera_info)
//         /tf (output_frame 을 지정한 경우)
//   출력: /nz2/tags (vision_msgs/Detection3DArray) — 팀 레포 토픽 이름(/nz2/tags)에 맞춤
//         Detection3D 1개 = 태그 1개
//           id                              : 태그 ID 숫자 문자열 ("5")
//           results[0].hypothesis.class_id  : "tag36h11"
//           results[0].hypothesis.score     : 1.0 = 자세까지 계산 / 0.5 = ID 만 (너무 작거나 camera_info 없음)
//           results[0].pose.pose, bbox.center : 태그 중심 자세 (z = 태그 면에서 카메라 쪽 법선, OpenCV ArUco 규약)
//           bbox.size.x / y                 : 태그 검은 사각형 한 변 (m), bbox.size.z = 영상에서 한 변 픽셀 수
//           header.frame_id                 : output_frame (비우면 카메라 광학 프레임)
//   태그 크기 (미션 v5 / 2026-10-28 확정): 퍽 ID 0~11 = 16 mm, 장소 L1~L3 = ID 12~14 = 48 mm
//
//   [2026-10-28] 팀 정렬판(nz2_perception, 클라우드에만 있던 것)의 tag_reader 를 이 패키지로 가져옴
//     - 전면 카메라 삭제 → 기본 토픽 그리퍼, 노드 1개
//     - 영상 구독 RELIABLE 기본 (image_reliable)
//     - QR 은 끔 (미션 바코드는 barcode_reader 가 Code 128 로 처리)
//     - 영상 나이 검사(max_image_age_s)는 0 이면 끔 (rosbag 재생 시험용)
//   픽셀 기준: 16 mm 태그는 한 변 20 px 이상이어야 ID 가 안정적 → 848x480 기준 약 0.5 m 이내
//
//   ※ C++ 문법 기초는 yolo_detector_node.cpp 맨 위 주석을 먼저 보세요.
// =====================================================================
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#endif
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#if __has_include(<tf2/LinearMath/Matrix3x3.hpp>)
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#else
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#endif

#include "nz2_perception/tag_detector.hpp"

namespace nz2_perception
{

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

class TagReaderNode : public rclcpp::Node
{
public:
  TagReaderNode()
  : rclcpp::Node("tag_reader")
  {
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    const auto image_topic = declare_parameter<std::string>("image_topic", "/gripper/color/image_raw", ro);
    const auto info_topic = declare_parameter<std::string>("info_topic", "/gripper/color/camera_info", ro);
    const auto out_topic = declare_parameter<std::string>("output_topic", "/nz2/tags", ro);
    const bool image_reliable = declare_parameter<bool>("image_reliable", true, ro);
    const double hz = declare_parameter<double>("process_hz", 10.0, ro);
    tag_size_ = declare_parameter<double>("tag_size_m", 0.016, ro);
    large_ids_ = declare_parameter<std::vector<int64_t>>("large_tag_ids", {12, 13, 14}, ro);
    large_size_ = declare_parameter<double>("large_tag_size_m", 0.048, ro);
    min_tag_px_ = declare_parameter<double>("min_tag_px", 10.0, ro);
    max_id_ = static_cast<int>(declare_parameter<int>("max_id", 14, ro));
    output_frame_ = declare_parameter<std::string>("output_frame", "", ro);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.05, ro);
    max_image_age_ = declare_parameter<double>("max_image_age_s", 0.5, ro);

    if (hz <= 0 || hz > 30) {throw std::invalid_argument("process_hz 는 (0, 30]");}
    if (tag_size_ <= 0 || large_size_ <= 0) {throw std::invalid_argument("태그 크기 > 0");}
    if (!tags_.available()) {
      throw std::runtime_error("이 OpenCV 에 ArUco(AprilTag) 모듈이 없음 → OpenCV 4.7+ 또는 opencv-contrib 필요");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto img_qos = rclcpp::QoS(rclcpp::KeepLast(2));
    if (image_reliable) {img_qos.reliable();} else {img_qos.best_effort();}
    image_sub_ = create_subscription<Image>(image_topic, img_qos, [this](Image::ConstSharedPtr m) {latest_ = m;});
    info_sub_ = create_subscription<CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(), [this](CameraInfo::ConstSharedPtr m) {info_ = m;});
    pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(out_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), std::bind(&TagReaderNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "tag_reader 시작: %s(%s), %.1f Hz, 태그 %.0f mm (장소 ID 12~14 %.0f mm), ID 0~%d → %s",
      image_topic.c_str(), image_reliable ? "RELIABLE" : "BEST_EFFORT", hz, tag_size_ * 1000, large_size_ * 1000,
      max_id_, out_topic.c_str());
  }

private:
  double size_of(int id) const
  {
    for (auto i : large_ids_) {if (i == id) {return large_size_;}}
    return tag_size_;
  }

  void on_timer()
  {
    const auto msg = latest_;
    if (!msg || msg == last_) {return;}
    last_ = msg;
    if (max_image_age_ > 0 && (now() - rclcpp::Time(msg->header.stamp)).seconds() > max_image_age_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "영상이 %.1f s 이상 지연 → 건너뜀 (rosbag 재생이면 max_image_age_s:=0)",
        max_image_age_);
      return;
    }
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "영상 변환 실패: %s", e.what());
      return;
    }
    if (bgr.empty()) {return;}
    if (!info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera_info 없음 → ID 만 발행 (자세 없음)");
    }

    vision_msgs::msg::Detection3DArray out;
    out.header = msg->header;
    const bool use_out_frame = !output_frame_.empty();
    if (use_out_frame) {out.header.frame_id = output_frame_;}

    cv::Matx33d K;
    std::vector<double> dist;
    const bool have_k = info_ && info_->k[0] > 0 && info_->k[4] > 0;
    if (have_k) {
      K = cv::Matx33d(info_->k[0], 0, info_->k[2], 0, info_->k[4], info_->k[5], 0, 0, 1);
      dist.assign(info_->d.begin(), info_->d.end());
    }

    for (const auto & t : tags_.detect(bgr)) {
      if (t.id < 0 || t.id > max_id_) {continue;}   // 대회에서 안 쓰는 ID = 오검출이나 다른 태그 → 버림
      vision_msgs::msg::Detection3D d;
      d.header = out.header;
      d.id = std::to_string(t.id);
      vision_msgs::msg::ObjectHypothesisWithPose h;
      h.hypothesis.class_id = "tag36h11";
      h.hypothesis.score = 1.0;
      const double s = size_of(t.id);
      cv::Vec3d rv, tv;
      if (have_k && t.side_px() >= min_tag_px_ && TagDetector::estimate_pose(t, s, K, dist, rv, tv)) {
        cv::Matx33d R;
        cv::Rodrigues(rv, R);
        tf2::Matrix3x3 m(R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));
        tf2::Quaternion q;
        m.getRotation(q);
        geometry_msgs::msg::PoseStamped cam, outp;
        cam.header = msg->header;
        cam.pose.position.x = tv[0];
        cam.pose.position.y = tv[1];
        cam.pose.position.z = tv[2];
        cam.pose.orientation.x = q.x();
        cam.pose.orientation.y = q.y();
        cam.pose.orientation.z = q.z();
        cam.pose.orientation.w = q.w();
        if (use_out_frame) {
          try {
            outp = tf_buffer_->transform(cam, output_frame_, tf2::durationFromSec(tf_timeout_));
          } catch (const tf2::TransformException & e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "tf→%s 실패, 이 태그 생략: %s",
              output_frame_.c_str(), e.what());
            continue;
          }
        }
        h.pose.pose = use_out_frame ? outp.pose : cam.pose;
      } else {
        h.pose.pose.orientation.w = 1.0;   // 자세 모름 (너무 작거나 camera_info 없음) → ID 만
        h.hypothesis.score = 0.5;
      }
      d.results.push_back(h);
      d.bbox.center = h.pose.pose;
      d.bbox.size.x = s;
      d.bbox.size.y = s;
      d.bbox.size.z = t.side_px();   // 영상에서 태그 한 변 픽셀 수 (판독 거리 확인용)
      out.detections.push_back(d);
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "태그 %zu개", out.detections.size());
    pub_->publish(out);
  }

  double tag_size_, large_size_, min_tag_px_, tf_timeout_, max_image_age_;
  int max_id_ = 14;
  std::vector<int64_t> large_ids_;
  std::string output_frame_;
  TagDetector tags_;
  Image::ConstSharedPtr latest_, last_;
  CameraInfo::ConstSharedPtr info_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nz2_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<nz2_perception::TagReaderNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("tag_reader"), "시작 실패: %s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
