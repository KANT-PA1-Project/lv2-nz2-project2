// =====================================================================
// obstacle_detector_node  (NZ2 인지)
//   역할: 전면 depth → 3D 점군 → base_link 로 변환 → 바닥·높은 곳 제거 → 장애물 점군
//   입력: depth_topic  (기본 /front/depth/image_rect_raw, sensor_msgs/Image 16UC1 mm 또는 32FC1 m)
//         info_topic   (기본 /front/depth/camera_info, sensor_msgs/CameraInfo)
//         /tf          base_link ← depth 광학 프레임
//   출력: /perception/obstacles  (sensor_msgs/PointCloud2, frame_id = base_frame)
//         → perception_master, planner_server(costmap), collision_monitor
//
//   [수정 2026-10-24]
//     - camera_info 구독 QoS: transient_local → SensorDataQoS (RealSense 와 호환. 예전엔 0개 수신)
//     - 바닥 제거 구현: 점을 base_link 로 바꾼 뒤 높이 z 가 floor_margin_m 이하(바닥)·max_height_m 이상은 버림
//       tf 가 없으면 바닥을 못 거르므로 아예 발행하지 않는다 (바닥 전체가 장애물로 나가 로봇이 못 움직이는 것 방지)
//     - 처리 주기 제한 process_hz (매 depth 프레임 처리 → CPU 과부하 방지)
//
//   ※ C++ 문법 기초는 yolo_detector_node.cpp 맨 위 주석을 먼저 보세요.
//   ※ 핵심 수식(핀홀 역투영): 픽셀(x,y)과 거리 z → X=(x-cx)/fx*z, Y=(y-cy)/fy*z, Z=z (카메라 광학 좌표)
// =====================================================================
#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   // tf2::fromMsg (Transform 메시지 → tf2::Transform)

namespace nz2_perception
{

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode()
  : rclcpp::Node("obstacle_detector")
  {
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    const auto depth_topic = declare_parameter<std::string>("depth_topic", "/front/depth/image_rect_raw", ro);
    const auto info_topic = declare_parameter<std::string>("info_topic", "/front/depth/camera_info", ro);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link", ro);
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.15, ro);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 3.0, ro);
    step_ = static_cast<int>(declare_parameter<int>("pixel_step", 4, ro));   // 다운샘플 간격 (4 → 점 수 1/16)
    floor_margin_m_ = declare_parameter<double>("floor_margin_m", 0.03, ro);  // base z 이 값 이하 = 바닥
    max_height_m_ = declare_parameter<double>("max_height_m", 0.50, ro);      // base z 이 값 이상 = 로봇과 무관
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.05, ro);
    const double hz = declare_parameter<double>("process_hz", 10.0, ro);

    if (!(0.0 < min_depth_m_ && min_depth_m_ < max_depth_m_)) {
      throw std::invalid_argument("min_depth_m must be >0 and < max_depth_m");
    }
    if (step_ < 1) {throw std::invalid_argument("pixel_step must be >= 1");}
    if (!(floor_margin_m_ < max_height_m_)) {throw std::invalid_argument("floor_margin_m < max_height_m");}
    if (!(0.0 < hz && hz <= 30.0)) {throw std::invalid_argument("process_hz must be in (0, 30]");}

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/perception/obstacles", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(),   // [수정] transient_local 은 RealSense camera_info 와 호환 안 됨
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {latest_info_ = m;});
    // depth 는 최신 1장만 보관, 타이머가 process_hz 로 처리
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {latest_depth_ = m;});
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / hz), std::bind(&ObstacleDetectorNode::process, this));

    RCLCPP_INFO(
      get_logger(), "obstacle_detector 시작: %s, depth %.2f~%.2f m, step %d, 높이 %.2f~%.2f m (%s), %.0f Hz "
      "→ /perception/obstacles", depth_topic.c_str(), min_depth_m_, max_depth_m_, step_, floor_margin_m_,
      max_height_m_, base_frame_.c_str(), hz);
  }

private:
  void process()
  {
    const auto msg = latest_depth_;
    if (!msg || msg == last_done_) {return;}
    last_done_ = msg;
    if (!latest_info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "camera_info 대기 중");
      return;
    }
    // 카메라 → base_link 변환 (없으면 바닥을 못 거르므로 발행하지 않음)
    tf2::Transform T;
    try {
      const auto ts = tf_buffer_->lookupTransform(
        base_frame_, msg->header.frame_id, msg->header.stamp, tf2::durationFromSec(tf_timeout_));
      tf2::fromMsg(ts.transform, T);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "tf %s→%s 없음 → 장애물 점군 발행 안 함: %s",
        msg->header.frame_id.c_str(), base_frame_.c_str(), e.what());
      return;
    }

    cv::Mat depth;
    try {
      depth = cv_bridge::toCvShare(msg)->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "depth 변환 실패: %s", e.what());
      return;
    }
    const double fx = latest_info_->k[0], fy = latest_info_->k[4];
    const double cx = latest_info_->k[2], cy = latest_info_->k[5];
    if (fx <= 0 || fy <= 0) {return;}
    const bool is_u16 = (depth.type() == CV_16UC1);
    if (!is_u16 && depth.type() != CV_32FC1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "지원하지 않는 depth 타입");
      return;
    }

    std::vector<std::array<float, 3>> pts;
    pts.reserve(static_cast<size_t>((depth.cols / step_) * (depth.rows / step_)));
    for (int y = 0; y < depth.rows; y += step_) {
      for (int x = 0; x < depth.cols; x += step_) {
        const double z = is_u16 ? depth.at<uint16_t>(y, x) * 0.001 : depth.at<float>(y, x);
        if (!(std::isfinite(z) && min_depth_m_ <= z && z <= max_depth_m_)) {continue;}
        // 카메라 광학 좌표 → base_link
        const tf2::Vector3 pb = T * tf2::Vector3((x - cx) / fx * z, (y - cy) / fy * z, z);
        if (pb.z() <= floor_margin_m_ || pb.z() >= max_height_m_) {continue;}   // 바닥 / 너무 높은 것 제거
        pts.push_back({static_cast<float>(pb.x()), static_cast<float>(pb.y()), static_cast<float>(pb.z())});
      }
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = msg->header.stamp;
    cloud.header.frame_id = base_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(pts.size());
    cloud.is_dense = false;
    cloud.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
    for (const auto & p : pts) {
      *ix = p[0]; *iy = p[1]; *iz = p[2];
      ++ix; ++iy; ++iz;
    }
    pub_->publish(cloud);
  }

  std::string base_frame_;
  double min_depth_m_, max_depth_m_, floor_margin_m_, max_height_m_, tf_timeout_;
  int step_ = 4;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_, last_done_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nz2_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<nz2_perception::ObstacleDetectorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("obstacle_detector"), "시작 실패: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
