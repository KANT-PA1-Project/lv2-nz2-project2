// =====================================================================
// shelf_detector_node  (NZ2 인지)
//   역할: AprilTag(장소 식별) + 그리퍼 depth(윗면 평면) → 장소(박스) pose
//   입력: /perception/apriltags           (isaac_ros_apriltag_interfaces/AprilTagDetectionArray)
//         /gripper/aligned_depth_to_color/image_raw (sensor_msgs/Image)  윗면 평면 추출 (아직 미사용)
//   [수정 2026-10-28] 전면 카메라 삭제 → 그리퍼 depth. 미션 v5: 장소 = 205x205x70 박스(L1~L3 태그 12~14,
//     접근 방향 옆면, 태그 중심 높이 35 mm) → 윗면 기본 높이 0.080 → 0.070
//         /tf                              (태그/맵 프레임)
//   출력: /perception/shelf_pose           (nz2_interfaces/ShelfPose, frame_id map)
//
//   [빌드 주의] isaac_ros_apriltag_interfaces 가 설치돼 있어야 이 노드가 빌드된다.
//     없으면 CMake 가 이 실행파일을 건너뛴다(find_package QUIET). Isaac ROS 환경에서 빌드.
//   [초안] 태그 pose 는 TF(map←tag 프레임)로 얻고, 선반 윗면 높이는 depth 평면 추출(TODO).
//          지금은 태그 id 로 ShelfPose 뼈대만 채워 발행.
//
//   ※ C++ 문법 기초는 yolo_detector_node.cpp 맨 위 주석을 먼저 보세요.
//   ※ 이 노드가 하는 일: 선반에 붙은 "AprilTag"(흑백 바코드 같은 마커)를 다른 노드가
//      인식해 주면, 그 태그가 지도(map) 상 어디에 있는지를 TF 로 계산해 선반 위치로 발행한다.
//   ※ TF 란? ROS 가 "좌표계들 사이 변환"을 계속 관리해 주는 시스템이다. 예: "map 기준으로
//      이 태그는 어디?"를 lookupTransform 한 번으로 물어볼 수 있다(직접 행렬계산 불필요).
// =====================================================================
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nz2_interfaces/msg/shelf_pose.hpp>

// tf2_ros 헤더: 최신 배포판(Lyrical 등)은 .hpp, 구버전은 .h 만 있다.
// .h 를 쓰면 'DEPRECATION' 빌드 경고가 나므로, .hpp 가 있으면 그걸 쓰도록 분기한다.
#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>                 // TF 저장소(과거 변환들을 들고 있음)
#include <tf2_ros/transform_listener.hpp>     // TF 를 자동 수신해 buffer 를 채워주는 리스너
#else
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#endif

#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>

namespace nz2_perception
{

class ShelfDetectorNode : public rclcpp::Node
{
public:
  ShelfDetectorNode()
  : rclcpp::Node("shelf_detector")
  {
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    map_frame_ = declare_parameter<std::string>("map_frame", "map", ro);  // 기준 좌표계 이름
    // 태그 프레임 이름 앞에 붙는 접두사. 예: id 3 → "tag36h11:3".
    tag_prefix_ = declare_parameter<std::string>("tag_frame_prefix", "tag36h11:", ro);
    default_top_height_ = declare_parameter<double>("default_top_height", 0.070, ro);  // 선반 윗면 높이 기본값(m)
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1, ro);        // TF 조회 최대 대기(초)

    // TF 준비: Buffer(저장소)는 하나만 있으면 되니 unique_ptr(단독 소유),
    //          Listener 는 내부에서 buffer 를 참조하며 공유되므로 shared_ptr.
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_ = create_publisher<nz2_interfaces::msg::ShelfPose>(
      "/perception/shelf_pose", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/gripper/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {latest_depth_ = m;});
    tag_sub_ = create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
      "/perception/apriltags", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&ShelfDetectorNode::on_tags, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "shelf_detector 시작: map=%s, tag_prefix='%s' → /perception/shelf_pose",
      map_frame_.c_str(), tag_prefix_.c_str());
  }

private:
  void on_tags(const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::ConstSharedPtr & tags)
  {
    for (const auto & det : tags->detections) {   // 이번에 보인 태그마다 반복
      const int tag_id = static_cast<int>(det.id);
      // 태그 좌표계 이름 조립. std::to_string(3) → "3", 문자열끼리 + 로 이어붙임.
      const std::string tag_frame = tag_prefix_ + std::to_string(tag_id);

      nz2_interfaces::msg::ShelfPose sp;
      sp.header.stamp = tags->header.stamp;
      sp.header.frame_id = map_frame_;        // 이 pose 가 map 좌표계 기준임을 명시
      sp.shelf_id = tag_id;
      sp.top_height = static_cast<float>(default_top_height_);  // TODO: depth 평면으로 실측
      sp.confidence = 1.0f;

      // 태그 pose 를 map 기준으로 TF 조회 (AprilTag 노드가 tag 프레임을 broadcast)
      try {
        // lookupTransform(목표프레임, 원본프레임, 시각, 타임아웃):
        //   "map 좌표계에서 본 tag_frame 의 위치/자세"를 돌려준다.
        //   실패하면(아직 TF 안 옴 등) 예외를 던지므로 try/catch 로 감쌈.
        const auto tf = tf_buffer_->lookupTransform(
          map_frame_, tag_frame, tags->header.stamp, tf2::durationFromSec(tf_timeout_));
        sp.pose.position.x = tf.transform.translation.x;   // 위치(미터)
        sp.pose.position.y = tf.transform.translation.y;
        sp.pose.position.z = tf.transform.translation.z;
        sp.pose.orientation = tf.transform.rotation;        // 자세(쿼터니언)
        // TODO(nz2): 태그 pose → 선반 "앞 모서리 중앙, x축 바깥" 규약으로 보정 + depth 윗면 높이.
      } catch (const tf2::TransformException & e) {
        // TF 를 못 구하면 "유효하지 않음"으로 표시(confidence 0)하고 넘어간다.
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "tf %s←%s 실패: %s",
          map_frame_.c_str(), tag_frame.c_str(), e.what());
        sp.pose.orientation.w = 1.0;
        sp.confidence = 0.0f;
      }
      pub_->publish(sp);
    }
  }

  // ── 멤버 변수 ──
  std::string map_frame_, tag_prefix_;
  double default_top_height_, tf_timeout_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;              // TF 저장소(단독 소유)
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // TF 수신기(공유)
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;    // 최신 depth(평면 실측용, 아직 미사용)

  rclcpp::Publisher<nz2_interfaces::msg::ShelfPose>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr tag_sub_;
};

}  // namespace nz2_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<nz2_perception::ShelfDetectorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("shelf_detector"), "시작 실패: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
