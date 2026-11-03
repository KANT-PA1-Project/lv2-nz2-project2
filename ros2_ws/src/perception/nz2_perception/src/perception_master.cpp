// =====================================================================
// perception_master  (NZ2 인지 집계)
//   역할: 인지 결과 집계 · 상태 감시 · 검출기 on/off (SetPerceptionMode)
//   입력: /perception/traffic_light (TrafficLight), /perception/puck_tracks (PuckArray),
//         /perception/barcodes (BarcodeArray), /gripper/color/camera_info, /perception/detections,
//         (선택) pose_topic (PoseWithCovarianceStamped)
//   서비스: /perception/set_mode (SetPerceptionMode)  — mission_fsm 이 호출
//   출력: /perception/state (PerceptionState) 10 Hz → mission_fsm, planning_master
//   양식: xmen_tracker 의 상태 수집 + 주기 발행 + 신선도(age) 감시 패턴.
//
//   [초안] vslam_tracking_ok 는 미연결(TODO, cuvslam 상태 연동). fps 는 수신 개수 기반 근사.
//   [수정 2026-10-28] 전면 카메라 삭제 → 신호등·바코드·퍽 모두 그리퍼 카메라.
//     gripper_camera_ok = 셋 중 하나라도 최근에 왔는가. front_camera_ok 는 항상 false (메시지 필드는 호환용으로 남김)
//     장애물(/perception/obstacles)은 하단 센서로 이관 → 카메라 헬스 판정에서 제외
//
//   [수정 2026-10-30] 퍽·신호·주사위 결과를 실제로 집계 (예전엔 옛 puck_localizer 의 /perception/pucks 만 받았음)
//     - 퍽: /perception/puck_tracks (puck_registry: 태그 ID·추적·3프레임 확정) 로 변경. 토픽은 파라미터 pucks_topic
//     - 카메라 생존: 결과 토픽이 아니라 /gripper/color/camera_info 수신으로 판정 (퍽이 안 보여도 카메라는 OK)
//     - yolo_fps: /perception/detections 수신률 (예전엔 퍽 토픽 수)
//     - 신호 기록(/perception/signal_result), 주사위(/nz2/dice_summary), 등록표(/nz2/puck_registry) 는
//       PerceptionState 메시지에 칸이 없어서(메시지 바꾸면 팀 노드와 호환 깨짐) → /perception/summary (String JSON,
//       transient_local, 1 Hz + 바뀔 때) 로 한 번에 묶어 발행. 미션 노드는 state + summary 둘만 보면 됨
//     - 위치: pose_topic(기본 /amcl_pose). 카토그래퍼만 쓰면 이 토픽이 없어 localization_ok=false → 추후 tf 판정 추가
//   [수정 2026-11-02] 팀 nz2_interfaces 통합(#21): PerceptionState 에서 shelves·front_camera_ok·vslam_tracking_ok 삭제,
//     gripper_camera_ok → camera_ok, front_fps → camera_fps. SetPerceptionMode 에서 enable_front 삭제
//   ※ C++ 문법 기초는 yolo_detector_node.cpp 맨 위 주석을 먼저 보세요.
//   ※ 이 노드의 역할(한 줄): 흩어져 있는 인지 결과(신호등/선반/퍽/바코드/장애물/위치)를
//      한 곳에 모아 두고, 10Hz 타이머로 "지금 인지 상태 요약(PerceptionState)"을 발행한다.
//      미션 FSM·플래너는 이 요약 하나만 보면 된다.
//   ※ "신선도(freshness)"가 핵심 안전 개념: 각 입력이 마지막으로 온 시각을 저장해 두고,
//      너무 오래됐으면(센서/노드가 죽었을 수 있음) 'OK=false'로 표시한다.
//      즉 "최근에 들어왔는가?"로 카메라·측위가 살아있는지 판단한다.
// =====================================================================
#include <cmath>
#include <map>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <nz2_interfaces/msg/perception_state.hpp>
#include <nz2_interfaces/msg/traffic_light.hpp>
#include <nz2_interfaces/msg/puck_array.hpp>
#include <nz2_interfaces/msg/barcode_array.hpp>
#include <nz2_interfaces/srv/set_perception_mode.hpp>

namespace nz2_perception
{

class PerceptionMaster : public rclcpp::Node
{
public:
  PerceptionMaster()
  : rclcpp::Node("perception_master")
  {
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    state_hz_ = declare_parameter<double>("state_hz", 10.0, ro);          // 상태 발행 주기(Hz)
    fresh_timeout_ = declare_parameter<double>("fresh_timeout", 0.5, ro); // 이 시간 넘게 안 오면 "끊김"으로 간주(초)
    loc_cov_max_ = declare_parameter<double>("loc_cov_max", 0.25, ro);  // x+y 분산 합 임계
    // [2026-10-30] 입력 토픽 (한곳에서 바꿈)
    const auto pucks_topic = declare_parameter<std::string>("pucks_topic", "/perception/puck_tracks", ro);
    const auto det_topic = declare_parameter<std::string>("detections_topic", "/perception/detections", ro);
    const auto info_topic = declare_parameter<std::string>("camera_info_topic", "/gripper/color/camera_info", ro);
    const auto pose_topic = declare_parameter<std::string>("pose_topic", "/amcl_pose", ro);
    summary_period_ = declare_parameter<double>("summary_period_s", 1.0, ro);   // summary 최소 발행 주기

    if (!(0.0 < state_hz_ && state_hz_ <= 50.0)) {
      throw std::invalid_argument("state_hz must be in (0, 50]");
    }
    if (fresh_timeout_ <= 0.0) {throw std::invalid_argument("fresh_timeout must be > 0");}

    // rel = "신뢰성(reliable) + 최근 10개 보관" QoS. 여러 구독에서 재사용하려고 변수로 둠.
    const rclcpp::QoS rel = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    state_pub_ = create_publisher<nz2_interfaces::msg::PerceptionState>("/perception/state", rel);
    // latched = 늦게 붙은 노드도 마지막 값을 바로 받음 (transient_local)
    const rclcpp::QoS latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    summary_pub_ = create_publisher<std_msgs::msg::String>("/perception/summary", latched);

    // 각 입력 토픽을 구독. 콜백은 람다 [this](...){...} 로, "최신값 저장 + 도착시각 기록"만 한다.
    // *m 은 "포인터 m 이 가리키는 메시지 본체"(역참조). light_ = *m 은 그 내용을 복사 저장.
    light_sub_ = create_subscription<nz2_interfaces::msg::TrafficLight>(
      "/perception/traffic_light", rel,
      [this](nz2_interfaces::msg::TrafficLight::ConstSharedPtr m) {
        light_ = *m; t_gripper_ = now(); ++gripper_msgs_;   // now()=현재시각, ++는 수신 카운트
      });
    // [2026-10-30] 선반(/perception/shelf_pose)·장애물(/perception/obstacles) 구독 삭제 — 두 노드를 런치에서 뺐음.
    //   토픽 목록에 구독만 있는 빈 토픽이 남지 않게. state.shelves 는 빈 배열 (메시지 호환용)
    pucks_sub_ = create_subscription<nz2_interfaces::msg::PuckArray>(
      pucks_topic, rel,
      [this](nz2_interfaces::msg::PuckArray::ConstSharedPtr m) {
        pucks_ = *m; t_gripper_ = now(); t_pucks_ = now();
      });
    // 카메라 생존 = camera_info 수신 (작은 메시지라 부담 없음). 센서 QoS(best effort)로 받아야 드라이버와 맞음
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr) {t_cam_ = now(); ++cam_msgs_;});
    det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      det_topic, rel,
      [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr) {++yolo_msgs_;});
    // JSON 결과 3종: 받은 문자열을 그대로 보관했다가 summary 에 끼워 넣음
    signal_sub_ = create_subscription<std_msgs::msg::String>(
      "/perception/signal_result", latched,
      [this](std_msgs::msg::String::ConstSharedPtr m) {signal_json_ = m->data; summary_dirty_ = true;});
    dice_sub_ = create_subscription<std_msgs::msg::String>(
      "/nz2/dice_summary", latched,
      [this](std_msgs::msg::String::ConstSharedPtr m) {dice_json_ = m->data; summary_dirty_ = true;});
    registry_sub_ = create_subscription<std_msgs::msg::String>(
      "/nz2/puck_registry", latched,
      [this](std_msgs::msg::String::ConstSharedPtr m) {registry_json_ = m->data; summary_dirty_ = true;});
    barcode_sub_ = create_subscription<nz2_interfaces::msg::BarcodeArray>(
      "/perception/barcodes", rel,
      [this](nz2_interfaces::msg::BarcodeArray::ConstSharedPtr m) {
        barcodes_ = *m; t_gripper_ = now(); ++gripper_msgs_;
      });
    if (!pose_topic.empty()) {   // [2026-10-30] "" 이면 위치 구독 안 함 (AMCL 안 쓸 때). localization_ok = false
    amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr m) {
        // 공분산(covariance)=위치 불확실성. 6x6 행렬을 36칸 배열로 편 것이라
        // x 분산=[0], y 분산=[7]. 둘의 합이 작을수록(= 임계 미만) 위치가 믿을 만하다.
        loc_ok_ = (m->pose.covariance[0] + m->pose.covariance[7]) < loc_cov_max_;
        t_amcl_ = now();
      });
    }

    // 서비스: 다른 노드가 "검출기 켜/꺼"를 요청하면 on_set_mode 가 응답한다(질문→대답 방식).
    set_mode_srv_ = create_service<nz2_interfaces::srv::SetPerceptionMode>(
      "/perception/set_mode",
      std::bind(&PerceptionMaster::on_set_mode, this, std::placeholders::_1, std::placeholders::_2));

    // 타이머: 1/state_hz 초마다 publish_state() 호출 → 요약 상태를 주기 발행.
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / state_hz_),
      std::bind(&PerceptionMaster::publish_state, this));

    last_fps_time_ = now();
    RCLCPP_INFO(get_logger(), "perception_master 시작: %.0f Hz 집계 → /perception/state, 요약 → /perception/summary "
      "(퍽 %s, 카메라 %s)", state_hz_, pucks_topic.c_str(), info_topic.c_str());
  }

private:
  // 서비스 콜백: req(요청)를 읽어 설정을 바꾸고, res(응답)에 결과를 채운다.
  void on_set_mode(
    const std::shared_ptr<nz2_interfaces::srv::SetPerceptionMode::Request> req,
    std::shared_ptr<nz2_interfaces::srv::SetPerceptionMode::Response> res)
  {
    mode_ = req->mode;
    enable_gripper_ = req->enable_gripper;
    // TODO(nz2): 실제로 각 검출기 노드를 켜고/끄려면 파라미터 서비스 또는 lifecycle 로 전파.
    res->success = true;
    res->message = "mode=" + std::to_string(mode_) +
      " gripper=" + (enable_gripper_ ? "on" : "off");
    RCLCPP_INFO(get_logger(), "set_mode: %s", res->message.c_str());
  }

  // "최근에 들어왔는가?" 판정. 한 번도 안 왔으면(=0) false, 왔으면 경과시간이 임계 이내인지.
  bool fresh(const rclcpp::Time & t) const
  {
    return t.nanoseconds() > 0 && (now() - t).seconds() <= fresh_timeout_;
  }

  // 타이머가 주기적으로 부른다. 모아둔 최신값들을 하나의 요약 메시지로 묶어 발행.
  void publish_state()
  {
    nz2_interfaces::msg::PerceptionState s;
    s.header.stamp = now();
    s.header.frame_id = "base_link";
    s.mode = mode_;
    s.traffic_light = light_;
    s.pucks = pucks_.pucks;
    s.barcodes = barcodes_.codes;
    // 각 센서의 생존 여부 = "최근에 메시지가 왔는가". 안 오면 죽은 것으로 보고 false.
    s.camera_ok = enable_gripper_ ? fresh(t_cam_) : true;    // [2026-10-30] camera_info 수신 기준. 끈 상태면 true
    s.localization_ok = fresh(t_amcl_) && loc_ok_;   // 최근 수신 + 불확실성 낮음, 둘 다여야 OK

    // fps 근사: (마지막 발행 이후 받은 메시지 수) ÷ (그동안 흐른 시간).
    const double dt = (now() - last_fps_time_).seconds();
    if (dt > 0.0) {
      s.camera_fps = static_cast<float>(cam_msgs_ / dt);   // 그리퍼 카메라 fps
      s.yolo_fps = static_cast<float>(yolo_msgs_ / dt);
    }
    gripper_msgs_ = yolo_msgs_ = cam_msgs_ = 0;
    last_fps_time_ = now();

    state_pub_->publish(s);
    publish_summary(s);
  }

  // [2026-10-30] 상태 + 퍽 목록 + 신호·주사위·등록표 JSON 을 하나로 (1 s 마다, 또는 결과가 바뀌면 바로)
  void publish_summary(const nz2_interfaces::msg::PerceptionState & s)
  {
    const bool due = (now() - last_summary_).seconds() >= summary_period_;
    if (!due && !summary_dirty_) {return;}
    summary_dirty_ = false;
    last_summary_ = now();
    auto raw = [](const std::string & j) {return j.empty() ? std::string("null") : j;};   // 받은 JSON 그대로
    auto num = [](double v) {   // NaN·inf 는 JSON 에 못 넣음 → null
        if (!std::isfinite(v)) {return std::string("null");}
        std::ostringstream t;
        t.setf(std::ios::fixed);
        t.precision(4);
        t << v;
        return t.str();
      };
    std::ostringstream o;
    o.setf(std::ios::fixed);
    o.precision(4);
    o << "{\"stamp\": " << now().seconds()
      << ", \"camera_ok\": " << (s.camera_ok ? "true" : "false")
      << ", \"localization_ok\": " << (s.localization_ok ? "true" : "false")
      << ", \"camera_fps\": " << num(s.camera_fps) << ", \"yolo_fps\": " << num(s.yolo_fps)
      << ", \"pucks_fresh\": " << (fresh(t_pucks_) ? "true" : "false")
      << ", \"pucks_frame\": \"" << pucks_.header.frame_id << "\", \"pucks\": [";
    for (size_t i = 0; i < s.pucks.size(); ++i) {
      const auto & p = s.pucks[i];
      o << (i ? ", " : "") << "{\"tag_id\": " << p.tag_id << ", \"track_id\": " << p.track_id
        << ", \"color\": " << static_cast<int>(p.color) << ", \"shape\": " << static_cast<int>(p.shape)
        << ", \"posture\": " << static_cast<int>(p.posture) << ", \"conf\": " << num(p.confidence)
        << ", \"xyz\": [" << num(p.position.x) << ", " << num(p.position.y) << ", " << num(p.position.z) << "]}";
    }
    o << "], \"signal\": " << raw(signal_json_) << ", \"dice\": " << raw(dice_json_)
      << ", \"registry\": " << raw(registry_json_) << "}";
    std_msgs::msg::String m;
    m.data = o.str();
    summary_pub_->publish(m);
  }

  // 파라미터
  double state_hz_ = 10.0, fresh_timeout_ = 0.5, loc_cov_max_ = 0.25, summary_period_ = 1.0;
  std::string signal_json_, dice_json_, registry_json_;   // 받은 JSON 원문
  bool summary_dirty_ = false;
  rclcpp::Time last_summary_{0, 0, RCL_ROS_TIME};

  // 최신 상태
  nz2_interfaces::msg::TrafficLight light_;
  nz2_interfaces::msg::PuckArray pucks_;
  nz2_interfaces::msg::BarcodeArray barcodes_;
  bool loc_ok_ = false;
  uint8_t mode_ = 0;
  bool enable_gripper_ = true;

  // 신선도 타임스탬프 = 각 입력이 "마지막으로 도착한 시각". {0,0,...}=아직 안 옴.
  rclcpp::Time t_gripper_{0, 0, RCL_ROS_TIME};   // 신호등·바코드·퍽 중 마지막 수신
  rclcpp::Time t_cam_{0, 0, RCL_ROS_TIME};       // [2026-10-30] 그리퍼 camera_info 마지막 수신 (카메라 생존)
  rclcpp::Time t_pucks_{0, 0, RCL_ROS_TIME};
  rclcpp::Time t_obst_{0, 0, RCL_ROS_TIME};
  rclcpp::Time t_amcl_{0, 0, RCL_ROS_TIME};

  // fps 근사
  rclcpp::Time last_fps_time_{0, 0, RCL_ROS_TIME};
  int gripper_msgs_ = 0, yolo_msgs_ = 0, cam_msgs_ = 0;

  rclcpp::Publisher<nz2_interfaces::msg::PerceptionState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_sub_, dice_sub_, registry_sub_;
  rclcpp::Subscription<nz2_interfaces::msg::TrafficLight>::SharedPtr light_sub_;
  rclcpp::Subscription<nz2_interfaces::msg::PuckArray>::SharedPtr pucks_sub_;
  rclcpp::Subscription<nz2_interfaces::msg::BarcodeArray>::SharedPtr barcode_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obst_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Service<nz2_interfaces::srv::SetPerceptionMode>::SharedPtr set_mode_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nz2_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<nz2_perception::PerceptionMaster>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("perception_master"), "시작 실패: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
