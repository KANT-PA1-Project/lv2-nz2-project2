// nz2_swerve_nav/src/swerve_controller_node.cpp
// 스워브 구동 노드 (Jetson / 개발 PC 시뮬 공용)
//
// 입력
//   /cmd_vel           geometry_msgs/TwistStamped (use_stamped_cmd_vel=true, Nav2 기본) 또는 Twist  — base_footprint 기준
//   /joint_states      sensor_msgs/JointState — 조향 위치 [rad], 바퀴 속도 [rad/s, 바퀴축 기준(기어 후)]
//   /estop             std_msgs/Bool — true 동안 바퀴 0 (해제는 false 수신 + cmd 0 확인 후)
//   /swerve/x_lock     std_msgs/Bool — true 이고 정지 명령이면 바퀴를 X 자(45°)로
// 출력
//   /swerve/command    std_msgs/Float64MultiArray [조향 4 (rad), 바퀴 4 (rad/s)] — 순서 = module_names. 실물 OpenCR 브리지 입력
//   /swerve/sim/<joint>/cmd_pos|cmd_vel  std_msgs/Float64 — sim_joint_topics=true 일 때 (Gazebo)
//   /odom              nav_msgs/Odometry (odom → base_footprint) + TF (publish_tf)
//   /swerve/footprint  geometry_msgs/PolygonStamped — 실제 조향각 기반 외곽 (Nav2 코스트맵 footprint 토픽으로 연결)
//   /swerve/diag       std_msgs/Float64MultiArray [정렬배율, 미끄럼잔차 m/s, cmd나이 s, js나이 s, estop]
//
// odom 전용 모드 (drive_enable=false, 11/13 설계 v2)
//   구동은 팀 r12_swerve_controller 가 맡음. 이 노드는 cmd_vel·estop·x_lock 을 구독하지 않고 명령도 내지 않음
//   출력은 /odom + TF + /swerve/footprint + /swerve/diag(정렬배율 = -1) 만
//   footprint 의 '명령 조향각' = steer_command_topic (기본 /r12/swerve_joint_commands, JointState position)
//   실물: joint_states → /r12/joint_states 리매핑
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "nz2_swerve_nav/swerve_kinematics.hpp"

using namespace std::chrono_literals;
using nz2_swerve_nav::kNumModules;

class SwerveControllerNode : public rclcpp::Node {
public:
  SwerveControllerNode() : Node("swerve_controller") {
    // ---------- 파라미터 (config/swerve.yaml 한곳에서 관리) ----------
    names_ = declare_parameter<std::vector<std::string>>(
        "module_names", {"front_left", "front_right", "rear_left", "rear_right"});
    const auto px = declare_parameter<std::vector<double>>("module_x", {0.124, 0.124, -0.124, -0.124});
    const auto py = declare_parameter<std::vector<double>>("module_y", {0.124, -0.124, 0.124, -0.124});
    const auto off = declare_parameter<std::vector<double>>("steer_offset", {0.0, 0.0, 0.0, 0.0});
    const auto wsg = declare_parameter<std::vector<double>>("wheel_sign", {1.0, 1.0, 1.0, 1.0});
    const auto bcx = declare_parameter<std::vector<double>>("box_center_x", {0.019183, 0.019248, -0.019248, -0.019183});
    const auto bcy = declare_parameter<std::vector<double>>("box_center_y", {0.001219, -0.001252, 0.001252, -0.001219});
    const auto bh = declare_parameter<std::vector<double>>("box_half", {0.0747, 0.0572});
    const auto ph = declare_parameter<std::vector<double>>("plate_half", {0.17, 0.17});
    // 라이다 하우징 중심 (base_link, x1 y1 x2 y2 ...) 과 반폭 — 메시 실측값. footprint 에 정사각형으로 포함
    const auto lid = declare_parameter<std::vector<double>>(
        "lidar_xy", {0.1851, 0.0, 0.0, 0.1851, -0.1851, 0.0, 0.0, -0.1851});
    const double lid_half = declare_parameter<double>("lidar_half", 0.0300);

    nz2_swerve_nav::SwerveParams p;
    p.base_yaw = declare_parameter<double>("base_yaw", M_PI);
    p.wheel_radius = declare_parameter<double>("wheel_radius", 0.033);
    p.max_wheel_speed = declare_parameter<double>("max_wheel_speed", 0.95);
    p.steer_limit = declare_parameter<double>("steer_limit", 1.92);
    p.clamp_tolerance = declare_parameter<double>("clamp_tolerance", 0.17);
    p.align_stop = declare_parameter<double>("align_stop", 0.35);
    p.min_module_speed = declare_parameter<double>("min_module_speed", 0.005);
    p.plate_half = {ph.at(0), ph.at(1)};
    if (lid.size() % 2 != 0) throw std::runtime_error("lidar_xy 길이는 짝수여야 함");
    for (std::size_t i = 0; i + 1 < lid.size(); i += 2)
      for (int sx : {-1, 1})
        for (int sy : {-1, 1}) p.extra_points.push_back({lid[i] + sx * lid_half, lid[i + 1] + sy * lid_half});

    if (names_.size() != kNumModules || px.size() != kNumModules || py.size() != kNumModules ||
        off.size() != kNumModules || wsg.size() != kNumModules || bcx.size() != kNumModules ||
        bcy.size() != kNumModules || bh.size() != 2) {
      throw std::runtime_error("모듈 파라미터 배열 길이 오류 (모듈 4개, box_half 2개)");
    }
    for (std::size_t i = 0; i < kNumModules; ++i) {
      auto & m = p.modules[i];
      m.pos = {px[i], py[i]};
      m.steer_offset = off[i];
      m.wheel_sign = (wsg[i] >= 0.0) ? 1.0 : -1.0;
      m.box_center = {bcx[i], bcy[i]};
      m.box_half = {bh[0], bh[1]};
      steer_joint_[i] = names_[i] + "_steer_joint";
      wheel_joint_[i] = names_[i] + "_wheel_joint";
    }
    kin_ = std::make_unique<nz2_swerve_nav::SwerveKinematics>(p);

    max_vx_ = declare_parameter<double>("max_vx", 0.30);
    max_vy_ = declare_parameter<double>("max_vy", 0.30);
    max_wz_ = declare_parameter<double>("max_wz", 1.0);
    acc_lin_ = declare_parameter<double>("max_acc_linear", 0.8);   // m/s^2
    acc_ang_ = declare_parameter<double>("max_acc_angular", 3.0);  // rad/s^2
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.15);
    js_timeout_ = declare_parameter<double>("joint_state_timeout", 0.20);
    rate_hz_ = declare_parameter<double>("control_rate", 50.0);
    footprint_padding_ = declare_parameter<double>("footprint_padding", 0.01);
    footprint_rate_ = declare_parameter<double>("footprint_rate", 5.0);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    // 오도메트리 보정 배율 (실측 ÷ odom). 시뮬 11/13 odom_check: 제자리 375° 회전 → odom 356° (비 0.950)
    // 실물은 2 m 직진·360° 회전 실측으로 따로 맞출 것. 0.8~1.2 밖이면 설정 오류로 보고 거부
    odom_lin_scale_ = declare_parameter<double>("odom_linear_scale", 1.0);
    odom_yaw_scale_ = declare_parameter<double>("odom_yaw_scale", 1.0);
    for (double k : {odom_lin_scale_, odom_yaw_scale_})
      if (!(k > 0.8 && k < 1.2)) throw std::runtime_error("odom_linear_scale / odom_yaw_scale 는 0.8~1.2");
    sim_topics_ = declare_parameter<bool>("sim_joint_topics", false);
    const bool stamped = declare_parameter<bool>("use_stamped_cmd_vel", true);
    drive_enable_ = declare_parameter<bool>("drive_enable", true);
    const auto steer_cmd_topic = declare_parameter<std::string>("steer_command_topic", "/r12/swerve_joint_commands");

    // ---------- 통신 ----------
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);
    sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", sensor_qos, [this](sensor_msgs::msg::JointState::ConstSharedPtr m) { onJointState(*m); });
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pub_fp_ = create_publisher<geometry_msgs::msg::PolygonStamped>("swerve/footprint", rclcpp::QoS(1).transient_local());
    pub_diag_ = create_publisher<std_msgs::msg::Float64MultiArray>("swerve/diag", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    last_tick_ = now();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz_), [this]() { tick(); });

    if (!drive_enable_) {
      // odom 전용: 외부 컨트롤러의 조향 명령을 footprint(조향 전환 중 휩쓸 영역)에만 사용
      sub_steer_cmd_ = create_subscription<sensor_msgs::msg::JointState>(
          steer_cmd_topic, 10, [this](sensor_msgs::msg::JointState::ConstSharedPtr m) { onSteerCommand(*m); });
      RCLCPP_INFO(get_logger(), "swerve_controller 시작 (odom 전용, drive_enable=false): %.0f Hz, 조향 명령 토픽 %s",
                  rate_hz_, steer_cmd_topic.c_str());
      return;
    }

    if (stamped) {
      sub_cmd_s_ = create_subscription<geometry_msgs::msg::TwistStamped>(
          "cmd_vel", 10, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) { onCmd(m->twist); });
    } else {
      sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
          "cmd_vel", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) { onCmd(*m); });
    }
    sub_estop_ = create_subscription<std_msgs::msg::Bool>(
        "estop", rclcpp::QoS(10).reliable(), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          if (m->data && !estop_) RCLCPP_ERROR(get_logger(), "E-STOP 수신: 바퀴 정지");
          if (m->data) estop_ = true; else estop_release_req_ = true;
        });
    sub_xlock_ = create_subscription<std_msgs::msg::Bool>(
        "swerve/x_lock", 10, [this](std_msgs::msg::Bool::ConstSharedPtr m) { x_lock_ = m->data; });

    pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>("swerve/command", 10);
    if (sim_topics_) {
      for (std::size_t i = 0; i < kNumModules; ++i) {
        pub_sim_steer_[i] = create_publisher<std_msgs::msg::Float64>("swerve/sim/" + steer_joint_[i] + "/cmd_pos", 10);
        pub_sim_wheel_[i] = create_publisher<std_msgs::msg::Float64>("swerve/sim/" + wheel_joint_[i] + "/cmd_vel", 10);
      }
    }
    RCLCPP_INFO(get_logger(), "swerve_controller 시작: %.0f Hz, 조향 한계 ±%.2f rad, 속도 상한 vx %.2f vy %.2f wz %.2f",
                rate_hz_, p.steer_limit, max_vx_, max_vy_, max_wz_);
  }

private:
  void onCmd(const geometry_msgs::msg::Twist & t) {
    if (!std::isfinite(t.linear.x) || !std::isfinite(t.linear.y) || !std::isfinite(t.angular.z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "cmd_vel 에 NaN/Inf → 무시");
      return;
    }
    target_.vx = std::clamp(t.linear.x, -max_vx_, max_vx_);
    target_.vy = std::clamp(t.linear.y, -max_vy_, max_vy_);
    target_.wz = std::clamp(t.angular.z, -max_wz_, max_wz_);
    last_cmd_time_ = now();
    have_cmd_ = true;
  }

  void onJointState(const sensor_msgs::msg::JointState & js) {
    bool got_steer[kNumModules] = {false}, got_wheel[kNumModules] = {false};
    for (std::size_t j = 0; j < js.name.size(); ++j) {
      for (std::size_t i = 0; i < kNumModules; ++i) {
        if (js.name[j] == steer_joint_[i] && j < js.position.size()) {
          meas_[i].steer = js.position[j]; got_steer[i] = true;
        } else if (js.name[j] == wheel_joint_[i] && j < js.velocity.size()) {
          meas_[i].wheel_vel = js.velocity[j]; got_wheel[i] = true;
        }
      }
    }
    bool all = true;
    for (std::size_t i = 0; i < kNumModules; ++i) all = all && got_steer[i] && got_wheel[i];
    if (all) { last_js_time_ = now(); have_js_ = true; }
    else RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_states 에 스워브 관절 8개가 모두 있지 않음");
  }

  // 외부 컨트롤러(r12) 조향 명령: 조향 관절 position 만 사용 (바퀴 메시지는 position 이 비어 있어 건너뜀)
  void onSteerCommand(const sensor_msgs::msg::JointState & js) {
    for (std::size_t j = 0; j < js.name.size() && j < js.position.size(); ++j)
      for (std::size_t i = 0; i < kNumModules; ++i)
        if (js.name[j] == steer_joint_[i] && std::isfinite(js.position[j])) {
          ext_steer_[i] = js.position[j];
          have_ext_steer_ = true;
        }
  }

  // odom 전용 틱: 명령 발행 없음
  void tickOdomOnly(const rclcpp::Time & t, double dt, bool js_ok, double js_age) {
    if (!js_ok) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_states 끊김/없음 → odom 정지(속도 0 적분)");
    std::array<nz2_swerve_nav::ModuleCommand, kNumModules> cmd{};
    for (std::size_t i = 0; i < kNumModules; ++i) cmd[i].steer = have_ext_steer_ ? ext_steer_[i] : meas_[i].steer;
    updateOdom(t, dt, js_ok);
    publishFootprint(t, cmd);
    std_msgs::msg::Float64MultiArray d;
    d.data = {-1.0, slip_residual_, -1.0, std::min(js_age, 99.0), 0.0};
    pub_diag_->publish(d);
  }

  static double ramp(double cur, double tgt, double max_step) {
    return cur + std::clamp(tgt - cur, -max_step, max_step);
  }

  void tick() {
    const rclcpp::Time t = now();
    double dt = (t - last_tick_).seconds();
    last_tick_ = t;
    if (dt < 0.0 || dt > 0.5) dt = 0.0;  // 시뮬 시간 정지·점프 보호 (적분·가속 안 함)

    const double cmd_age = have_cmd_ ? (t - last_cmd_time_).seconds() : 1e9;
    const double js_age = have_js_ ? (t - last_js_time_).seconds() : 1e9;
    const bool js_ok = js_age < js_timeout_;
    if (!drive_enable_) { tickOdomOnly(t, dt, js_ok, js_age); return; }

    // E-STOP 해제: false 수신 + 목표 속도 0 일 때만 (갑자기 튀어나가는 것 방지)
    if (estop_ && estop_release_req_) {
      if (std::fabs(target_.vx) < 1e-3 && std::fabs(target_.vy) < 1e-3 && std::fabs(target_.wz) < 1e-3) {
        estop_ = false;
        RCLCPP_WARN(get_logger(), "E-STOP 해제");
      }
      estop_release_req_ = false;
    }

    nz2_swerve_nav::Twist2D goal = target_;
    if (cmd_age > cmd_timeout_ || estop_) goal = {};
    if (cmd_age > cmd_timeout_ && have_cmd_ && was_moving_)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "cmd_vel 끊김 %.2f s → 정지", cmd_age);
    if (!js_ok) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "joint_states 끊김/없음 → 바퀴 정지");

    // 가감속 제한 (Nav2 velocity_smoother 앞단 보조. E-STOP·끊김은 즉시 0)
    if (estop_ || cmd_age > cmd_timeout_) {
      cur_ = {};
    } else {
      cur_.vx = ramp(cur_.vx, goal.vx, acc_lin_ * dt);
      cur_.vy = ramp(cur_.vy, goal.vy, acc_lin_ * dt);
      cur_.wz = ramp(cur_.wz, goal.wz, acc_ang_ * dt);
    }
    const bool moving = std::fabs(cur_.vx) + std::fabs(cur_.vy) + std::fabs(cur_.wz) > 1e-4;
    was_moving_ = moving;

    std::array<nz2_swerve_nav::ModuleCommand, kNumModules> cmd;
    double align = 1.0;
    if (estop_) cmd = kin_->hold();
    else if (!moving && x_lock_) cmd = kin_->xLock();
    else cmd = kin_->inverse(cur_, meas_, js_ok, &align);

    // 조향이 목표를 못 따라가 바퀴가 계속 막히면 경고 (Gazebo 게인·실물 조향 문제 진단용)
    if (moving && js_ok && align == 0.0) {
      if (blocked_since_.nanoseconds() == 0) blocked_since_ = t;
      if ((t - blocked_since_).seconds() > 1.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "조향 미정렬로 바퀴 정지 중: 목표 [%.2f %.2f %.2f %.2f] 측정 [%.2f %.2f %.2f %.2f] rad",
            cmd[0].steer, cmd[1].steer, cmd[2].steer, cmd[3].steer,
            meas_[0].steer, meas_[1].steer, meas_[2].steer, meas_[3].steer);
      }
    } else {
      blocked_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    publishCommand(cmd);
    updateOdom(t, dt, js_ok);
    publishFootprint(t, cmd);

    std_msgs::msg::Float64MultiArray d;
    d.data = {align, slip_residual_, std::min(cmd_age, 99.0), std::min(js_age, 99.0), estop_ ? 1.0 : 0.0};
    pub_diag_->publish(d);
  }

  void publishCommand(const std::array<nz2_swerve_nav::ModuleCommand, kNumModules> & cmd) {
    std_msgs::msg::Float64MultiArray m;
    m.data.resize(2 * kNumModules);
    for (std::size_t i = 0; i < kNumModules; ++i) {
      m.data[i] = cmd[i].steer;
      m.data[kNumModules + i] = cmd[i].wheel_vel;
    }
    pub_cmd_->publish(m);
    if (sim_topics_) {
      for (std::size_t i = 0; i < kNumModules; ++i) {
        std_msgs::msg::Float64 s, w;
        s.data = cmd[i].steer;
        w.data = cmd[i].wheel_vel;
        pub_sim_steer_[i]->publish(s);
        pub_sim_wheel_[i]->publish(w);
      }
    }
  }

  void updateOdom(const rclcpp::Time & t, double dt, bool js_ok) {
    nz2_swerve_nav::Twist2D v{};
    if (js_ok) v = kin_->forward(meas_, &slip_residual_);
    v.vx *= odom_lin_scale_;
    v.vy *= odom_lin_scale_;
    v.wz *= odom_yaw_scale_;
    // 중점 적분
    const double th_mid = th_ + 0.5 * v.wz * dt;
    x_ += (v.vx * std::cos(th_mid) - v.vy * std::sin(th_mid)) * dt;
    y_ += (v.vx * std::sin(th_mid) + v.vy * std::cos(th_mid)) * dt;
    th_ = nz2_swerve_nav::wrapPi(th_ + v.wz * dt);

    nav_msgs::msg::Odometry o;
    o.header.stamp = t;
    o.header.frame_id = odom_frame_;
    o.child_frame_id = base_frame_;
    o.pose.pose.position.x = x_;
    o.pose.pose.position.y = y_;
    o.pose.pose.orientation.z = std::sin(th_ / 2);
    o.pose.pose.orientation.w = std::cos(th_ / 2);
    o.twist.twist.linear.x = v.vx;
    o.twist.twist.linear.y = v.vy;
    o.twist.twist.angular.z = v.wz;
    // 공분산 [가정]: 미끄럼 잔차가 클수록 키움
    const double k = 1.0 + 50.0 * slip_residual_;
    o.pose.covariance[0] = o.pose.covariance[7] = 0.001 * k;
    o.pose.covariance[35] = 0.003 * k;
    o.pose.covariance[14] = o.pose.covariance[21] = o.pose.covariance[28] = 1e6;
    o.twist.covariance[0] = o.twist.covariance[7] = 0.002 * k;
    o.twist.covariance[35] = 0.005 * k;
    o.twist.covariance[14] = o.twist.covariance[21] = o.twist.covariance[28] = 1e6;
    pub_odom_->publish(o);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = o.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.rotation = o.pose.pose.orientation;
      tf_->sendTransform(tf);
    }
  }

  void publishFootprint(const rclcpp::Time & t, const std::array<nz2_swerve_nav::ModuleCommand, kNumModules> & cmd) {
    if ((t - last_fp_time_).seconds() < 1.0 / footprint_rate_) return;
    last_fp_time_ = t;
    std::array<double, kNumModules> a{}, b{};
    for (std::size_t i = 0; i < kNumModules; ++i) { a[i] = meas_[i].steer; b[i] = cmd[i].steer; }
    // 측정·명령 합집합 → 조향 전환 중 휩쓸 영역 포함
    const auto hull = kin_->footprint({a, b}, footprint_padding_);
    geometry_msgs::msg::PolygonStamped ps;
    ps.header.stamp = t;
    ps.header.frame_id = base_frame_;
    for (const auto & q : hull) {
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(q.x);
      p.y = static_cast<float>(q.y);
      ps.polygon.points.push_back(p);
    }
    pub_fp_->publish(ps);
  }

  // 상태
  std::vector<std::string> names_;
  std::array<std::string, kNumModules> steer_joint_, wheel_joint_;
  std::unique_ptr<nz2_swerve_nav::SwerveKinematics> kin_;
  std::array<nz2_swerve_nav::ModuleState, kNumModules> meas_{};
  nz2_swerve_nav::Twist2D target_{}, cur_{};
  double max_vx_, max_vy_, max_wz_, acc_lin_, acc_ang_, cmd_timeout_, js_timeout_, rate_hz_;
  double footprint_padding_, footprint_rate_;
  std::string odom_frame_, base_frame_;
  bool publish_tf_{true}, sim_topics_{false}, drive_enable_{true}, have_ext_steer_{false};
  std::array<double, kNumModules> ext_steer_{};
  bool have_cmd_{false}, have_js_{false}, estop_{false}, estop_release_req_{false}, x_lock_{false}, was_moving_{false};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME}, last_js_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME}, last_fp_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time blocked_since_{0, 0, RCL_ROS_TIME};
  double x_{0.0}, y_{0.0}, th_{0.0}, slip_residual_{0.0};
  double odom_lin_scale_{1.0}, odom_yaw_scale_{1.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_s_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_, sub_steer_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_, sub_xlock_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_, pub_diag_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_fp_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kNumModules> pub_sim_steer_, pub_sim_wheel_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwerveControllerNode>());
  rclcpp::shutdown();
  return 0;
}
