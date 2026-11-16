// nz2_swerve_nav/src/scan_merger_node.cpp
// 라이다 4개 → 자기 몸(외곽 다각형 안) 점 제거 → base_footprint 기준 360° 스캔 1개 (/scan)
//
// 왜 필요한가
//  - 라이다가 차체 네 변 중앙(바닥 위 47.5 mm)에 있어 각 라이다가 옆 스워브 모듈 2개를 "장애물"로 본다
//    → 그대로 쓰면 코스트맵에 로봇 둘레가 항상 막혀 Nav2 가 출발을 못 함
//  - AMCL·slam_toolbox 는 스캔 토픽을 1개만 받는다 → 4개를 하나로 합쳐야 함
//
// 입력  /lidar_{front,left,rear,right}/scan (sensor_msgs/LaserScan), /swerve/footprint (PolygonStamped, base_footprint)
//       TF: base_footprint → lidar_*_scan (robot_state_publisher 의 고정 TF)
// 출력  /scan (sensor_msgs/LaserScan, frame base_footprint, angle −π~π)
// 안전  스캔이 max_scan_age 보다 오래되면 그 라이다는 빼고, 전부 끊기면 발행 중단 + 경고 (빈 스캔을 "장애물 없음"으로 보내지 않음)
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

struct P2 { double x, y; };

// 점이 다각형 안인지 (짝홀 규칙)
static bool insidePolygon(const std::vector<P2> & poly, double x, double y) {
  bool in = false;
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const auto & a = poly[i];
    const auto & b = poly[j];
    if (((a.y > y) != (b.y > y)) && (x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x)) in = !in;
  }
  return in;
}

// 점과 다각형 경계 최소 거리
static double distToPolygon(const std::vector<P2> & poly, double x, double y) {
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    const double ax = poly[j].x, ay = poly[j].y, bx = poly[i].x, by = poly[i].y;
    const double dx = bx - ax, dy = by - ay;
    const double l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((x - ax) * dx + (y - ay) * dy) / l2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    best = std::min(best, std::hypot(x - (ax + t * dx), y - (ay + t * dy)));
  }
  return best;
}

class ScanMergerNode : public rclcpp::Node {
public:
  ScanMergerNode() : Node("scan_merger") {
    inputs_ = declare_parameter<std::vector<std::string>>(
        "input_topics", {"/lidar_front/scan", "/lidar_left/scan", "/lidar_rear/scan", "/lidar_right/scan"});
    // 비어 있으면 메시지 frame_id 사용. 실물 드라이버 frame_id 가 URDF 와 다르면 여기서 URDF 프레임으로 고정
    frames_ = declare_parameter<std::vector<std::string>>(
        "input_frames", {"lidar_front_scan", "lidar_left_scan", "lidar_rear_scan", "lidar_right_scan"});
    out_frame_ = declare_parameter<std::string>("output_frame", "base_footprint");
    bins_ = declare_parameter<int>("bins", 720);                    // 0.5° 간격
    range_min_ = declare_parameter<double>("range_min", 0.05);      // [m]
    range_max_ = declare_parameter<double>("range_max", 12.0);      // [m]
    margin_ = declare_parameter<double>("footprint_margin", 0.01);   // 외곽 바깥 이 거리 안 점도 제거 [m] (벽 여유 60 mm 라 작게)
    default_half_ = declare_parameter<double>("default_footprint_half", 0.235);  // footprint 수신 전 사각 반폭 [m]
    max_age_ = declare_parameter<double>("max_scan_age", 0.25);     // [s]
    const double rate = declare_parameter<double>("publish_rate", 10.0);

    if (inputs_.size() != frames_.size()) throw std::runtime_error("input_topics 와 input_frames 길이가 같아야 함");
    if (bins_ < 90) throw std::runtime_error("bins 는 90 이상");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const double h = default_half_;
    footprint_ = {{h, h}, {-h, h}, {-h, -h}, {h, -h}};

    latest_.resize(inputs_.size());
    tf_cache_.resize(inputs_.size());
    for (std::size_t i = 0; i < inputs_.size(); ++i) {
      subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
          inputs_[i], rclcpp::SensorDataQoS(),
          [this, i](sensor_msgs::msg::LaserScan::ConstSharedPtr m) { latest_[i] = m; }));
    }
    sub_fp_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
        "swerve/footprint", rclcpp::QoS(1).transient_local(),
        [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr m) {
          if (m->polygon.points.size() < 3) return;
          if (!m->header.frame_id.empty() && m->header.frame_id != out_frame_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "footprint frame %s ≠ %s → 무시",
                                 m->header.frame_id.c_str(), out_frame_.c_str());
            return;
          }
          footprint_.clear();
          for (const auto & p : m->polygon.points) footprint_.push_back({p.x, p.y});
          have_fp_ = true;
        });
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(10));
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { tick(); });
    RCLCPP_INFO(get_logger(), "scan_merger 시작: 입력 %zu개 → %s/scan, %d bins", inputs_.size(), out_frame_.c_str(), bins_);
  }

private:
  struct Tf2D { double x, y, yaw; };

  std::optional<Tf2D> lookup(std::size_t i, const std::string & frame) {
    if (tf_cache_[i]) return tf_cache_[i];  // 라이다는 차체에 고정 → 한 번 찾으면 계속 사용
    try {
      const auto t = tf_buffer_->lookupTransform(out_frame_, frame, tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      // 쿼터니언 → roll/pitch/yaw (tf2 LinearMath 헤더는 배포판마다 .h/.hpp 가 달라 직접 계산)
      const double r = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
      const double p = std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
      const double y = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      if (std::fabs(r) > 0.05 || std::fabs(p) > 0.05)
        RCLCPP_WARN(get_logger(), "%s 가 수평이 아님 (roll %.2f pitch %.2f rad)", frame.c_str(), r, p);
      tf_cache_[i] = Tf2D{t.transform.translation.x, t.transform.translation.y, y};
      RCLCPP_INFO(get_logger(), "%s → %s: x %.3f y %.3f yaw %.1f°", frame.c_str(), out_frame_.c_str(),
                  tf_cache_[i]->x, tf_cache_[i]->y, tf_cache_[i]->yaw * 180.0 / M_PI);
      return tf_cache_[i];
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "TF %s→%s 없음: %s", out_frame_.c_str(), frame.c_str(), e.what());
      return std::nullopt;
    }
  }

  void tick() {
    const rclcpp::Time now = this->now();
    const double inc = 2.0 * M_PI / bins_;
    std::vector<float> ranges(bins_, std::numeric_limits<float>::infinity());
    int used = 0;
    rclcpp::Time newest(0, 0, get_clock()->get_clock_type());
    std::size_t removed = 0, kept = 0;

    for (std::size_t i = 0; i < latest_.size(); ++i) {
      const auto & s = latest_[i];
      if (!s) continue;
      const rclcpp::Time st(s->header.stamp, get_clock()->get_clock_type());
      if ((now - st).seconds() > max_age_) continue;
      const std::string frame = frames_[i].empty() ? s->header.frame_id : frames_[i];
      const auto tf = lookup(i, frame);
      if (!tf) continue;
      ++used;
      if (st > newest) newest = st;
      const double c = std::cos(tf->yaw), sn = std::sin(tf->yaw);
      for (std::size_t k = 0; k < s->ranges.size(); ++k) {
        const double r = s->ranges[k];
        if (!std::isfinite(r) || r < std::max<double>(s->range_min, range_min_) || r > std::min<double>(s->range_max, range_max_)) continue;
        const double a = s->angle_min + k * s->angle_increment;
        const double lx = r * std::cos(a), ly = r * std::sin(a);
        const double x = tf->x + c * lx - sn * ly;
        const double y = tf->y + sn * lx + c * ly;
        if (insidePolygon(footprint_, x, y) || distToPolygon(footprint_, x, y) < margin_) { ++removed; continue; }
        ++kept;
        const double rr = std::hypot(x, y);
        int b = static_cast<int>(std::floor((std::atan2(y, x) + M_PI) / inc));
        b = std::clamp(b, 0, bins_ - 1);
        if (rr < ranges[b]) ranges[b] = static_cast<float>(rr);
      }
    }

    if (used == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "유효한 라이다 스캔 없음 → /scan 발행 중단");
      return;
    }
    if (used < static_cast<int>(latest_.size()))
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "라이다 %d/%zu 개만 사용 중", used, latest_.size());
    if (!have_fp_)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "/swerve/footprint 미수신 → 기본 사각 ±%.3f m 로 자기 몸 제거", default_half_);

    sensor_msgs::msg::LaserScan out;
    out.header.stamp = newest;
    out.header.frame_id = out_frame_;
    out.angle_min = -M_PI;
    out.angle_increment = inc;
    out.angle_max = -M_PI + inc * (bins_ - 1);
    out.time_increment = 0.0;
    out.scan_time = 0.1;
    out.range_min = 0.0;   // 출력은 로봇 중심 기준 거리라 원래 range_min 의미 없음 (몸 제거로 최소 ≈ 0.22 m)
    out.range_max = range_max_;
    out.ranges = std::move(ranges);
    pub_->publish(out);
    RCLCPP_DEBUG(get_logger(), "used %d kept %zu removed %zu", used, kept, removed);
  }

  std::vector<std::string> inputs_, frames_;
  std::string out_frame_;
  int bins_;
  double range_min_, range_max_, margin_, default_half_, max_age_;
  std::vector<sensor_msgs::msg::LaserScan::ConstSharedPtr> latest_;
  std::vector<std::optional<Tf2D>> tf_cache_;
  std::vector<P2> footprint_;
  bool have_fp_{false};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> subs_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_fp_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanMergerNode>());
  rclcpp::shutdown();
  return 0;
}
