// nz2_swerve_nav/swerve_kinematics.hpp
// 스워브 역기구학·순기구학·외곽(footprint) 계산 — ROS 의존성 없는 순수 C++ (단위 테스트 가능)
//
// 좌표 약속
//  - 내부 계산은 전부 URDF base_link 좌표 (모듈 위치·조향 관절각이 URDF 와 같은 값)
//  - 외부 입출력(cmd_vel, odom, footprint)은 base_footprint 좌표 (+X = 로봇 정면 = 팔 쪽)
//  - base_footprint 에서 본 base_link 의 yaw = base_yaw (최종 URDF: pi)
//
// 통로 여유를 늘리는 핵심 (assembly_nz2.urdf 충돌상자 기준 계산값)
//  - 모듈 상자 긴 변(0.149 m)이 바퀴 굴림 방향과 같음 → 바퀴를 진행 방향으로 맞추면
//    진행 방향에 수직인 폭 = 0.365 m (정사각 0.436 m 대비 71 mm 감소)
//    단, 변 중앙 라이다(±0.189 m) 하우징이 그보다 튀어나오면 폭 = 2×(0.189 + 하우징 반폭)
//  - 대각선 진행(45°)이면 폭 0.481 m → 통로(0.55 m)에서 금지. 차체 yaw 는 통로 축에 맞추고 게걸음
//  - 제자리 회전 원 반지름 0.252 m (지름 0.504 m) → 통로 중앙에서만 가능 (한쪽 여유 23 mm)
//  - 조향 중에 바퀴가 돌면 차체가 옆으로 밀려 벽에 닿음 → 조향 오차가 크면 바퀴 정지(정렬 게이트)
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace nz2_swerve_nav {

constexpr std::size_t kNumModules = 4;

inline double wrapPi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

struct Vec2 {
  double x{0.0}, y{0.0};
};

inline Vec2 rot(const Vec2 & v, double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  return {c * v.x - s * v.y, s * v.x + c * v.y};
}

struct Twist2D {
  double vx{0.0}, vy{0.0}, wz{0.0};  // m/s, m/s, rad/s
};

// 모듈 1개 기하·보정값 (URDF base_link 기준)
struct ModuleGeom {
  Vec2 pos;                 // 조향축 위치 [m]
  double steer_offset{0.0}; // 실물 보정: (실제 굴림 방향) = 관절각 + steer_offset  [rad]
  double wheel_sign{1.0};   // 실물 보정: 바퀴 +회전이 굴림 방향 +로 가면 +1, 반대면 -1
  // 충돌상자 (조향 프레임 기준, footprint 계산용)
  Vec2 box_center;          // 조향축에서 상자 중심까지 [m]
  Vec2 box_half;            // 상자 반폭 (굴림 방향 x, 옆 방향 y) [m]
};

struct SwerveParams {
  std::array<ModuleGeom, kNumModules> modules;
  double base_yaw{M_PI};          // base_footprint → base_link yaw
  double wheel_radius{0.033};     // [m]
  double max_wheel_speed{0.95};   // 바퀴 선속도 상한 [m/s]
  double steer_limit{1.92};       // 조향 관절 한계 ± [rad]
  double clamp_tolerance{0.17};   // 한계에 걸려 생기는 방향 오차 허용 [rad] (≈10°)
  double align_stop{0.35};        // 이보다 조향 오차가 크면 바퀴 정지 [rad] (≈20°)
  double min_module_speed{0.005}; // 이보다 느리면 조향 유지, 바퀴 0 [m/s]
  Vec2 plate_half{0.17, 0.17};    // 차체 판 반폭 (base_link) [m]
  // 판·모듈 밖으로 나온 고정 부품 꼭짓점 (base_link) — 라이다 하우징 등. footprint 에 항상 포함
  std::vector<Vec2> extra_points;
};

struct ModuleCommand {
  double steer{0.0};       // 조향 관절 목표 [rad]
  double wheel_vel{0.0};   // 바퀴 관절 각속도 [rad/s]
  double linear{0.0};      // 바퀴 선속도 (부호 포함) [m/s] — 디버그용
};

struct ModuleState {
  double steer{0.0};       // 측정 조향각 [rad]
  double wheel_vel{0.0};   // 측정 바퀴 각속도 [rad/s]
};

// 조향 후보 선택 결과
struct SteerChoice {
  double steer{0.0};
  double sign{1.0};        // 바퀴 선속도 부호 (+1 / -1)
  double clamp_err{0.0};   // 한계로 잘려 생긴 방향 오차 [rad]
};

// 굴림 방향 heading(base_link 기준)을 만족하는 관절각 중
//  (1) 한계 안(오차 ≤ clamp_tolerance) (2) 이전 명령 prev 에서 가장 적게 도는 것 을 고른다.
//  → θ 와 θ±π(바퀴 역회전)를 함께 보므로 조향은 최대 90° 만 돌고, 한계 근처에서 180° 뒤집힘 반복을 막는다.
inline SteerChoice chooseSteer(double heading, double steer_offset, double prev,
                               double limit, double clamp_tol) {
  SteerChoice best;
  double best_cost = std::numeric_limits<double>::infinity();
  bool found = false;
  const double base = wrapPi(heading - steer_offset);
  for (int k = -2; k <= 2; ++k) {
    const double cand = base + k * M_PI;
    const double clamped = std::clamp(cand, -limit, limit);
    const double err = std::fabs(cand - clamped);
    if (err > clamp_tol) continue;
    const double cost = std::fabs(clamped - prev) + 2.0 * err;
    if (cost < best_cost) {
      best_cost = cost;
      best = {clamped, (k % 2 == 0) ? 1.0 : -1.0, err};
      found = true;
    }
  }
  if (!found) {  // 한계가 ±90° 보다 많이 좁은 경우만 해당: 가장 가까운 한계값, 바퀴는 정렬 게이트가 막음
    best = {std::clamp(base, -limit, limit), 1.0, M_PI};
  }
  return best;
}

class SwerveKinematics {
public:
  explicit SwerveKinematics(const SwerveParams & p) : p_(p) {
    for (auto & c : last_cmd_) c = ModuleCommand{};
  }

  const SwerveParams & params() const { return p_; }
  const std::array<ModuleCommand, kNumModules> & lastCommand() const { return last_cmd_; }

  // 역기구학: base_footprint 속도 → 모듈 명령
  //  meas: 측정 조향각 (정렬 게이트용). meas_valid=false 면 바퀴 0
  //  align_scale_out: 정렬 게이트로 적용된 배율 (0~1)
  std::array<ModuleCommand, kNumModules> inverse(const Twist2D & cmd_fp,
                                                 const std::array<ModuleState, kNumModules> & meas,
                                                 bool meas_valid, double * align_scale_out = nullptr) {
    const Vec2 v_bl = rot({cmd_fp.vx, cmd_fp.vy}, -p_.base_yaw);
    const double wz = cmd_fp.wz;
    std::array<ModuleCommand, kNumModules> out{};
    double max_lin = 0.0;
    for (std::size_t i = 0; i < kNumModules; ++i) {
      const auto & m = p_.modules[i];
      const double vx = v_bl.x - wz * m.pos.y;
      const double vy = v_bl.y + wz * m.pos.x;
      const double speed = std::hypot(vx, vy);
      if (speed < p_.min_module_speed) {
        out[i].steer = last_cmd_[i].steer;  // 정지 시 조향 유지 (불필요한 회전 방지)
        out[i].linear = 0.0;
        continue;
      }
      const SteerChoice c = chooseSteer(std::atan2(vy, vx), m.steer_offset, last_cmd_[i].steer,
                                        p_.steer_limit, p_.clamp_tolerance);
      out[i].steer = c.steer;
      out[i].linear = c.sign * speed * std::max(0.0, std::cos(c.clamp_err));
      max_lin = std::max(max_lin, std::fabs(out[i].linear));
    }
    // 포화: 한 바퀴라도 상한을 넘으면 전체를 같은 비율로 줄임 (진행 방향 유지)
    double scale = 1.0;
    if (max_lin > p_.max_wheel_speed && max_lin > 0.0) scale = p_.max_wheel_speed / max_lin;
    // 정렬 게이트: 모든 모듈에 같은 배율 → 차체 궤적이 휘지 않음
    double align = meas_valid ? 1.0 : 0.0;
    if (meas_valid) {
      for (std::size_t i = 0; i < kNumModules; ++i) {
        if (out[i].linear == 0.0) continue;
        const double e = std::fabs(wrapPi(out[i].steer - meas[i].steer));
        if (e > p_.align_stop) { align = 0.0; break; }
        align = std::min(align, std::cos(e));
      }
    }
    if (align_scale_out) *align_scale_out = align;
    for (std::size_t i = 0; i < kNumModules; ++i) {
      out[i].linear *= scale * align;
      out[i].wheel_vel = out[i].linear / (p_.wheel_radius * p_.modules[i].wheel_sign);
    }
    last_cmd_ = out;
    return out;
  }

  // X 잠금 (정지 상태에서 팔 작업 시 밀림 방지): 바퀴를 중심 방향(대각 45°)으로, 바퀴 0
  std::array<ModuleCommand, kNumModules> xLock() {
    std::array<ModuleCommand, kNumModules> out{};
    for (std::size_t i = 0; i < kNumModules; ++i) {
      const auto & m = p_.modules[i];
      const SteerChoice c = chooseSteer(std::atan2(m.pos.y, m.pos.x), m.steer_offset,
                                        last_cmd_[i].steer, p_.steer_limit, p_.clamp_tolerance);
      out[i].steer = c.steer;
    }
    last_cmd_ = out;
    return out;
  }

  // 정지 명령 (조향 유지, 바퀴 0)
  std::array<ModuleCommand, kNumModules> hold() {
    for (auto & c : last_cmd_) { c.wheel_vel = 0.0; c.linear = 0.0; }
    return last_cmd_;
  }

  // 순기구학 (최소제곱): 측정 → base_footprint 속도. residual_out = 모듈 속도 잔차 RMS [m/s] (미끄럼 지표)
  Twist2D forward(const std::array<ModuleState, kNumModules> & meas, double * residual_out = nullptr) const {
    // A^T A x = A^T b, A 행: [1 0 -y], [0 1 x]
    double ATA[3][3] = {{0}}, ATb[3] = {0};
    std::array<Vec2, kNumModules> vm{};
    for (std::size_t i = 0; i < kNumModules; ++i) {
      const auto & m = p_.modules[i];
      const double lin = meas[i].wheel_vel * p_.wheel_radius * m.wheel_sign;
      const double h = meas[i].steer + m.steer_offset;
      vm[i] = {lin * std::cos(h), lin * std::sin(h)};
      const double rows[2][3] = {{1.0, 0.0, -m.pos.y}, {0.0, 1.0, m.pos.x}};
      const double b[2] = {vm[i].x, vm[i].y};
      for (int r = 0; r < 2; ++r)
        for (int a = 0; a < 3; ++a) {
          ATb[a] += rows[r][a] * b[r];
          for (int c = 0; c < 3; ++c) ATA[a][c] += rows[r][a] * rows[r][c];
        }
    }
    double x[3];
    solve3(ATA, ATb, x);
    if (residual_out) {
      double s = 0.0;
      for (std::size_t i = 0; i < kNumModules; ++i) {
        const auto & m = p_.modules[i];
        const double ex = x[0] - x[2] * m.pos.y - vm[i].x;
        const double ey = x[1] + x[2] * m.pos.x - vm[i].y;
        s += ex * ex + ey * ey;
      }
      *residual_out = std::sqrt(s / kNumModules);
    }
    const Vec2 v_fp = rot({x[0], x[1]}, p_.base_yaw);
    return {v_fp.x, v_fp.y, x[2]};
  }

  // 외곽 다각형 (base_footprint 기준, 반시계 볼록 껍질)
  //  steer_sets: 여러 조향각 세트(측정값·명령값)를 합집합으로 → 조향 전환 중에도 안전
  std::vector<Vec2> footprint(const std::vector<std::array<double, kNumModules>> & steer_sets,
                              double padding) const {
    std::vector<Vec2> pts;
    const double px = p_.plate_half.x, py = p_.plate_half.y;
    for (int sx : {-1, 1})
      for (int sy : {-1, 1}) pts.push_back({sx * px, sy * py});
    for (const auto & set : steer_sets) {
      for (std::size_t i = 0; i < kNumModules; ++i) {
        const auto & m = p_.modules[i];
        for (int sx : {-1, 1})
          for (int sy : {-1, 1}) {
            const Vec2 local{m.box_center.x + sx * m.box_half.x, m.box_center.y + sy * m.box_half.y};
            const Vec2 r = rot(local, set[i]);
            pts.push_back({m.pos.x + r.x, m.pos.y + r.y});
          }
      }
    }
    for (const auto & q : p_.extra_points) pts.push_back(q);
    for (auto & q : pts) q = rot(q, p_.base_yaw);
    auto hull = convexHull(pts);
    if (padding > 0.0) {  // 꼭짓점을 중심에서 바깥으로 padding 만큼 (근사)
      for (auto & q : hull) {
        const double n = std::hypot(q.x, q.y);
        if (n > 1e-9) { q.x += padding * q.x / n; q.y += padding * q.y / n; }
      }
    }
    return hull;
  }

  static std::vector<Vec2> convexHull(std::vector<Vec2> p) {
    std::sort(p.begin(), p.end(), [](const Vec2 & a, const Vec2 & b) {
      return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    if (p.size() < 3) return p;
    auto cross = [](const Vec2 & o, const Vec2 & a, const Vec2 & b) {
      return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };
    std::vector<Vec2> h(2 * p.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < p.size(); ++i) {
      while (k >= 2 && cross(h[k - 2], h[k - 1], p[i]) <= 0) --k;
      h[k++] = p[i];
    }
    for (std::size_t i = p.size() - 1, t = k + 1; i > 0; --i) {
      while (k >= t && cross(h[k - 2], h[k - 1], p[i - 1]) <= 0) --k;
      h[k++] = p[i - 1];
    }
    h.resize(k - 1);
    return h;
  }

private:
  static void solve3(double A[3][3], double b[3], double x[3]) {
    // 가우스 소거 (부분 피벗). 모듈 4개면 A^T A 는 항상 정칙
    double M[3][4];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) M[r][c] = A[r][c];
      M[r][3] = b[r];
    }
    for (int c = 0; c < 3; ++c) {
      int piv = c;
      for (int r = c + 1; r < 3; ++r)
        if (std::fabs(M[r][c]) > std::fabs(M[piv][c])) piv = r;
      for (int k = 0; k < 4; ++k) std::swap(M[c][k], M[piv][k]);
      const double d = M[c][c];
      if (std::fabs(d) < 1e-12) { x[0] = x[1] = x[2] = 0.0; return; }
      for (int k = c; k < 4; ++k) M[c][k] /= d;
      for (int r = 0; r < 3; ++r) {
        if (r == c) continue;
        const double f = M[r][c];
        for (int k = c; k < 4; ++k) M[r][k] -= f * M[c][k];
      }
    }
    for (int r = 0; r < 3; ++r) x[r] = M[r][3];
  }

  SwerveParams p_;
  std::array<ModuleCommand, kNumModules> last_cmd_{};
};

}  // namespace nz2_swerve_nav
