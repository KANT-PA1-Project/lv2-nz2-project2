// 스워브 기구학 단위 테스트 (ROS 불필요). 실패 시 종료코드 1
// 빌드: colcon build --packages-select nz2_swerve_nav && colcon test --packages-select nz2_swerve_nav
// 단독: g++ -std=c++17 -I include test/test_swerve_kinematics.cpp -o t && ./t
#include <cstdio>
#include <random>
#include "nz2_swerve_nav/swerve_kinematics.hpp"

using namespace nz2_swerve_nav;

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { std::printf("FAIL %s:%d ", __FILE__, __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); ++g_fail; } } while (0)

// assembly_nz2.urdf 값 (순서: front_left, front_right, rear_left, rear_right — URDF 이름)
static SwerveParams urdfParams() {
  SwerveParams p;
  const double P[4][2] = {{0.124, 0.124}, {0.124, -0.124}, {-0.124, 0.124}, {-0.124, -0.124}};
  const double C[4][2] = {{0.019183, 0.001219}, {0.019248, -0.001252}, {-0.019248, 0.001252}, {-0.019183, -0.001219}};
  for (int i = 0; i < 4; ++i) {
    p.modules[i].pos = {P[i][0], P[i][1]};
    p.modules[i].box_center = {C[i][0], C[i][1]};
    p.modules[i].box_half = {0.0747, 0.0572};
  }
  return p;
}

static std::array<ModuleState, 4> measFrom(const std::array<ModuleCommand, 4> & c) {
  std::array<ModuleState, 4> m{};
  for (int i = 0; i < 4; ++i) m[i] = {c[i].steer, c[i].wheel_vel};
  return m;
}

// 조향이 이미 맞춰진 상태로 수렴시키기 (게이트 통과용)
static std::array<ModuleCommand, 4> settle(SwerveKinematics & k, const Twist2D & t) {
  std::array<ModuleState, 4> meas{};
  std::array<ModuleCommand, 4> c{};
  for (int it = 0; it < 3; ++it) { c = k.inverse(t, meas, true); meas = measFrom(c); }
  return c;
}

static double width(const std::vector<Vec2> & h, double travel) {
  const double nx = -std::sin(travel), ny = std::cos(travel);
  double lo = 1e9, hi = -1e9;
  for (auto & q : h) { const double s = q.x * nx + q.y * ny; lo = std::min(lo, s); hi = std::max(hi, s); }
  return hi - lo;
}

int main() {
  // 1) 왕복: inverse → forward 가 원래 속도를 복원
  {
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> u(-0.5, 0.5);
    for (int n = 0; n < 2000; ++n) {
      SwerveKinematics k(urdfParams());
      const Twist2D t{u(rng), u(rng), 2.0 * u(rng)};
      const auto c = settle(k, t);
      const Twist2D f = k.forward(measFrom(c));
      // 포화로 줄어든 경우 방향만 같으면 됨 → 비율로 비교
      const double s = (std::fabs(t.vx) + std::fabs(t.vy) + std::fabs(t.wz)) > 1e-6
          ? (std::fabs(f.vx) + std::fabs(f.vy) + std::fabs(f.wz)) / (std::fabs(t.vx) + std::fabs(t.vy) + std::fabs(t.wz)) : 1.0;
      CHECK(std::fabs(f.vx - s * t.vx) < 1e-6 && std::fabs(f.vy - s * t.vy) < 1e-6 && std::fabs(f.wz - s * t.wz) < 1e-6,
            "roundtrip n=%d t=(%.3f %.3f %.3f) f=(%.3f %.3f %.3f)", n, t.vx, t.vy, t.wz, f.vx, f.vy, f.wz);
      for (auto & m : c) CHECK(std::fabs(m.steer) <= 1.92 + 1e-9, "steer limit");
      for (auto & m : c) CHECK(std::fabs(m.linear) <= 0.95 + 1e-9, "wheel limit");
    }
  }
  // 2) 정면(+x_fp) 전진: base_link 에서 -x 이므로 조향 0, 바퀴 음수
  {
    SwerveKinematics k(urdfParams());
    const auto c = settle(k, {0.3, 0, 0});
    for (auto & m : c) CHECK(std::fabs(m.steer) < 1e-9 && m.linear < 0, "forward steer=%.3f lin=%.3f", m.steer, m.linear);
  }
  // 3) 한계 근처 흔들림(옆 게걸음 ±2°)에서 180° 뒤집힘 없음 — 한계 ±90° 로도 확인
  for (double lim : {1.92, 1.5708}) {
    auto p = urdfParams(); p.steer_limit = lim;
    SwerveKinematics k(p);
    settle(k, {0, 0.3, 0});
    double prev[4]; for (int i = 0; i < 4; ++i) prev[i] = k.lastCommand()[i].steer;
    std::array<ModuleState, 4> meas{}; for (int i = 0; i < 4; ++i) meas[i].steer = prev[i];
    for (int n = 0; n < 200; ++n) {
      const double a = M_PI / 2 + ((n % 2) ? 1 : -1) * 0.035;
      const auto c = k.inverse({0.3 * std::cos(a), 0.3 * std::sin(a), 0}, meas, true);
      for (int i = 0; i < 4; ++i) {
        CHECK(std::fabs(c[i].steer - prev[i]) < 0.1, "flip lim=%.2f i=%d %.3f->%.3f", lim, i, prev[i], c[i].steer);
        meas[i].steer = c[i].steer;
      }
    }
  }
  // 4) 정렬 게이트: 조향 오차 30° 면 바퀴 0, 측정 무효면 바퀴 0
  {
    SwerveKinematics k(urdfParams());
    std::array<ModuleState, 4> meas{};
    for (auto & m : meas) m.steer = 0.52;  // 30°
    double a = -1;
    auto c = k.inverse({0.3, 0, 0}, meas, true, &a);
    CHECK(a == 0.0, "gate align=%.3f", a);
    for (auto & m : c) CHECK(m.wheel_vel == 0.0, "gate wheel");
    c = k.inverse({0.3, 0, 0}, meas, false, &a);
    for (auto & m : c) CHECK(m.wheel_vel == 0.0, "invalid meas wheel");
  }
  // 5) 정지 시 조향 유지
  {
    SwerveKinematics k(urdfParams());
    const auto c1 = settle(k, {0, 0.3, 0});
    const auto c2 = k.inverse({0, 0, 0}, measFrom(c1), true);
    for (int i = 0; i < 4; ++i) CHECK(c2[i].steer == c1[i].steer && c2[i].wheel_vel == 0.0, "hold");
  }
  // 6) 외곽 폭 (진행 방향에 수직)
  {
    SwerveKinematics k(urdfParams());
    for (double deg : {0.0, 90.0, 45.0}) {
      const double tr = deg * M_PI / 180.0;
      const auto c = settle(k, {0.3 * std::cos(tr), 0.3 * std::sin(tr), 0});
      std::array<double, 4> s; for (int i = 0; i < 4; ++i) s[i] = c[i].steer;
      const auto h = k.footprint({s}, 0.0);
      std::printf("travel %4.0f deg: width %.4f m (corridor 0.55 -> side clearance %.1f mm)\n",
                  deg, width(h, tr), (0.55 - width(h, tr)) / 2 * 1000);
    }
    const double w0 = width(k.footprint({{0, 0, 0, 0}}, 0.0), 0.0);
    CHECK(std::fabs(w0 - 0.365) < 0.002, "width0 %.4f", w0);
    // 라이다 하우징 반폭 20 mm [가정] 포함
    auto p = urdfParams();
    const double L[4][2] = {{0.189, 0}, {0, 0.189}, {-0.189, 0}, {0, -0.189}};
    for (auto & c : L) for (int sx : {-1, 1}) for (int sy : {-1, 1}) p.extra_points.push_back({c[0] + sx * 0.02, c[1] + sy * 0.02});
    SwerveKinematics kl(p);
    const double wl = width(kl.footprint({{0, 0, 0, 0}}, 0.0), 0.0);
    std::printf("with lidar housing 20 mm: width %.4f m (side clearance %.1f mm)\n", wl, (0.55 - wl) / 2 * 1000);
    CHECK(std::fabs(wl - 0.418) < 0.002, "width lidar %.4f", wl);
  }
  // 7) 제자리 회전 시 모듈이 접선 방향
  {
    SwerveKinematics k(urdfParams());
    const auto c = settle(k, {0, 0, 1.0});
    const auto f = k.forward(measFrom(c));
    CHECK(std::fabs(f.vx) < 1e-9 && std::fabs(f.vy) < 1e-9 && f.wz > 0, "spin");
  }
  std::printf(g_fail ? "FAILED %d\n" : "ALL PASS\n", g_fail);
  return g_fail ? 1 : 0;
}
