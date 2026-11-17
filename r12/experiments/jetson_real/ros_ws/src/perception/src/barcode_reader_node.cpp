// =====================================================================
// barcode_reader_node  (NZ2 인지)
//   역할: 그리퍼 카메라 영상 → 벽 바코드(Code 128) 판독 → 정렬 방향(오름/내림) 결정에 사용
//   입력: image_topic (sensor_msgs/Image, 기본 /gripper/color/image_raw)
//   출력: /perception/barcodes  (nz2_interfaces/BarcodeArray) → perception_master, 미션 노드
//
//   [수정 2026-10-28] 미션 v5: 바코드 = Code 128 문자열 "P2-01" ~ "P2-04" (1D)
//     - 예전 초안은 OpenCV QRCodeDetector 만 써서 1D 바코드를 아예 못 읽었음 → ZBar 로 교체
//       (빌드 시 libzbar 를 찾으면 ZBar, 없으면 QR 만 되는 예전 방식 + 시작 시 오류 로그)
//       Jetson/노트북 설치: sudo apt install libzbar-dev
//     - 전면 카메라 삭제 → 기본 토픽 /gripper/color/image_raw
//     - 확정: 같은 문자열이 confirm_reads 번 연속 판독돼야 confidence=1.0 (그 전엔 0.5) → 미션 노드는 1.0 만 사용
//     - valid_codes 에 없는 문자열은 버림 (벽의 다른 글자·오판독 방지). 빈 목록이면 전부 허용
//     - 작게 찍히면 못 읽으므로 실패 시 2배 확대해서 한 번 더 시도
//   픽셀 기준: Code 128 은 모듈(가장 가는 막대) 1개가 2 px 이상 찍혀야 안정적으로 읽힘.
//     예) 바코드 폭 60 mm(약 110 모듈), 1280 px 가로 화각 69° 기준 → 약 0.25 m 이내로 접근 [가정: 실물 폭 미측정]
//
//   ※ 이 노드의 포인트(레이트 제한): 영상은 latest_ 에 저장만 하고, 타이머가 초당 process_hz 번만 디코딩
// =====================================================================
#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nz2_interfaces/msg/barcode.hpp>
#include <nz2_interfaces/msg/barcode_array.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#ifdef NZ2_WITH_ZBAR
#include <zbar.h>
#endif

namespace nz2_perception
{

struct Decoded
{
  std::string data;
  std::string format;
};

class BarcodeReaderNode : public rclcpp::Node
{
public:
  BarcodeReaderNode()
  : rclcpp::Node("barcode_reader")
  {
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    process_hz_ = declare_parameter<double>("process_hz", 5.0, ro);
    const auto image_topic = declare_parameter<std::string>("image_topic", "/gripper/color/image_raw", ro);
    const bool image_reliable = declare_parameter<bool>("image_reliable", true, ro);
    confirm_reads_ = static_cast<int>(declare_parameter<int>("confirm_reads", 3, ro));
    valid_codes_ = declare_parameter<std::vector<std::string>>(
      "valid_codes", {"P2-01", "P2-02", "P2-03", "P2-04"}, ro);
    upscale_retry_ = declare_parameter<bool>("upscale_retry", true, ro);

    if (!(0.0 < process_hz_ && process_hz_ <= 30.0)) {throw std::invalid_argument("process_hz must be in (0, 30]");}
    if (confirm_reads_ < 1) {throw std::invalid_argument("confirm_reads must be >= 1");}

#ifdef NZ2_WITH_ZBAR
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);         // 전부 끄고
    scanner_.set_config(zbar::ZBAR_CODE128, zbar::ZBAR_CFG_ENABLE, 1);      // Code 128
    scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);       // QR (예비)
    const char * backend = "ZBar(Code128+QR)";
#else
    const char * backend = "OpenCV QR 전용";
    RCLCPP_ERROR(get_logger(), "ZBar 없이 빌드됨 → Code 128 바코드를 읽을 수 없음! "
      "sudo apt install libzbar-dev 후 colcon build --packages-select perception");
#endif

    pub_ = create_publisher<nz2_interfaces::msg::BarcodeArray>(
      "/perception/barcodes", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    auto img_qos = rclcpp::QoS(rclcpp::KeepLast(2));
    if (image_reliable) {img_qos.reliable();} else {img_qos.best_effort();}
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, img_qos, [this](sensor_msgs::msg::Image::ConstSharedPtr m) {latest_ = m;});
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / process_hz_), std::bind(&BarcodeReaderNode::process_latest, this));

    RCLCPP_INFO(get_logger(), "barcode_reader 시작: %s, %s, %.1f Hz, %d회 연속이면 확정 → /perception/barcodes",
      backend, image_topic.c_str(), process_hz_, confirm_reads_);
  }

private:
  std::vector<Decoded> decode(const cv::Mat & gray)
  {
    std::vector<Decoded> out;
#ifdef NZ2_WITH_ZBAR
    zbar::Image zimg(gray.cols, gray.rows, "Y800", gray.data, static_cast<unsigned long>(gray.cols * gray.rows));
    scanner_.scan(zimg);
    for (auto s = zimg.symbol_begin(); s != zimg.symbol_end(); ++s) {
      out.push_back({s->get_data(), s->get_type_name()});
    }
    zimg.set_data(nullptr, 0);   // 메모리는 cv::Mat 소유 → zbar 가 해제하지 않게
#else
    std::vector<std::string> texts;
    try {
      qr_.detectAndDecodeMulti(gray, texts);
    } catch (const cv::Exception &) {}
    for (const auto & t : texts) {if (!t.empty()) {out.push_back({t, "QRCODE"});}}
#endif
    return out;
  }

  bool valid(const std::string & s) const
  {
    return valid_codes_.empty() || std::find(valid_codes_.begin(), valid_codes_.end(), s) != valid_codes_.end();
  }

  void process_latest()
  {
    auto msg = latest_;
    if (!msg || msg == done_) {return;}   // 새 영상이 없으면 같은 영상을 다시 디코딩하지 않음
    done_ = msg;
    cv::Mat gray;
    try {
      gray = cv_bridge::toCvShare(msg, "mono8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "영상 변환 실패: %s", e.what());
      return;
    }
    if (gray.empty()) {return;}
    if (!gray.isContinuous()) {gray = gray.clone();}

    auto found = decode(gray);
    if (found.empty() && upscale_retry_) {   // 작게 찍힌 바코드: 2배 확대해서 한 번 더
      cv::Mat big;
      cv::resize(gray, big, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
      found = decode(big);
    }

    nz2_interfaces::msg::BarcodeArray out;
    out.header = msg->header;
    std::string best;
    for (const auto & f : found) {
      if (!valid(f.data)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "등록되지 않은 코드 무시: '%s' (%s)",
          f.data.c_str(), f.format.c_str());
        continue;
      }
      if (best.empty()) {best = f.data;}
      nz2_interfaces::msg::Barcode b;
      b.header = msg->header;
      b.data = f.data;
      b.format = f.format;
      b.position.x = b.position.y = b.position.z = std::numeric_limits<double>::quiet_NaN();
      out.codes.push_back(b);
    }
    // 연속 판독 확인: 같은 문자열이 confirm_reads 번 연속이면 확정(confidence=1.0)
    if (best.empty()) {
      streak_ = 0;
      streak_code_.clear();
    } else if (best == streak_code_) {
      ++streak_;
    } else {
      streak_code_ = best;
      streak_ = 1;
    }
    for (auto & b : out.codes) {
      b.confidence = (b.data == streak_code_ && streak_ >= confirm_reads_) ? 1.0f : 0.5f;
    }
    if (streak_ == confirm_reads_) {
      RCLCPP_INFO(get_logger(), "바코드 확정: '%s' (%d회 연속)", streak_code_.c_str(), streak_);
    }
    pub_->publish(out);
  }

  double process_hz_ = 5.0;
  int confirm_reads_ = 3;
  bool upscale_retry_ = true;
  std::vector<std::string> valid_codes_;
  int streak_ = 0;
  std::string streak_code_;
#ifdef NZ2_WITH_ZBAR
  zbar::ImageScanner scanner_;
#else
  cv::QRCodeDetector qr_;
#endif
  sensor_msgs::msg::Image::ConstSharedPtr latest_, done_;
  rclcpp::Publisher<nz2_interfaces::msg::BarcodeArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nz2_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<nz2_perception::BarcodeReaderNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("barcode_reader"), "시작 실패: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
