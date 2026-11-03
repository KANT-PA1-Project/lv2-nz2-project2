// =============================================================
// 패키지 : nz2_perception
// 파일   : src/tag_detector.cpp
// 역할   : AprilTag(tag36h11) 검출 래퍼. OpenCV 버전별 API 차이를 여기서만 처리
// =============================================================
#include "nz2_perception/tag_detector.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#if __has_include(<opencv2/objdetect/aruco_detector.hpp>)
#include <opencv2/objdetect/aruco_detector.hpp>
#define NZ2_ARUCO_NEW 1
#elif __has_include(<opencv2/aruco.hpp>)
#include <opencv2/aruco.hpp>
#define NZ2_ARUCO_OLD 1
#endif

namespace nz2_perception
{

cv::Point2f TagObservation::center() const
{
  cv::Point2f c(0, 0);
  for (const auto & p : corners) {c += p;}
  return corners.empty() ? c : c * (1.0f / corners.size());
}

float TagObservation::side_px() const
{
  if (corners.size() != 4) {return 0.0f;}
  float s = 0;
  for (int i = 0; i < 4; ++i) {s += static_cast<float>(cv::norm(corners[i] - corners[(i + 1) % 4]));}
  return s / 4.0f;
}

struct TagDetector::Impl
{
#if defined(NZ2_ARUCO_NEW)
  cv::aruco::ArucoDetector det;
  Impl()
  {
    cv::aruco::DetectorParameters p;
    p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;   // 작은 태그(16mm) 자세 정확도↑
    det = cv::aruco::ArucoDetector(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11), p);
  }
#elif defined(NZ2_ARUCO_OLD)
  cv::Ptr<cv::aruco::Dictionary> dict;
  cv::Ptr<cv::aruco::DetectorParameters> params;
  Impl()
  {
    dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    params = cv::aruco::DetectorParameters::create();
    params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  }
#endif
};

TagDetector::TagDetector()
: impl_(std::make_unique<Impl>()) {}

TagDetector::~TagDetector() = default;

bool TagDetector::available() const
{
#if defined(NZ2_ARUCO_NEW) || defined(NZ2_ARUCO_OLD)
  return true;
#else
  return false;
#endif
}

std::vector<TagObservation> TagDetector::detect(const cv::Mat & img, const cv::Rect & roi_in) const
{
  std::vector<TagObservation> out;
#if defined(NZ2_ARUCO_NEW) || defined(NZ2_ARUCO_OLD)
  if (img.empty()) {return out;}
  cv::Rect roi = roi_in.area() > 0 ? (roi_in & cv::Rect(0, 0, img.cols, img.rows)) : cv::Rect(0, 0, img.cols, img.rows);
  if (roi.area() <= 0) {return out;}
  cv::Mat gray;
  if (img.channels() == 3) {cv::cvtColor(img(roi), gray, cv::COLOR_BGR2GRAY);} else {gray = img(roi);}

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
#if defined(NZ2_ARUCO_NEW)
  impl_->det.detectMarkers(gray, corners, ids);
#else
  cv::aruco::detectMarkers(gray, impl_->dict, corners, ids, impl_->params);
#endif
  const cv::Point2f off(static_cast<float>(roi.x), static_cast<float>(roi.y));
  for (size_t i = 0; i < ids.size(); ++i) {
    TagObservation t;
    t.id = ids[i];
    for (const auto & p : corners[i]) {t.corners.push_back(p + off);}
    out.push_back(t);
  }
#else
  (void)img;
  (void)roi_in;
#endif
  return out;
}

bool TagDetector::estimate_pose(
  const TagObservation & tag, double size_m, const cv::Matx33d & K, const std::vector<double> & dist,
  cv::Vec3d & rvec, cv::Vec3d & tvec)
{
  if (tag.corners.size() != 4 || size_m <= 0) {return false;}
  const float h = static_cast<float>(size_m / 2);
  // ArUco 코너 순서(좌상, 우상, 우하, 좌하)에 맞춘 태그 평면 좌표 (z=0)
  const std::vector<cv::Point3f> obj = {{-h, h, 0}, {h, h, 0}, {h, -h, 0}, {-h, -h, 0}};
  try {
    return cv::solvePnP(obj, tag.corners, K, dist, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
  } catch (const cv::Exception &) {
    return false;
  }
}

}  // namespace nz2_perception
