// =============================================================
// 패키지 : nz2_perception
// 파일   : include/nz2_perception/tag_detector.hpp
// 역할   : AprilTag(tag36h11) 검출 + 크기를 아는 태그의 3D 자세(solvePnP) 계산. ROS 와 무관
//          OpenCV 4.7+ : objdetect 의 cv::aruco::ArucoDetector
//          OpenCV 4.6- : contrib aruco 모듈 (없으면 available() == false, 태그 기능만 꺼짐)
// 태그 좌표계 (OpenCV ArUco 규약): 원점 = 태그 중심, z = 태그 면에서 카메라 쪽으로 나오는 법선,
//                               x = 태그 오른쪽, y = 태그 위쪽
// =============================================================
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace nz2_perception
{

struct TagObservation
{
  int id = -1;
  std::vector<cv::Point2f> corners;   // 4개, 영상 픽셀 (좌상 → 우상 → 우하 → 좌하)
  cv::Point2f center() const;
  float side_px() const;               // 평균 한 변 길이 (픽셀)
};

class TagDetector
{
public:
  TagDetector();
  ~TagDetector();
  bool available() const;
  // gray(또는 BGR) 영상에서 tag36h11 검출. roi 를 주면 그 안에서만 찾고 좌표는 전체 영상 기준으로 돌려준다
  std::vector<TagObservation> detect(const cv::Mat & img, const cv::Rect & roi = cv::Rect()) const;

  // 태그 1개의 카메라 좌표계 자세. size_m = 검은 사각형 한 변 (m). 성공하면 rvec/tvec (m)
  static bool estimate_pose(
    const TagObservation & tag, double size_m, const cv::Matx33d & K, const std::vector<double> & dist,
    cv::Vec3d & rvec, cv::Vec3d & tvec);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nz2_perception
