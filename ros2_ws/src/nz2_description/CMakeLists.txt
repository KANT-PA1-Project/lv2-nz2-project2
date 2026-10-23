# =============================================================
# 패키지 : nz2_description
# 파일   : CMakeLists.txt
# 역할   : urdf/ · meshes/ · launch/ 폴더를 설치한다 (빌드할 코드는 없음)
# =============================================================
cmake_minimum_required(VERSION 3.8)
project(nz2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
