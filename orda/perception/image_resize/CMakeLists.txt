cmake_minimum_required(VERSION 3.8)
project(image_resize)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# /usr/local 에 시스템 전역으로 설치된 OpenCV 4.11(cv::dnn 이 YOLOv8 을 못 돌리는
# ROS 기본 4.5.4 문제를 우회하려고 별도로 빌드된 것)이 cv_bridge(4.5.4 기준으로
# 빌드됨)보다 CMake 탐색 우선순위가 높다. 버전을 안 박으면 이 패키지가 4.11 헤더로
# 컴파일되고 cv_bridge 가 넘겨준 cv::Mat(4.5.4 레이아웃)을 4.11 레이아웃으로 잘못
# 해석해서 크기가 깨진다 — cv::resize 호출에서 setSize assertion(s>=0)으로 죽거나,
# 죽지 않고 조용히 잘못된 크롭이 나올 수도 있다. 실차(J4012)에는 4.11 이 없어서
# 안 나던 문제라 개발 PC 에서만 재현된다.
#
# find_package(cv_bridge) 보다 반드시 먼저 와야 한다 — cv_bridge 의 cmake config
# 가 내부적으로 find_package(OpenCV) 를 호출하는데, 그게 먼저 실행되면 버전
# 제약 없이 기본 탐색 경로(4.11 우선)로 이미 캐시돼서 아래 지정이 무시된다.
# 경로에 아키텍처를 박으면 실차(J4012, aarch64)에서 깨진다. 그 머신에는
# /usr/lib/x86_64-linux-gnu 가 없고 OpenCV config 가 /usr/lib/cmake/opencv4 에
# 있다. NO_DEFAULT_PATH 는 그대로 두어 /usr/local 4.11 은 계속 배제한다.
find_package(OpenCV REQUIRED PATHS
  /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/opencv4
  /usr/lib/cmake/opencv4
  NO_DEFAULT_PATH)
find_package(cv_bridge REQUIRED)

# 실행 파일 빌드
add_executable(resize_node src/resize.cpp)

# ROS2 의존성 연결
ament_target_dependencies(resize_node rclcpp std_msgs sensor_msgs cv_bridge)

# include 경로 추가
target_include_directories(resize_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 실행 파일 설치
install(TARGETS
  resize_node
  DESTINATION lib/${PROJECT_NAME})

# OpenCV 연결
target_include_directories(resize_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(resize_node ${OpenCV_LIBS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()