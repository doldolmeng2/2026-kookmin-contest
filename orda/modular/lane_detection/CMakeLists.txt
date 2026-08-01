cmake_minimum_required(VERSION 3.8)
project(lane_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# OpenCV 4.5.4 설치 경로
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nlohmann_json REQUIRED)

# 실행 파일 빌드
add_executable(lane_node
  src/lane_detection.cpp
  src/parameter_loader.cpp
)
add_executable(hls_hsv_tuner_node src/hls_hsv_tuner.cpp)

# ROS2 의존성 연결
ament_target_dependencies(lane_node rclcpp std_msgs sensor_msgs cv_bridge)
ament_target_dependencies(hls_hsv_tuner_node rclcpp sensor_msgs cv_bridge)

# include 경로 추가
target_include_directories(lane_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 실행 파일 설치
install(TARGETS
  lane_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS hls_hsv_tuner_node
  DESTINATION lib/${PROJECT_NAME})

# OpenCV 연결
target_include_directories(lane_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lane_node ${OpenCV_LIBS})
target_link_libraries(hls_hsv_tuner_node ${OpenCV_LIBS})

target_link_libraries(lane_node nlohmann_json::nlohmann_json)

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