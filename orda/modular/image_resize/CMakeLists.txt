cmake_minimum_required(VERSION 3.8)
project(image_resize)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

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