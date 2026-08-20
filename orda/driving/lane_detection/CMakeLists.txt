cmake_minimum_required(VERSION 3.8)
project(lane_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# cv_bridge(4.5.4 기준 빌드)와 ABI 가 다른 /usr/local 의 OpenCV 4.11 이 잡히지
# 않도록 버전을 고정한다. 자세한 이유는 image_resize/CMakeLists.txt 참고.
# 경로에 아키텍처를 박으면 실차(J4012, aarch64)에서 깨진다. 그 머신에는
# /usr/lib/x86_64-linux-gnu 가 없고 OpenCV config 가 /usr/lib/cmake/opencv4 에
# 있다. NO_DEFAULT_PATH 는 그대로 두어 /usr/local 4.11 은 계속 배제한다.
find_package(OpenCV REQUIRED PATHS
  /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/opencv4
  /usr/lib/cmake/opencv4
  NO_DEFAULT_PATH)
find_package(cv_bridge REQUIRED)
find_package(nlohmann_json REQUIRED)

# 실행 파일 빌드
add_executable(lane_node
  src/lane_detection.cpp
  src/parameter_loader.cpp
)
add_executable(hls_hsv_tuner_node src/hls_hsv_tuner.cpp)

# ROS2 의존성 연결
ament_target_dependencies(
  lane_node ament_index_cpp rclcpp diagnostic_msgs std_msgs sensor_msgs cv_bridge)
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
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_lane_change_state_tracker
    test/test_lane_change_state_tracker.cpp)
  target_include_directories(test_lane_change_state_tracker PRIVATE include)

  ament_add_gtest(test_lane_measurement_publication_policy
    test/test_lane_measurement_publication_policy.cpp)
  target_include_directories(test_lane_measurement_publication_policy PRIVATE include)

  ament_add_gtest(test_lane_pipeline_diagnostics
    test/test_lane_pipeline_diagnostics.cpp)
  target_include_directories(test_lane_pipeline_diagnostics PRIVATE include)
  ament_target_dependencies(test_lane_pipeline_diagnostics diagnostic_msgs std_msgs)

  ament_add_gtest(test_reacquire_histogram
    test/test_reacquire_histogram.cpp)
  target_include_directories(test_reacquire_histogram PRIVATE
    include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_reacquire_histogram ${OpenCV_LIBS})
endif()

install(DIRECTORY include/ DESTINATION include)
install(FILES lane_detection_parameter.json DESTINATION share/${PROJECT_NAME})

ament_package()
