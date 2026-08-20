cmake_minimum_required(VERSION 3.8)
project(object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# cv_bridge(4.5.4 기준 빌드)와 ABI 가 다른 /usr/local 의 OpenCV 4.11 이 잡히지
# 않도록 버전을 고정한다. classifyLight() 의 cv::dnn::readNet(light_cls.onnx) 는
# 4.5.4 에서 문제없이 검증됐다(object_detection.cpp 참고) — 4.11 이 필요했던 건
# best.onnx YOLO 검출기 쪽인데 그건 use_external_yolo_=true 로 이미 꺼져 있다.
# 자세한 이유는 image_resize/CMakeLists.txt 참고.
# 경로에 아키텍처를 박으면 실차(J4012, aarch64)에서 깨진다. 그 머신에는
# /usr/lib/x86_64-linux-gnu 가 없고 OpenCV config 가 /usr/lib/cmake/opencv4 에
# 있다. NO_DEFAULT_PATH 는 그대로 두어 /usr/local 4.11 은 계속 배제한다.
find_package(OpenCV REQUIRED PATHS
  /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/opencv4
  /usr/lib/cmake/opencv4
  NO_DEFAULT_PATH)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(object_node src/object_detection.cpp)
ament_target_dependencies(
  object_node ament_index_cpp rclcpp std_msgs sensor_msgs cv_bridge)

target_include_directories(object_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(object_node ${OpenCV_LIBS})

install(TARGETS object_node
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/object_yolo_node.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  best.onnx
  model/train10_detector_best.onnx
  model/light1_classifier_best.onnx
  light_cls.onnx
  DESTINATION share/${PROJECT_NAME}/model)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lane_stabilizer test/test_lane_stabilizer.cpp)
  target_include_directories(test_lane_stabilizer PRIVATE include)
  ament_add_gtest(test_side_clearance test/test_side_clearance.cpp)
  target_include_directories(test_side_clearance PRIVATE include)
endif()

ament_package()
