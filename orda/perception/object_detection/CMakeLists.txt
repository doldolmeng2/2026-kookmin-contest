cmake_minimum_required(VERSION 3.8)
project(object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(object_node src/object_detection.cpp)
ament_target_dependencies(object_node rclcpp std_msgs sensor_msgs cv_bridge)

target_include_directories(object_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(object_node ${OpenCV_LIBS})

install(TARGETS object_node
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/object_yolo_node.py
  DESTINATION lib/${PROJECT_NAME})

# best_simplified.onnx 는 이 브랜치에 없고 로드하는 코드도 없다.
# 목록에 남겨 두면 install 단계에서 통째로 실패한다.
install(FILES
  best.onnx
  DESTINATION share/${PROJECT_NAME}/model)

ament_package()
