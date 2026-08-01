cmake_minimum_required(VERSION 3.8)
project(object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenCV_DIR "/usr/local/lib/cmake/opencv4")

find_package(ament_cmake REQUIRED)
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

ament_package()
