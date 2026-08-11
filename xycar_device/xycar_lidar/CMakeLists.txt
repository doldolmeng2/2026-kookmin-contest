# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 3.5)
project(xycar_lidar C CXX)

##################ros2#############################################
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################find package#####################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

############## YDLIDAR SDK START#####################################
#find ydlidar_sdk package
find_package(ydlidar_sdk REQUIRED)

# Some bundled YDLidar SDK releases export the library name but leave
# YDLIDAR_SDK_LIBRARY_DIRS empty.  Derive the installed lib directory from
# <prefix>/lib/cmake/ydlidar_sdk when using such an SDK.
get_filename_component(
  YDLIDAR_SDK_CONFIG_LIB_DIR
  "${ydlidar_sdk_DIR}/../.."
  ABSOLUTE
)

# The bundled SDK config exports "ydlidar_sdk" by name but does not export
# its installed library directory. Resolve the actual library once and use
# its absolute path so the linker does not depend on a global -L path.
find_library(
  YDLIDAR_SDK_LIBRARY
  NAMES ydlidar_sdk
  PATHS "${YDLIDAR_SDK_CONFIG_LIB_DIR}"
  NO_DEFAULT_PATH
)

if(NOT YDLIDAR_SDK_LIBRARY)
  message(FATAL_ERROR
    "YDLidar SDK library not found in ${YDLIDAR_SDK_CONFIG_LIB_DIR}")
endif()

list(REMOVE_ITEM YDLIDAR_SDK_LIBRARIES ydlidar_sdk)
list(INSERT YDLIDAR_SDK_LIBRARIES 0 "${YDLIDAR_SDK_LIBRARY}")

message(STATUS "YDLidar SDK library: ${YDLIDAR_SDK_LIBRARY}")
############## YDLIDAR SDK END#####################################

#Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${YDLIDAR_SDK_INCLUDE_DIRS})

# YDLidar SDK is linked by the absolute library path resolved above.

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp)

#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_node
    "rclcpp"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "std_srvs"
    )

target_link_libraries(${PROJECT_NAME}_node
    ${YDLIDAR_SDK_LIBRARIES})

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_client
    src/${PROJECT_NAME}_client.cpp)

#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_client
    "rclcpp"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "std_srvs"
    )

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------
install(TARGETS
    ${PROJECT_NAME}_node ${PROJECT_NAME}_client
    DESTINATION lib/${PROJECT_NAME})

# Install only resource directories that exist in this vendor snapshot.
foreach(resource_dir launch params startup rviz config)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${resource_dir}")
    install(
      DIRECTORY "${resource_dir}"
      DESTINATION "share/${PROJECT_NAME}"
    )
  endif()
endforeach()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()

