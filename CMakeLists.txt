cmake_minimum_required(VERSION 3.8)
project(micvision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
Eigen3
std_srvs
ament_cmake 
rclcpp
rclpy
tf2_geometry_msgs
geometry_msgs
tf2_ros
tf2
nav_msgs
sensor_msgs
std_msgs
rclcpp_action
)


add_executable(localization 
src/main.cpp
src/laser_calculation.cpp
src/initial_pose.cpp
src/map_calculation.cpp)


ament_target_dependencies(localization ${dependencies})


install(
  DIRECTORY
    src
  DESTINATION
    share/micvision
)

install(TARGETS
localization
  DESTINATION lib/${PROJECT_NAME})

# target_include_directories(${PROJECT_NAME}
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>)

include_directories(
  include
  )       #to put header

# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib/${PROJECT_NAME}
#   LIBRARY DESTINATION lib/${PROJECT_NAME}
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS ${PROJECT_NAME}_lib
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.h"
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
