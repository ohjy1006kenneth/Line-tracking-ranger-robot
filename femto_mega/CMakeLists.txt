cmake_minimum_required(VERSION 3.8)
project(femto_mega)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(depth_camera src/depth_camera.cpp)
ament_target_dependencies(depth_camera rclcpp std_msgs cv_bridge sensor_msgs)

add_executable(color_camera src/color_camera.cpp)
ament_target_dependencies(color_camera rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
target_link_libraries(color_camera ${OpenCV_LIBS})


install(TARGETS
  depth_camera
  color_camera
  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

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
