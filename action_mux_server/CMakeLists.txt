cmake_minimum_required(VERSION 3.8)
project(action_mux_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_mux_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

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

find_package(std_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_mux_interfaces REQUIRED)

add_executable(server src/action_server.cpp)
ament_target_dependencies(server rclcpp rclcpp_action action_mux_interfaces std_msgs)

add_executable(gen_subscriber src/generic_subscriber.cpp)
ament_target_dependencies(gen_subscriber rclcpp std_msgs)

install(TARGETS
  server
  gen_subscriber
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
