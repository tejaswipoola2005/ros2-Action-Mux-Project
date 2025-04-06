cmake_minimum_required(VERSION 3.8)
project(action_mux_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_mux_interfaces REQUIRED)

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

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_mux_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(client src/action_client.cpp)
ament_target_dependencies(client rclcpp rclcpp_action action_mux_interfaces std_msgs)

add_executable(pub_1 src/publisher_1.cpp)
ament_target_dependencies(pub_1 rclcpp std_msgs)

add_executable(pub_2 src/publisher_2.cpp)
ament_target_dependencies(pub_2 rclcpp std_msgs)

add_executable(pub_3 src/publisher_3.cpp)
ament_target_dependencies(pub_3 rclcpp std_msgs)

install(TARGETS
  client
  pub_1
  pub_2
  pub_3
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
