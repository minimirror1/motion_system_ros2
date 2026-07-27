cmake_minimum_required(VERSION 3.8)
project(motion_control_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(motion_control_msgs REQUIRED)
find_package(motor_manager REQUIRED)
find_package(common_motor_interface REQUIRED)

add_library(status_publish_policy STATIC
  src/status_publish_policy.cpp
)
target_include_directories(status_publish_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(status_publish_policy PUBLIC cxx_std_17)

add_executable(motor_manager_node
  src/motor_manager_node.cpp
)
target_include_directories(motor_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(motor_manager_node
  rclcpp
  std_msgs
  motion_control_msgs
)
target_link_libraries(motor_manager_node
  status_publish_policy
  motor_manager::motor_manager
  common_motor_interface::common_motor_interface
)
target_compile_features(motor_manager_node PUBLIC cxx_std_17)

install(TARGETS motor_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_status_publish_policy
    test/test_status_publish_policy.cpp
  )
  target_link_libraries(test_status_publish_policy
    status_publish_policy
  )
endif()

ament_package()
