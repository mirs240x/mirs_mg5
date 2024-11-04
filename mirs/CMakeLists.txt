cmake_minimum_required(VERSION 3.5)
project(mirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mirs_msgs REQUIRED)

add_executable(test_cli src/test.cpp)
ament_target_dependencies(test_cli
  rclcpp
  mirs_msgs
)

add_executable(update src/parameter_update.cpp)
ament_target_dependencies(update
  rclcpp
  mirs_msgs
)

add_executable(reboot src/reboot.cpp)
ament_target_dependencies(reboot
  rclcpp
  mirs_msgs
)

add_executable(reset src/encoder_reset.cpp)
ament_target_dependencies(reset
  rclcpp
  mirs_msgs
)

install(TARGETS
  test_cli
  update
  reboot
  reset
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
