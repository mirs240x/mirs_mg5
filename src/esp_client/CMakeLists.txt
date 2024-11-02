cmake_minimum_required(VERSION 3.5)
project(esp_client)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(esp_srvs REQUIRED)

add_executable(update src/parameter_update.cpp)
ament_target_dependencies(update
  rclcpp
  esp_srvs
)

add_executable(reboot src/reboot.cpp)
ament_target_dependencies(reboot
  rclcpp
  esp_srvs
)

add_executable(reset src/encoder_reset.cpp)
ament_target_dependencies(reset
  rclcpp
  esp_srvs
)

install(TARGETS
  update
  reboot
  reset
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
