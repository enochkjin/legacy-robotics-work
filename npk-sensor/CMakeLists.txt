cmake_minimum_required(VERSION 3.8)
project(npk_sensor)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the custom service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetSoilData.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# Install the Python ROS node
install(
  PROGRAMS
    npk_sensor/npk_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
