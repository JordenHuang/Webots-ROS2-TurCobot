# I DON'T know if this file helps anything

cmake_minimum_required(VERSION 3.5)
project(my_package)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MyService.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
