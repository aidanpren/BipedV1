cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# MotorTelemetry embeds a std_msgs/Header, so std_msgs has to be found here AND
# listed in DEPENDENCIES below. Omitting the DEPENDENCIES line is the classic
# failure: cmake configures fine and the generator then fails on an unknown
# type it was never told where to look for.
find_package(std_msgs REQUIRED)

# every .srv / .msg you add goes in this list
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorTelemetry.msg"
  "srv/SetMode.srv"
  "srv/SaveParams.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
