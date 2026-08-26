cmake_minimum_required(VERSION 3.8)
project(sr_amr_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(deps
  geometry_msgs
  std_msgs
)

foreach(dep IN LISTS deps)
  find_package(${dep} REQUIRED)
endforeach()

set(interface_files
  "msg/Path.msg"
  "msg/BatteryState.msg"
  "msg/SystemState.msg"

  "action/MoveToStation.action"
  "action/MoveFollowPath.action"
  "action/LocateByPose.action"
  "action/LocateByStation.action"
  "action/SetMapWithInitialPose.action"
  "action/StartCharging.action"
  "action/StopCharging.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${interface_files}
  DEPENDENCIES
    ${deps}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
