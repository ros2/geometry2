cmake_minimum_required(VERSION 3.5)
project(tf2_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TF2Error.msg"
  "msg/TFMessage.msg"
  "srv/FrameGraph.srv"
  "action/LookupTransform.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
  ADD_LINTER_TESTS
)

install(
  FILES tfmessage_bridge_mapping_rule.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
