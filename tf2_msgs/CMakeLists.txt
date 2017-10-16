cmake_minimum_required(VERSION 3.5)
project(tf2_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
#find_package(actionlib_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

## TODO(tfoote) add actions back
#add_action_files(DIRECTORY action FILES LookupTransform.action)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TF2Error.msg"
  "msg/TFMessage.msg"
  "srv/FrameGraph.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

install(
  FILES tfmessage_bridge_mapping_rule.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
