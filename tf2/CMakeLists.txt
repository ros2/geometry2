cmake_minimum_required(VERSION 3.5)
project(tf2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake)
find_package(console_bridge_vendor REQUIRED) # Provides console_bridge 0.4.0 on platforms without it.
find_package(console_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

# export user definitions

#CPP Libraries
add_library(tf2 SHARED
  src/buffer_core.cpp
  src/static_cache.cpp
  src/time.cpp
  src/time_cache.cpp
  src/transform_storage.cpp)
target_include_directories(tf2 PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(tf2
  "console_bridge"
  "geometry_msgs"
  "rcutils"
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(tf2 PRIVATE "TF2_BUILDING_DLL")

install(TARGETS tf2 EXPORT tf2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  # TODO(clalancette) enable linters once https://github.com/ament/ament_lint/pull/238 lands

  find_package(ament_cmake_gtest)

  ament_add_gtest(test_time_cache test/test_time_cache.cpp)
  if(TARGET test_time_cache)
    target_link_libraries(test_time_cache tf2)
    ament_target_dependencies(test_time_cache
      "console_bridge"
      "geometry_msgs"
    )
  endif()

  ament_add_gtest(test_static_cache test/test_static_cache.cpp)
  if(TARGET test_static_cache)
    target_link_libraries(test_static_cache tf2)
    ament_target_dependencies(test_static_cache
      "console_bridge"
      "geometry_msgs"
    )
  endif()

  ament_add_gtest(test_buffer_core test/test_buffer_core.cpp)
  if(TARGET test_buffer_core)
    target_link_libraries(test_buffer_core tf2)
    ament_target_dependencies(test_buffer_core
      "console_bridge"
      "geometry_msgs"
    )
  endif()

  ament_add_gtest(test_time test/test_time.cpp)
  if(TARGET test_time)
    target_link_libraries(test_time tf2)
  endif()

  ament_add_gtest(test_linear_math test/test_linear_math.cpp)
  if(TARGET test_linear_math)
    target_link_libraries(test_linear_math tf2)
  endif()
endif()

ament_export_dependencies(console_bridge geometry_msgs rcutils rosidl_runtime_cpp)
ament_export_include_directories(include)
ament_export_libraries(tf2)
ament_export_targets(tf2)
ament_package()
