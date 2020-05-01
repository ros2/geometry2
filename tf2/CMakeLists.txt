cmake_minimum_required(VERSION 3.5)
project(tf2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake)
find_package(console_bridge_vendor REQUIRED) # Provides console_bridge 0.4.0 on platforms without it.
find_package(console_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcutils REQUIRED)

# export user definitions

#CPP Libraries
add_library(tf2 SHARED src/cache.cpp src/buffer_core.cpp src/static_cache.cpp src/time.cpp)
target_include_directories(tf2 PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/bt>"
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
  # TODO(tfoote) enable linters
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest)

  ament_add_gtest(test_cache_unittest test/cache_unittest.cpp)
  if(TARGET test_cache_unittest)
    target_link_libraries(test_cache_unittest tf2)
    ament_target_dependencies(test_cache_unittest
      "geometry_msgs"
      "console_bridge"
    )
  endif()

  ament_add_gtest(test_static_cache_unittest test/static_cache_test.cpp)
  if(TARGET test_static_cache_unittest)
    target_link_libraries(test_static_cache_unittest tf2)
    ament_target_dependencies(test_static_cache_unittest
      "geometry_msgs"
      "console_bridge"
    )
    target_link_libraries(test_static_cache_unittest tf2 ${geometry_msgs_LIBRARIES} ${console_bridge_LIBRARIES})
  endif()

  ament_add_gtest(test_simple test/simple_tf2_core.cpp)
  if(TARGET test_simple)
    target_link_libraries(test_simple tf2)
    ament_target_dependencies(test_simple
      "geometry_msgs"
      "console_bridge"
    )
  endif()

  ament_add_gtest(test_time test/test_time.cpp)
  if (TARGET test_time)
    target_link_libraries(test_time tf2)
  endif()

# TODO(tfoote) reimplement speed test without dependency on message datatypes.
# add_executable(speed_test EXCLUDE_FROM_ALL test/speed_test.cpp)
# target_link_libraries(speed_test tf2  ${geometry_msgs_LIBRARIES} ${console_bridge_LIBRARIES})
# add_dependencies(tests speed_test)

endif()

ament_export_dependencies(console_bridge geometry_msgs rcutils)
ament_export_include_directories(include)
ament_export_libraries(tf2)
ament_export_targets(tf2)
ament_package()
