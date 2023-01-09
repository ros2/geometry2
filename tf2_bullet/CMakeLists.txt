cmake_minimum_required(VERSION 3.5)
project(tf2_bullet)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include(bullet-extras.cmake)

add_library(tf2_bullet INTERFACE)
target_include_directories(tf2_bullet INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(tf2_bullet INTERFACE
  tf2_bullet::Bullet
  tf2::tf2
  ${geometry_msgs_TARGETS}
  tf2_ros::tf2_ros)

install(TARGETS tf2_bullet EXPORT export_tf2_bullet)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cppcheck
  )
  ament_lint_auto_find_test_dependencies()
  ament_cppcheck(LANGUAGE c++)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bullet test/test_tf2_bullet.cpp)
  target_link_libraries(test_bullet tf2_bullet)
endif()

ament_export_targets(export_tf2_bullet)

ament_package(CONFIG_EXTRAS bullet-extras.cmake)
