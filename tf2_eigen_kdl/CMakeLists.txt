cmake_minimum_required(VERSION 3.5)
project(tf2_eigen_kdl)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# Work around broken find module in AlmaLinux/RHEL eigen3-devel from PowerTools repo
find_package(Eigen3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  find_package(Eigen3 REQUIRED)
endif()
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/tf2_eigen_kdl.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  orocos-kdl
  tf2::tf2)
if(TARGET Eigen3::Eigen)
  # TODO(sloretz) require target to exist when https://github.com/ros2/choco-packages/issues/19 is addressed
  target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
else()
  target_include_directories(${PROJECT_NAME} PUBLIC ${Eigen3_INCLUDE_DIRS})
endif()

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "TF2_EIGEN_KDL_BUILDING_DLL")

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(
  Eigen3
  orocos_kdl_vendor
  orocos_kdl
  tf2
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(tf2_eigen_kdl_test test/tf2_eigen_kdl_test.cpp)
  target_link_libraries(tf2_eigen_kdl_test ${PROJECT_NAME} tf2::tf2)
endif()

ament_package()
