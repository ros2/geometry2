cmake_minimum_required(VERSION 3.5)
project(tf2_bullet)

# Default to C11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(WIN32)
  set(BULLET_ROOT $ENV{ChocolateyInstall}/lib/bullet)
endif()
find_package(Bullet REQUIRED)

include_directories(include ${BULLET_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

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
  ament_target_dependencies(test_bullet
    tf2
    geometry_msgs
    tf2_ros
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
