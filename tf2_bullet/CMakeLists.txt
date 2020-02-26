cmake_minimum_required(VERSION 3.5)
project(tf2_bullet)

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
