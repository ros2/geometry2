cmake_minimum_required(VERSION 3.5)
project(tf2_sensor_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME} INTERFACE
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)

if(TARGET Eigen3::Eigen)
  # TODO(sloretz) require target to exist when https://github.com/ros2/choco-packages/issues/19 is addressed
  target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)
else()
  target_include_directories(${PROJECT_NAME} INTERFACE ${Eigen3_INCLUDE_DIRS})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # TODO(ros2/geometry2#259) Remove once headers
  # are renamed to .hpp
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS --language CPP)
  set(ament_cmake_cppcheck_LANGUAGE c++)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_tf2_sensor_msgs_py test/test_tf2_sensor_msgs.py)

  ament_add_gtest(test_tf2_sensor_msgs_cpp test/test_tf2_sensor_msgs.cpp)
  if(TARGET test_tf2_sensor_msgs_cpp)
    target_include_directories(test_tf2_sensor_msgs_cpp PUBLIC include)
    target_link_libraries(test_tf2_sensor_msgs_cpp
      ${geometry_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      rclcpp::rclcpp
      tf2::tf2
      tf2_ros::tf2_ros
    )
    if(TARGET Eigen3::Eigen)
      # TODO(sloretz) require target to exist when https://github.com/ros2/choco-packages/issues/19 is addressed
      target_link_libraries(test_tf2_sensor_msgs_cpp Eigen3::Eigen)
    else()
      target_include_directories(test_tf2_sensor_msgs_cpp PUBLIC ${Eigen3_INCLUDE_DIRS})
    endif()
  endif()
endif()

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  "Eigen3"
  "geometry_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
)

ament_package()
