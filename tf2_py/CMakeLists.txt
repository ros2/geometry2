cmake_minimum_required(VERSION 3.5)
project(tf2_py)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()
set(CMAKE_BUILD_TYPE Debug)
find_package(ament_cmake REQUIRED)
find_package(python_cmake_module REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PythonLibs 3 REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

include_directories(${PYTHON_INCLUDE_PATH})

add_library(${PROJECT_NAME} SHARED src/tf2_py.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME tf2 PREFIX "_" SUFFIX ".so")

ament_target_dependencies(${PROJECT_NAME} 
  "builtin_interfaces"
  "geometry_msgs"
  "python"
  "rclpy"
  "tf2"
  "tf2_msgs"
)

ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION ${PYTHON_INSTALL_DIR}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(tf2_py_test test)
endif()

ament_package()
