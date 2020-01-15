cmake_minimum_required(VERSION 3.5)
project(tf2_ros_py)

find_package(ament_cmake REQUIRED)

ament_python_install_package(tf2_ros PACKAGE_DIR tf2_ros)

# Tests
if(BUILD_TESTING)
  # Python test
  # Provides PYTHON_EXECUTABLE_DEBUG
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra REQUIRED)

  set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
  if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
  endif()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(tf2_ros_py_test test)

  set(PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")
endif()

ament_package()
