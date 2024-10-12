cmake_minimum_required(VERSION 3.20)
project(tf2_py)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(CMAKE_COMPILER_IS_GNUCXX)
  # GCC 8 on CentOS 7 (correctly) warns that casting
  # METH_VARARGS | METH_KEYWORDS functions (with 3 PyObject * arguments) to a
  # PyCFunction (with 2 PyObject * arguments) in PyMethodDef is an incompatible
  # cast.  This works fine since it is just a pointer, so disable that
  # warning to quiet it down.  Note that we do this globally rather than in a
  # pragma since gcc prior to 8 ignores unknown command-line flags while it
  # does *not* ignore unknown pragmas.
  add_compile_options(-Wno-cast-function-type)
endif()

# By default, without the settings below, find_package(Python3) will attempt
# to find the newest python version it can, and additionally will find the
# most specific version.  For instance, on a system that has
# /usr/bin/python3.10, /usr/bin/python3.11, and /usr/bin/python3, it will find
# /usr/bin/python3.11, even if /usr/bin/python3 points to /usr/bin/python3.10.
# The behavior we want is to prefer the "system" installed version unless the
# user specifically tells us othewise through the Python3_EXECUTABLE hint.
# Setting CMP0094 to NEW means that the search will stop after the first
# python version is found.  Setting Python3_FIND_UNVERSIONED_NAMES means that
# the search will prefer /usr/bin/python3 over /usr/bin/python3.11.  And that
# latter functionality is only available in CMake 3.20 or later, so we need
# at least that version.
cmake_policy(SET CMP0094 NEW)
set(Python3_FIND_UNVERSIONED_NAMES FIRST)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

python3_add_library(_tf2_py src/tf2_py.cpp)

if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  # python3_add_library should really take care of this for us, but it doesn't
  set_target_properties(_tf2_py PROPERTIES DEBUG_POSTFIX "_d")
endif()

# Set output directories to import module from the build directory
# Use a no-op generator expression so multi-config generators don't append an
# extra directory like Release/ or Debug/ and break the Python import.
set_target_properties(_tf2_py PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "$<1:${CMAKE_CURRENT_BINARY_DIR}/test_${PROJECT_NAME}>"
  RUNTIME_OUTPUT_DIRECTORY "$<1:${CMAKE_CURRENT_BINARY_DIR}/test_${PROJECT_NAME}>"
)

target_link_libraries(_tf2_py PRIVATE
  ${geometry_msgs_TARGETS}
  tf2::tf2
)

install(TARGETS
  _tf2_py
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # TODO (ahcorde): disable the copyright check. Once we figure out the ament_lint situation
  # https://github.com/ros2/geometry2/pull/222
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(tf2_py_test test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
  )

  # Create importable location in build directory
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/test_tf2_py/__init__.py" "")
endif()

ament_package()
