cmake_minimum_required(VERSION 3.5)
project(tf2_py)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(python_cmake_module REQUIRED)
find_package(PythonExtra REQUIRED)

set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")

if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

ament_python_install_package(${PROJECT_NAME})

function(set_properties _targetname _build_type)
  set_target_properties(${_targetname} PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/test_${PROJECT_NAME}"
    RUNTIME_OUTPUT_DIRECTORY${_build_type} "${CMAKE_CURRENT_BINARY_DIR}/test_${PROJECT_NAME}"
    OUTPUT_NAME "_${_targetname}${PythonExtra_EXTENSION_SUFFIX}"
    SUFFIX "${PythonExtra_EXTENSION_EXTENSION}")
endfunction()

add_library(${PROJECT_NAME} SHARED src/tf2_py.cpp)

set_properties(${PROJECT_NAME} "")
if(WIN32)
  set_properties(${PROJECT_NAME} "_DEBUG")
  set_properties(${PROJECT_NAME} "_MINSIZEREL")
  set_properties(${PROJECT_NAME} "_RELEASE")
  set_properties(${PROJECT_NAME} "_RELWITHDEBINFO")
endif()

ament_target_dependencies(${PROJECT_NAME}
  "geometry_msgs"
  "tf2"
  "PythonExtra"
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(tf2_py_test test
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
  )

  # Create importable location in build directory
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/test_tf2_py/__init__.py" "")
endif()

set(PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}")

ament_package()
