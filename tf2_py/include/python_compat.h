#ifndef PYTHON_COMPAT_H_
#define PYTHON_COMPAT_H_

#include <Python.h>

#include <string>

/// \brief Converts a C++ string into a Python string.
/// \note The caller is responsible for decref'ing the returned object.
/// \note If the return value is NULL then an exception is set.
/// \return a new PyObject reference, or NULL
inline PyObject * stringToPython(const std::string & input)
{
#if PY_MAJOR_VERSION >= 3
  return PyUnicode_FromStringAndSize(input.c_str(), input.size());
#else
  return PyString_FromStringAndSize(input.c_str(), input.size());
#endif
}

/// \brief Converts a C string into a Python string.
/// \note The caller is responsible for decref'ing the returned object.
/// \note If the return value is NULL then an exception is set.
/// \return a new PyObject reference, or NULL
inline PyObject * stringToPython(const char * input)
{
#if PY_MAJOR_VERSION >= 3
  return PyUnicode_FromString(input);
#else
  return PyString_FromString(input);
#endif
}

/// \brief Converts a Python string into a C++ string.
/// \note The input PyObject is borrowed, and will not be decref'd.
/// \note It's possible for this function to set an exception.
///   If the returned string is empty, callers should check if an exception was
///   set using PyErr_Ocurred().
/// \return a new std::string instance
inline std::string stringFromPython(PyObject * input)
{
  Py_ssize_t size;
  const char * data;
#if PY_MAJOR_VERSION >= 3
  data = PyUnicode_AsUTF8AndSize(input, &size);
#else
  PyString_AsStringAndSize(input, &data, &size);
#endif
  return std::string(data, size);
}

/// \brief Imports a python module by name.
/// \note The caller is responsible for decref'ing the returned object.
/// \note If the return value is NULL then an exception is set.
/// \return a reference to the imported module.
inline PyObject * pythonImport(const std::string & name)
{
  PyObject * py_name = stringToPython(name);
  if (!py_name) {
    return nullptr;
  }
  PyObject * module = PyImport_Import(py_name);
  Py_XDECREF(py_name);
  return module;
}

#endif  // PYTHON_COMPAT_H_
