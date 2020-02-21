#include <python_compat.h>
#include <Python.h>

#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>

#include <string>
#include <vector>

// Run x (a tf method, catching TF's exceptions and reraising them as Python exceptions)
//
#define WRAP(x) \
  do { \
    try \
    { \
      x; \
    } \
    catch (const tf2::ConnectivityException & e) \
    { \
      PyErr_SetString(tf2_connectivityexception, e.what()); \
      return nullptr; \
    } \
    catch (const tf2::LookupException & e) \
    { \
      PyErr_SetString(tf2_lookupexception, e.what()); \
      return nullptr; \
    } \
    catch (const tf2::ExtrapolationException & e) \
    { \
      PyErr_SetString(tf2_extrapolationexception, e.what()); \
      return nullptr; \
    } \
    catch (const tf2::InvalidArgumentException & e) \
    { \
      PyErr_SetString(tf2_invalidargumentexception, e.what()); \
      return nullptr; \
    } \
    catch (const tf2::TimeoutException & e) \
    { \
      PyErr_SetString(tf2_timeoutexception, e.what()); \
      return nullptr; \
    } \
    catch (const tf2::TransformException & e) \
    { \
      PyErr_SetString(tf2_exception, e.what()); \
      return nullptr; \
    } \
  } while (0)

static PyObject * pModulerclpy = nullptr;
static PyObject * pModulerclpytime = nullptr;
static PyObject * pModulebuiltininterfacesmsgs = nullptr;
static PyObject * pModulegeometrymsgs = nullptr;
static PyObject * tf2_exception = nullptr;
static PyObject * tf2_connectivityexception = nullptr, * tf2_lookupexception = nullptr,
  * tf2_extrapolationexception = nullptr,
  * tf2_invalidargumentexception = nullptr, * tf2_timeoutexception = nullptr;

struct buffer_core_t
{
  PyObject_HEAD
  tf2::BufferCore * bc;
};

static PyObject * transform_converter(const geometry_msgs::msg::TransformStamped * transform)
{
  PyObject * pclass = nullptr;
  PyObject * pargs = nullptr;
  PyObject * pinst = nullptr;
  PyObject * builtin_interfaces_time = nullptr;
  PyObject * args = nullptr;
  PyObject * kwargs = nullptr;
  PyObject * sec = nullptr;
  PyObject * nanosec = nullptr;
  PyObject * time_obj = nullptr;
  PyObject * pheader = nullptr;
  PyObject * pframe_id = nullptr;
  PyObject * ptransform = nullptr;
  PyObject * ptranslation = nullptr;
  PyObject * protation = nullptr;
  PyObject * child_frame_id = nullptr;
  PyObject * ptrans_x = nullptr;
  PyObject * ptrans_y = nullptr;
  PyObject * ptrans_z = nullptr;
  PyObject * prot_x = nullptr;
  PyObject * prot_y = nullptr;
  PyObject * prot_z = nullptr;
  PyObject * prot_w = nullptr;
  pclass = PyObject_GetAttrString(pModulegeometrymsgs, "TransformStamped");
  if (!pclass) {
    goto cleanup;
  }

  pargs = Py_BuildValue("()");
  if (!pargs) {
    goto cleanup;
  }

  pinst = PyEval_CallObject(pclass, pargs);
  if (!pinst) {
    goto cleanup;
  }

  // we need to convert the time to python
  builtin_interfaces_time = PyObject_GetAttrString(pModulebuiltininterfacesmsgs, "Time");
  if (!builtin_interfaces_time) {
    goto cleanup;
  }
  args = PyTuple_New(0);
  if (!args) {
    goto cleanup;
  }
  kwargs = PyDict_New();
  if (!kwargs) {
    goto cleanup;
  }
  sec = Py_BuildValue("i", transform->header.stamp.sec);
  if (!sec) {
    goto cleanup;
  }
  nanosec = Py_BuildValue("i", transform->header.stamp.nanosec);
  if (!nanosec) {
    goto cleanup;
  }
  if (-1 == PyDict_SetItemString(kwargs, "sec", sec)) {
    goto cleanup;
  }
  if (-1 == PyDict_SetItemString(kwargs, "nanosec", nanosec)) {
    goto cleanup;
  }

  time_obj = PyObject_Call(builtin_interfaces_time, args, kwargs);
  if (!time_obj) {
    goto cleanup;
  }

  pheader = PyObject_GetAttrString(pinst, "header");
  if (!pheader) {
    goto cleanup;
  }
  if (-1 == PyObject_SetAttrString(pheader, "stamp", time_obj)) {
    goto cleanup;
  }

  pframe_id = stringToPython(transform->header.frame_id);
  if (!pframe_id) {
    goto cleanup;
  }
  if (-1 == PyObject_SetAttrString(pheader, "frame_id", pframe_id)) {
    goto cleanup;
  }

  ptransform = PyObject_GetAttrString(pinst, "transform");
  if (!ptransform) {
    goto cleanup;
  }
  ptranslation = PyObject_GetAttrString(ptransform, "translation");
  if (!ptranslation) {
    goto cleanup;
  }
  protation = PyObject_GetAttrString(ptransform, "rotation");
  if (!ptranslation) {
    goto cleanup;
  }

  child_frame_id = stringToPython(transform->child_frame_id);
  if (!child_frame_id) {
    goto cleanup;
  }
  if (-1 == PyObject_SetAttrString(pinst, "child_frame_id", child_frame_id)) {
    goto cleanup;
  }

  ptrans_x = PyFloat_FromDouble(transform->transform.translation.x);
  if (!ptrans_x) {
    goto cleanup;
  }
  ptrans_y = PyFloat_FromDouble(transform->transform.translation.y);
  if (!ptrans_y) {
    goto cleanup;
  }
  ptrans_z = PyFloat_FromDouble(transform->transform.translation.z);
  if (!ptrans_z) {
    goto cleanup;
  }
  if (-1 == PyObject_SetAttrString(ptranslation, "x", ptrans_x)) {
    goto cleanup;
  }
  if (-1 == PyObject_SetAttrString(ptranslation, "y", ptrans_y)) {
    goto cleanup;
  }
  if (-1 == PyObject_SetAttrString(ptranslation, "z", ptrans_z)) {
    goto cleanup;
  }

  prot_x = PyFloat_FromDouble(transform->transform.rotation.x);
  if (!prot_x) {
    goto cleanup;
  }

  prot_y = PyFloat_FromDouble(transform->transform.rotation.y);
  if (!prot_y) {
    goto cleanup;
  }

  prot_z = PyFloat_FromDouble(transform->transform.rotation.z);
  if (!prot_z) {
    goto cleanup;
  }

  prot_w = PyFloat_FromDouble(transform->transform.rotation.w);
  if (!prot_w) {
    goto cleanup;
  }

  if (-1 == PyObject_SetAttrString(protation, "x", prot_x)) {
    goto cleanup;
  }

  if (-1 == PyObject_SetAttrString(protation, "y", prot_y)) {
    goto cleanup;
  }

  if (-1 == PyObject_SetAttrString(protation, "z", prot_z)) {
    goto cleanup;
  }

  if (-1 == PyObject_SetAttrString(protation, "w", prot_w)) {
    goto cleanup;
  }
cleanup:
  if (PyErr_Occurred()) {
    Py_XDECREF(pinst);
    pinst = nullptr;
  }
  Py_XDECREF(pclass);
  Py_XDECREF(pargs);
  Py_XDECREF(builtin_interfaces_time);
  Py_XDECREF(args);
  Py_XDECREF(kwargs);
  Py_XDECREF(sec);
  Py_XDECREF(nanosec);
  Py_XDECREF(time_obj);
  Py_XDECREF(pheader);
  Py_XDECREF(pframe_id);
  Py_XDECREF(ptransform);
  Py_XDECREF(ptranslation);
  Py_XDECREF(protation);
  Py_XDECREF(child_frame_id);
  Py_XDECREF(ptrans_x);
  Py_XDECREF(ptrans_y);
  Py_XDECREF(ptrans_z);
  Py_XDECREF(prot_x);
  Py_XDECREF(prot_y);
  Py_XDECREF(prot_z);
  Py_XDECREF(prot_w);
  return pinst;
}

static builtin_interfaces::msg::Time toMsg(const tf2::TimePoint & t)
{
  std::chrono::nanoseconds ns = t.time_since_epoch();
  std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(ns);
  ns -= s;
  builtin_interfaces::msg::Time time_msg;
  time_msg.sec = (int32_t)s.count();
  time_msg.nanosec = (uint32_t)ns.count();
  return time_msg;
}

static tf2::TimePoint fromMsg(const builtin_interfaces::msg::Time & time_msg)
{
  std::chrono::nanoseconds ns = std::chrono::seconds(time_msg.sec) + std::chrono::nanoseconds(
    time_msg.nanosec);
  return tf2::TimePoint(ns);
}

static int rostime_converter(PyObject * obj, tf2::TimePoint * rt)
{
  if (PyObject_HasAttrString(obj, "sec") && PyObject_HasAttrString(obj, "nanosec")) {
    PyObject * sec = PyObject_GetAttrString(obj, "sec");
    PyObject * nanosec = PyObject_GetAttrString(obj, "nanosec");
    builtin_interfaces::msg::Time msg;
    msg.sec = PyLong_AsLong(sec);
    msg.nanosec = PyLong_AsUnsignedLong(nanosec);
    *rt = fromMsg(msg);
    Py_XDECREF(nanosec);
    Py_XDECREF(sec);
    return PyErr_Occurred() ? 0 : 1;
  }

  if (PyObject_HasAttrString(obj, "nanoseconds")) {
    PyObject * nanoseconds = PyObject_GetAttrString(obj, "nanoseconds");
    const int64_t d = PyLong_AsLongLong(nanoseconds);
    *rt = tf2::TimePoint(std::chrono::nanoseconds(d));
    Py_XDECREF(nanoseconds);
    return PyErr_Occurred() ? 0 : 1;
  }

  PyErr_SetString(PyExc_TypeError, "time must have sec and nanosec, or nanoseconds.");
  return 0;
}

static int rosduration_converter(PyObject * obj, tf2::Duration * rt)
{
  if (PyObject_HasAttrString(obj, "sec") && PyObject_HasAttrString(obj, "nanosec")) {
    PyObject * sec = PyObject_GetAttrString(obj, "sec");
    PyObject * nanosec = PyObject_GetAttrString(obj, "nanosec");
    *rt = std::chrono::seconds(PyLong_AsLong(sec)) +
      std::chrono::nanoseconds(PyLong_AsUnsignedLong(nanosec));
    Py_XDECREF(nanosec);
    Py_XDECREF(sec);
    return PyErr_Occurred() ? 0 : 1;
  }

  if (PyObject_HasAttrString(obj, "nanoseconds")) {
    PyObject * nanoseconds = PyObject_GetAttrString(obj, "nanoseconds");
    const int64_t d = PyLong_AsLongLong(nanoseconds);
    *rt = std::chrono::duration_cast<tf2::Duration>(std::chrono::nanoseconds(d));
    Py_XDECREF(nanoseconds);
    return PyErr_Occurred() ? 0 : 1;
  }

  PyErr_SetString(PyExc_TypeError, "duration must have sec and nanosec, or nanoseconds.");
  return 0;
}

static int BufferCore_init(PyObject * self, PyObject * args, PyObject * kw)
{
  (void)kw;

  tf2::Duration cache_time(tf2::BUFFER_CORE_DEFAULT_CACHE_TIME);

  if (!PyArg_ParseTuple(args, "|O&", rosduration_converter, &cache_time)) {
    return -1;
  }

  reinterpret_cast<buffer_core_t *>(self)->bc = new tf2::BufferCore(cache_time);

  return 0;
}

static PyObject * allFramesAsYAML(PyObject * self, PyObject * args)
{
  (void)args;
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  return stringToPython(bc->allFramesAsYAML());
}

static PyObject * allFramesAsString(PyObject * self, PyObject * args)
{
  (void)args;
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  return stringToPython(bc->allFramesAsString());
}

static PyObject * canTransformCore(PyObject * self, PyObject * args, PyObject * kw)
{
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  char * target_frame, * source_frame;
  tf2::TimePoint time;
  static const char * keywords[] = {"target_frame", "source_frame", "time", nullptr};

  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "ssO&",
      const_cast<char **>(reinterpret_cast<const char **>(keywords)), &target_frame,
      &source_frame, rostime_converter, &time))
  {
    return nullptr;
  }
  std::string error_msg;
  bool can_transform = bc->canTransform(target_frame, source_frame, time, &error_msg);
  // return PyBool_FromLong(t->canTransform(target_frame, source_frame, time));
  return Py_BuildValue("bs", can_transform, error_msg.c_str());
}

static PyObject * canTransformFullCore(PyObject * self, PyObject * args, PyObject * kw)
{
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  char * target_frame, * source_frame, * fixed_frame;
  tf2::TimePoint target_time, source_time;
  static const char * keywords[] =
  {"target_frame", "target_time", "source_frame", "source_time", "fixed_frame", nullptr};

  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "sO&sO&s",
      const_cast<char **>(reinterpret_cast<const char **>(keywords)),
      &target_frame,
      rostime_converter,
      &target_time,
      &source_frame,
      rostime_converter,
      &source_time,
      &fixed_frame))
  {
    return nullptr;
  }
  std::string error_msg;
  bool can_transform = bc->canTransform(
    target_frame, target_time, source_frame, source_time,
    fixed_frame, &error_msg);
  // return PyBool_FromLong(t->canTransform(target_frame, target_time, source_frame,
  //                                        source_time, fixed_frame));
  return Py_BuildValue("bs", can_transform, error_msg.c_str());
}

static PyObject * asListOfStrings(std::vector<std::string> los)
{
  PyObject * r = PyList_New(los.size());
  size_t i;
  for (i = 0; i < los.size(); i++) {
    PyList_SetItem(r, i, stringToPython(los[i]));
  }
  return r;
}

static PyObject * _chain(PyObject * self, PyObject * args, PyObject * kw)
{
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  char * target_frame, * source_frame, * fixed_frame;
  tf2::TimePoint target_time, source_time;
  std::vector<std::string> output;
  static const char * keywords[] =
  {"target_frame", "target_time", "source_frame", "source_time", "fixed_frame", nullptr};

  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "sO&sO&s",
      const_cast<char **>(reinterpret_cast<const char **>(keywords)),
      &target_frame,
      rostime_converter,
      &target_time,
      &source_frame,
      rostime_converter,
      &source_time,
      &fixed_frame))
  {
    return nullptr;
  }

  WRAP(
    bc->_chainAsVector(
      target_frame, target_time, source_frame, source_time, fixed_frame,
      output));
  return asListOfStrings(output);
}

static PyObject * getLatestCommonTime(PyObject * self, PyObject * args)
{
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  char * target_frame, * source_frame;
  tf2::CompactFrameID target_id, source_id;
  tf2::TimePoint tf2_time;
  std::string error_string;

  if (!PyArg_ParseTuple(args, "ss", &target_frame, &source_frame)) {
    return nullptr;
  }
  WRAP(target_id = bc->_validateFrameId("get_latest_common_time", target_frame));
  WRAP(source_id = bc->_validateFrameId("get_latest_common_time", source_frame));
  const tf2::TF2Error r = bc->_getLatestCommonTime(target_id, source_id, tf2_time, &error_string);

  if (r != tf2::TF2Error::NO_ERROR) {
    PyErr_SetString(tf2_exception, error_string.c_str());
    return nullptr;
  }

  builtin_interfaces::msg::Time time_msg;
  PyObject * rclpy_time = nullptr;
  PyObject * call_args = nullptr;
  PyObject * kwargs = nullptr;
  PyObject * seconds = nullptr;
  PyObject * nanoseconds = nullptr;
  PyObject * ob = nullptr;

  rclpy_time = PyObject_GetAttrString(pModulerclpytime, "Time");
  if (!rclpy_time) {
    goto cleanup;
  }
  time_msg = toMsg(tf2_time);
  call_args = PyTuple_New(0);
  if (!call_args) {
    goto cleanup;
  }
  kwargs = PyDict_New();
  if (!kwargs) {
    goto cleanup;
  }
  seconds = Py_BuildValue("i", time_msg.sec);
  if (!seconds) {
    goto cleanup;
  }
  nanoseconds = Py_BuildValue("i", time_msg.nanosec);
  if (!nanoseconds) {
    goto cleanup;
  }
  if (0 != PyDict_SetItemString(kwargs, "seconds", seconds)) {
    goto cleanup;
  }
  if (0 != PyDict_SetItemString(kwargs, "nanoseconds", nanoseconds)) {
    goto cleanup;
  }

  ob = PyObject_Call(rclpy_time, call_args, kwargs);

cleanup:
  if (PyErr_Occurred()) {
    Py_XDECREF(ob);
    ob = nullptr;
  }
  Py_XDECREF(rclpy_time);
  Py_XDECREF(call_args);
  Py_XDECREF(kwargs);
  Py_XDECREF(seconds);
  Py_XDECREF(nanoseconds);
  return ob;
}

static PyObject * lookupTransformCore(PyObject * self, PyObject * args, PyObject * kw)
{
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  char * target_frame, * source_frame;
  tf2::TimePoint time;
  static const char * keywords[] = {"target_frame", "source_frame", "time", nullptr};

  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "ssO&",
      const_cast<char **>(reinterpret_cast<const char **>(keywords)), &target_frame,
      &source_frame, rostime_converter, &time))
  {
    return nullptr;
  }
  geometry_msgs::msg::TransformStamped transform;
  WRAP(transform = bc->lookupTransform(target_frame, source_frame, time));
  // TODO(anyone): Create a converter that will actually return a python message
  return Py_BuildValue("O&", transform_converter, &transform);
  // return Py_BuildValue("(ddd)(dddd)",
  //    origin.x, origin.y, origin.z,
  //    rotation.x, rotation.y, rotation.z, rotation.w);
}

static PyObject * lookupTransformFullCore(PyObject * self, PyObject * args, PyObject * kw)
{
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  char * target_frame, * source_frame, * fixed_frame;
  tf2::TimePoint target_time, source_time;
  static const char * keywords[] =
  {"target_frame", "target_time", "source_frame", "source_time", "fixed_frame", nullptr};

  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "sO&sO&s",
      const_cast<char **>(reinterpret_cast<const char **>(keywords)),
      &target_frame,
      rostime_converter,
      &target_time,
      &source_frame,
      rostime_converter,
      &source_time,
      &fixed_frame))
  {
    return nullptr;
  }
  geometry_msgs::msg::TransformStamped transform;
  WRAP(
    transform =
    bc->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame));
  // TODO(anyone): Create a converter that will actually return a python message
  return Py_BuildValue("O&", transform_converter, &transform);
}
/*
static PyObject *lookupTwistCore(PyObject *self, PyObject *args, PyObject *kw)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *tracking_frame, *observation_frame;
  builtin_interfaces::msg::Time time;
  tf2::Duration averaging_interval;
  static const char *keywords[] = { "tracking_frame", "observation_frame", "time", "averaging_interval", nullptr };

  if (!PyArg_ParseTupleAndKeywords(args, kw, "ssO&O&", (char**)keywords, &tracking_frame, &observation_frame, rostime_converter, &time, rosduration_converter, &averaging_interval))
    return nullptr;
  geometry_msgs::msg::Twist twist;
  WRAP(twist = bc->lookupTwist(tracking_frame, observation_frame, time, averaging_interval));

  return Py_BuildValue("(ddd)(ddd)",
      twist.linear.x, twist.linear.y, twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z);
}

static PyObject *lookupTwistFullCore(PyObject *self, PyObject *args)
{
  tf2::BufferCore *bc = ((buffer_core_t*)self)->bc;
  char *tracking_frame, *observation_frame, *reference_frame, *reference_point_frame;
  builtin_interfaces::msg::Time time;
  tf2::Duration averaging_interval;
  double px, py, pz;

  if (!PyArg_ParseTuple(args, "sss(ddd)sO&O&",
                        &tracking_frame,
                        &observation_frame,
                        &reference_frame,
                        &px, &py, &pz,
                        &reference_point_frame,
                        rostime_converter, &time,
                        rosduration_converter, &averaging_interval))
    return nullptr;
  geometry_msgs::msg::Twist twist;
  tf::Point pt(px, py, pz);
  WRAP(twist = bc->lookupTwist(tracking_frame, observation_frame, reference_frame, pt, reference_point_frame, time, averaging_interval));

  return Py_BuildValue("(ddd)(ddd)",
      twist.linear.x, twist.linear.y, twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z);
}
*/
static inline int checkTranslationType(PyObject * o)
{
  PyTypeObject * translation_type =
    reinterpret_cast<PyTypeObject *>(PyObject_GetAttrString(
      pModulegeometrymsgs,
      "Vector3"));
  if (!translation_type) {
    return 0;
  }
  int type_check = PyObject_TypeCheck(o, translation_type);
  Py_DECREF(reinterpret_cast<PyObject *>(translation_type));
  int attr_check = PyObject_HasAttrString(o, "x") &&
    PyObject_HasAttrString(o, "y") &&
    PyObject_HasAttrString(o, "z");
  if (!type_check) {
    PyErr_WarnEx(PyExc_UserWarning, "translation should be of type Vector3", 1);
    return type_check;
  }
  return attr_check;
}

static inline int checkRotationType(PyObject * o)
{
  PyTypeObject * rotation_type =
    reinterpret_cast<PyTypeObject *>(PyObject_GetAttrString(
      pModulegeometrymsgs,
      "Quaternion"));
  if (!rotation_type) {
    return 0;
  }
  int type_check = PyObject_TypeCheck(o, rotation_type);
  Py_DECREF(reinterpret_cast<PyObject *>(rotation_type));
  int attr_check = PyObject_HasAttrString(o, "w") &&
    PyObject_HasAttrString(o, "x") &&
    PyObject_HasAttrString(o, "y") &&
    PyObject_HasAttrString(o, "z");
  if (!type_check) {
    PyErr_WarnEx(PyExc_UserWarning, "translation should be of type Quaternion", 1);
  }
  return attr_check;
}

static PyObject * setTransform(PyObject * self, PyObject * args)
{
  PyObject * ret = nullptr;
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  PyObject * py_transform;
  char * authority;
  tf2::TimePoint time;

  if (!PyArg_ParseTuple(args, "Os", &py_transform, &authority)) {
    return nullptr;
  }

  geometry_msgs::msg::TransformStamped transform;

  PyObject * header = nullptr;
  PyObject * stamp = nullptr;
  PyObject * frame_id = nullptr;
  PyObject * child_frame_id = nullptr;

  PyObject * mtransform = nullptr;

  PyObject * translation = nullptr;
  PyObject * tx = nullptr;
  PyObject * ty = nullptr;
  PyObject * tz = nullptr;

  PyObject * rotation = nullptr;
  PyObject * rx = nullptr;
  PyObject * ry = nullptr;
  PyObject * rz = nullptr;
  PyObject * rw = nullptr;

  header = PyObject_GetAttrString(py_transform, "header");
  if (!header) {
    goto cleanup;
  }
  stamp = PyObject_GetAttrString(header, "stamp");
  if (!stamp) {
    goto cleanup;
  }
  frame_id = PyObject_GetAttrString(header, "frame_id");
  if (!frame_id) {
    goto cleanup;
  }
  child_frame_id = PyObject_GetAttrString(py_transform, "child_frame_id");
  if (!child_frame_id) {
    goto cleanup;
  }

  mtransform = PyObject_GetAttrString(py_transform, "transform");
  if (!mtransform) {
    goto cleanup;
  }

  translation = PyObject_GetAttrString(mtransform, "translation");
  if (!translation) {
    goto cleanup;
  }
  tx = PyObject_GetAttrString(translation, "x");
  if (!tx) {
    goto cleanup;
  }
  ty = PyObject_GetAttrString(translation, "y");
  if (!ty) {
    goto cleanup;
  }
  tz = PyObject_GetAttrString(translation, "z");
  if (!tz) {
    goto cleanup;
  }

  rotation = PyObject_GetAttrString(mtransform, "rotation");
  if (!rotation) {
    goto cleanup;
  }
  rx = PyObject_GetAttrString(rotation, "x");
  if (!rx) {
    goto cleanup;
  }
  ry = PyObject_GetAttrString(rotation, "y");
  if (!ry) {
    goto cleanup;
  }
  rz = PyObject_GetAttrString(rotation, "z");
  if (!rz) {
    goto cleanup;
  }
  rw = PyObject_GetAttrString(rotation, "w");
  if (!rw) {
    goto cleanup;
  }

  transform.header.frame_id = stringFromPython(frame_id);

  if (rostime_converter(stamp, &time) != 1) {
    goto cleanup;
  }

  transform.child_frame_id = stringFromPython(child_frame_id);

  transform.header.stamp = toMsg(time);

  if (!checkTranslationType(translation)) {
    PyErr_SetString(PyExc_TypeError, "transform.translation must have members x, y, z");
    goto cleanup;
  }

  transform.transform.translation.x = PyFloat_AsDouble(tx);
  transform.transform.translation.y = PyFloat_AsDouble(ty);
  transform.transform.translation.z = PyFloat_AsDouble(tz);

  if (!checkRotationType(rotation)) {
    PyErr_SetString(PyExc_TypeError, "transform.rotation must have members w, x, y, z");
    goto cleanup;
  }

  transform.transform.rotation.x = PyFloat_AsDouble(rx);
  transform.transform.rotation.y = PyFloat_AsDouble(ry);
  transform.transform.rotation.z = PyFloat_AsDouble(rz);
  transform.transform.rotation.w = PyFloat_AsDouble(rw);

  bc->setTransform(transform, authority);

  Py_INCREF(Py_None);
  ret = Py_None;

cleanup:
  Py_XDECREF(rw);
  Py_XDECREF(rz);
  Py_XDECREF(ry);
  Py_XDECREF(rx);
  Py_XDECREF(rotation);
  Py_XDECREF(tz);
  Py_XDECREF(ty);
  Py_XDECREF(tx);
  Py_XDECREF(translation);
  Py_XDECREF(mtransform);
  Py_XDECREF(child_frame_id);
  Py_XDECREF(frame_id);
  Py_XDECREF(stamp);
  Py_XDECREF(header);

  return ret;
}

static PyObject * setTransformStatic(PyObject * self, PyObject * args)
{
  PyObject * ret = nullptr;
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  PyObject * py_transform;
  char * authority;
  tf2::TimePoint time;

  if (!PyArg_ParseTuple(args, "Os", &py_transform, &authority)) {
    return nullptr;
  }

  geometry_msgs::msg::TransformStamped transform;

  PyObject * header = nullptr;
  PyObject * stamp = nullptr;
  PyObject * child_frame_id = nullptr;
  PyObject * frame_id = nullptr;
  PyObject * mtransform = nullptr;
  PyObject * translation = nullptr;
  PyObject * tx = nullptr;
  PyObject * ty = nullptr;
  PyObject * tz = nullptr;
  PyObject * rotation = nullptr;
  PyObject * rx = nullptr;
  PyObject * ry = nullptr;
  PyObject * rz = nullptr;
  PyObject * rw = nullptr;

  header = PyObject_GetAttrString(py_transform, "header");
  if (!header) {
    goto cleanup;
  }
  stamp = PyObject_GetAttrString(header, "stamp");
  if (!stamp) {
    goto cleanup;
  }
  child_frame_id = PyObject_GetAttrString(py_transform, "child_frame_id");
  if (!child_frame_id) {
    goto cleanup;
  }
  frame_id = PyObject_GetAttrString(header, "frame_id");
  if (!frame_id) {
    goto cleanup;
  }
  mtransform = PyObject_GetAttrString(py_transform, "transform");
  if (!mtransform) {
    goto cleanup;
  }
  translation = PyObject_GetAttrString(mtransform, "translation");
  if (!translation) {
    goto cleanup;
  }
  tx = PyObject_GetAttrString(translation, "x");
  if (!tx) {
    goto cleanup;
  }
  ty = PyObject_GetAttrString(translation, "y");
  if (!ty) {
    goto cleanup;
  }
  tz = PyObject_GetAttrString(translation, "z");
  if (!tz) {
    goto cleanup;
  }
  rotation = PyObject_GetAttrString(mtransform, "rotation");
  if (!rotation) {
    goto cleanup;
  }
  rx = PyObject_GetAttrString(rotation, "x");
  if (!rx) {
    goto cleanup;
  }
  ry = PyObject_GetAttrString(rotation, "y");
  if (!ry) {
    goto cleanup;
  }
  rz = PyObject_GetAttrString(rotation, "z");
  if (!rz) {
    goto cleanup;
  }
  rw = PyObject_GetAttrString(rotation, "w");
  if (!rw) {
    goto cleanup;
  }

  transform.child_frame_id = stringFromPython(child_frame_id);
  transform.header.frame_id = stringFromPython(frame_id);

  if (rostime_converter(stamp, &time) != 1) {
    goto cleanup;
  }
  transform.header.stamp = toMsg(time);

  if (!checkTranslationType(translation)) {
    PyErr_SetString(PyExc_TypeError, "transform.translation must be of type Vector3");
    goto cleanup;
  }

  transform.transform.translation.x = PyFloat_AsDouble(tx);
  transform.transform.translation.y = PyFloat_AsDouble(ty);
  transform.transform.translation.z = PyFloat_AsDouble(tz);

  if (!checkRotationType(rotation)) {
    PyErr_SetString(PyExc_TypeError, "transform.rotation must be of type Quaternion");
    goto cleanup;
  }

  transform.transform.rotation.x = PyFloat_AsDouble(rx);
  transform.transform.rotation.y = PyFloat_AsDouble(ry);
  transform.transform.rotation.z = PyFloat_AsDouble(rz);
  transform.transform.rotation.w = PyFloat_AsDouble(rw);

  // only difference to above is is_static == True
  bc->setTransform(transform, authority, true);

  Py_INCREF(Py_None);
  ret = Py_None;

cleanup:
  Py_XDECREF(rw);
  Py_XDECREF(rz);
  Py_XDECREF(ry);
  Py_XDECREF(rx);
  Py_XDECREF(rotation);
  Py_XDECREF(tz);
  Py_XDECREF(ty);
  Py_XDECREF(tx);
  Py_XDECREF(translation);
  Py_XDECREF(mtransform);
  Py_XDECREF(frame_id);
  Py_XDECREF(child_frame_id);
  Py_XDECREF(stamp);
  Py_XDECREF(header);

  return ret;
}

static PyObject * clear(PyObject * self, PyObject * args)
{
  (void)args;
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  bc->clear();
  Py_RETURN_NONE;
}

static PyObject * _frameExists(PyObject * self, PyObject * args)
{
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  char * frame_id_str;
  if (!PyArg_ParseTuple(args, "s", &frame_id_str)) {
    return nullptr;
  }
  return PyBool_FromLong(bc->_frameExists(frame_id_str));
}

static PyObject * _getFrameStrings(PyObject * self, PyObject * args)
{
  (void)args;
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  std::vector<std::string> ids;
  bc->_getFrameStrings(ids);
  return asListOfStrings(ids);
}

static PyObject * _allFramesAsDot(PyObject * self, PyObject * args, PyObject * kw)
{
  tf2::BufferCore * bc = reinterpret_cast<buffer_core_t *>(self)->bc;
  static const char * keywords[] = {"time", nullptr};
  tf2::TimePoint time;
  if (!PyArg_ParseTupleAndKeywords(
      args, kw, "|O&",
      const_cast<char **>(reinterpret_cast<const char **>(keywords)), rostime_converter, &time))
  {
    return nullptr;
  }
  return stringToPython(bc->_allFramesAsDot(time).c_str());
}


static struct PyMethodDef buffer_core_methods[] =
{
  {"all_frames_as_yaml", allFramesAsYAML, METH_VARARGS, nullptr},
  {"all_frames_as_string", allFramesAsString, METH_VARARGS, nullptr},
  {"set_transform", setTransform, METH_VARARGS, nullptr},
  {"set_transform_static", setTransformStatic, METH_VARARGS, nullptr},
  {"can_transform_core", (PyCFunction)canTransformCore, METH_VARARGS | METH_KEYWORDS, nullptr},
  {"can_transform_full_core", (PyCFunction)canTransformFullCore, METH_VARARGS | METH_KEYWORDS,
    nullptr},
  {"_chain", (PyCFunction)_chain, METH_VARARGS | METH_KEYWORDS, nullptr},
  {"clear", (PyCFunction)clear, METH_VARARGS | METH_KEYWORDS, nullptr},
  {"_frameExists", (PyCFunction)_frameExists, METH_VARARGS, nullptr},
  {"_getFrameStrings", (PyCFunction)_getFrameStrings, METH_VARARGS, nullptr},
  {"_allFramesAsDot", (PyCFunction)_allFramesAsDot, METH_VARARGS | METH_KEYWORDS, nullptr},
  {"get_latest_common_time", (PyCFunction)getLatestCommonTime, METH_VARARGS, nullptr},
  {"lookup_transform_core", (PyCFunction)lookupTransformCore, METH_VARARGS | METH_KEYWORDS,
    nullptr},
  {"lookup_transform_full_core", (PyCFunction)lookupTransformFullCore, METH_VARARGS | METH_KEYWORDS,
    nullptr},
  // {"lookupTwistCore", (PyCFunction)lookupTwistCore, METH_VARARGS | METH_KEYWORDS},
  // {"lookupTwistFullCore", lookupTwistFullCore, METH_VARARGS},
  {nullptr, nullptr, 0, nullptr}
};

static PyMethodDef module_methods[] = {
  // {"Transformer", mkTransformer, METH_VARARGS},
  {nullptr, nullptr, 0, nullptr},
};

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif
static PyTypeObject buffer_core_Type = {
  PyVarObject_HEAD_INIT(nullptr, 0)
  "_tf2.BufferCore",                           /* tp_name */
  sizeof(buffer_core_t),                       /* tp_basicsize */
  // The other members are initialized in `staticInit`.
};
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

bool staticInit()
{
  // Initialize buffer_core_Type
  buffer_core_Type.tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE;
  buffer_core_Type.tp_methods = buffer_core_methods;
  buffer_core_Type.tp_init = BufferCore_init;
  buffer_core_Type.tp_alloc = PyType_GenericAlloc;
  buffer_core_Type.tp_new = PyType_GenericNew;

#if PYTHON_API_VERSION >= 1007
  tf2_exception =
    PyErr_NewException(const_cast<char *>("tf2.TransformException"), nullptr, nullptr);
  tf2_connectivityexception = PyErr_NewException(
    const_cast<char *>("tf2.ConnectivityException"),
    tf2_exception,
    nullptr);
  tf2_lookupexception = PyErr_NewException(
    const_cast<char *>("tf2.LookupException"), tf2_exception,
    nullptr);
  tf2_extrapolationexception = PyErr_NewException(
    const_cast<char *>("tf2.ExtrapolationException"),
    tf2_exception, nullptr);
  tf2_invalidargumentexception =
    PyErr_NewException(
    const_cast<char *>("tf2.InvalidArgumentException"),
    tf2_exception, nullptr);
  tf2_timeoutexception = PyErr_NewException(
    const_cast<char *>("tf2.TimeoutException"),
    tf2_exception, nullptr);
#else
  tf2_exception = stringToPython("tf2.error");
  tf2_connectivityexception = stringToPython("tf2.ConnectivityException");
  tf2_lookupexception = stringToPython("tf2.LookupException");
  tf2_extrapolationexception = stringToPython("tf2.ExtrapolationException");
  tf2_invalidargumentexception = stringToPython("tf2.InvalidArgumentException");
  tf2_timeoutexception = stringToPython("tf2.TimeoutException");
#endif

  pModulerclpy = pythonImport("rclpy");
  pModulerclpytime = pythonImport("rclpy.time");
  pModulebuiltininterfacesmsgs = pythonImport("builtin_interfaces.msg");
  pModulegeometrymsgs = pythonImport("geometry_msgs.msg");

  if (pModulerclpy == nullptr) {
    printf("Cannot load rclpy module");
    return false;
  }

  if (pModulerclpytime == nullptr) {
    printf("Cannot load rclpy.time.Time module");
    return false;
  }

  if (pModulegeometrymsgs == nullptr) {
    printf("Cannot load geometry_msgs module");
    return false;
  }

  if (pModulebuiltininterfacesmsgs == nullptr) {
    printf("Cannot load builtin_interfaces module");
    return false;
  }

  if (PyType_Ready(&buffer_core_Type) != 0) {
    return false;
  }
  return true;
}

PyObject * moduleInit(PyObject * m)
{
  PyModule_AddObject(m, "BufferCore", reinterpret_cast<PyObject *>(&buffer_core_Type));
  PyObject * d = PyModule_GetDict(m);
  PyDict_SetItemString(d, "TransformException", tf2_exception);
  PyDict_SetItemString(d, "ConnectivityException", tf2_connectivityexception);
  PyDict_SetItemString(d, "LookupException", tf2_lookupexception);
  PyDict_SetItemString(d, "ExtrapolationException", tf2_extrapolationexception);
  PyDict_SetItemString(d, "InvalidArgumentException", tf2_invalidargumentexception);
  PyDict_SetItemString(d, "TimeoutException", tf2_timeoutexception);
  return m;
}

struct PyModuleDef tf_module = {
  PyModuleDef_HEAD_INIT, /* m_base */
  "_tf2_py",             /* m_name */
  nullptr,                  /* m_doc */
  -1,                    /* m_size - state size (but we're using globals) */
  module_methods,        /* m_methods */
  nullptr,                  /* m_slots */
  nullptr,                  /* m_traverse */
  nullptr,                  /* m_clear */
  nullptr,                  /* m_free */
};

PyMODINIT_FUNC PyInit__tf2_py()
{
  if (!staticInit()) {
    return nullptr;
  }
  return moduleInit(PyModule_Create(&tf_module));
}
