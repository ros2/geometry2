tf2_kdl_py
==========

:mod:`tf2_kdl_py` is a set of ``PyKDL`` bindings written for the tf2 system for Python.

1.1 Converting Types
--------------------

To make these types univsersal with other ``tf2`` types, they must have conversion methods,
the easiest way to change to geometry_msgs types or back to ``PyKDL`` types is by calling:

* :meth:`tf2_ros.convert(TransformableObject, TransformableObjectType)`

Essentially take any valid ``geometry_msgs`` object or ``PyKDL`` object to be TransformableObject,
and the desired converted type to be TransformableObjectType, and the system will automatically
convert.

.. toctree::
    :maxdepth: 2

    self
    Python Modules <modules>

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
