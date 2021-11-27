Architecture of ``tf2``
=======================

``tf2`` is a transform library designed to provide implementation of interface that keeps track of multiple coordinate frames over time.
``tf2`` maintains the relationship between coordinate frames in a tree structure buffered in time, and lets the user transform data between any two coordinate frames at any desired point in time.
The high level goal is to allow developers and users not to have to worry about which coordinate frame any specific data is stored in.

Main Interface
--------------

The main interface is defined through the `tf2::BufferCore <https://docs.ros2.org/latest/api/tf2/buffer__core_8h.html>`_ interface.
It uses the exceptions in `exceptions.h <https://docs.ros2.org/latest/api/tf2/exceptions_8h.html>`_ and the :class:`tf2::Stamped` datatype in `transform_datatypes.h <https://docs.ros2.org/latest/api/tf2/transform__datatypes_8h.html>`_.

Conversion Interface
--------------------

``tf2`` offers a templated conversion interface for external libraries to specify conversions between ``tf2``-specific data types and user-defined data types.
Various templated functions in ``tf2_ros`` use the conversion interface to apply transformations from the tf server to these custom datatypes.
The conversion interface is defined in `tf2/convert.h <https://docs.ros2.org/latest/api/tf2/convert_8h.html>`_.

Buffer Core: Record and Lookup Relations Between Frames
-------------------------------------------------------

The ``tf2`` library implements the interface defined by :class:`tf2::BufferCore`.
This class and all classes derived from it are responsible for providing coordinate transforms between any two frames in a system.
This class provides a simple interface to allow recording and lookup of relationships between arbitrary frames of the system.

``tf2`` assumes that there is a tree of coordinate frame transforms which define the relationship between all coordinate frames.
After transformation relationships are supplied, query specifiyng target frame, source frame, and time point can be used to obtain required data.
``tf2`` will take care of all the intermediate transfromation steps for specific queries.

Additionally, ``tf2`` has a feature of data interpolation.
The probability that a specific query would be at the same timestamp of all the frames in the system is very unlikely in an asynchronous system with nanosecond precision.
Therefore, for any given query it can be expected that data is interpolated.

It should be noted that buffer implicitly limits the maximum cache size of 10s by default as defined by the :const:`tf2::TIMECACHE_DEFAULT_MAX_STORAGE_TIME` and it cannot interpolate outside of the cache history.
Thus there is a risk of incurring extrapolation limits based on specific system.

Buffer Core Methods
^^^^^^^^^^^^^^^^^^^

The :class:`tf2::BufferCore` contains useful methods to update the existing tf buffer.

- :meth:`tf2::BufferCore::clear`

  - This method clears all data in the buffer.

- :meth:`tf2::BufferCore::setTransform`

  - This method will add  :class:`geometry_msgs::msg::TransformStamped` information to the tf data structure.

- :meth:`tf2::BufferCore::lookupTransform`

  - This method returns the transform between two frames by frame ID.
  - Possible exceptions :class:`tf2::LookupException`, :class:`tf2::ConnectivityException`, :class:`tf2::ExtrapolationException`, :class:`tf2::InvalidArgumentException`.

- :meth:`tf2::BufferCore::canTransform`

  - This method tests if a transform is possible.

- :meth:`tf2::BufferCore::getAllFrameNames`

  - This method returns all frames that exist in the system.

- :meth:`tf2::BufferCore::allFramesAsYAML`

  - This method allows to see what frames have been cached in yaml format and is useful for debugging tools.

- :meth:`tf2::BufferCore::allFramesAsString`

  - This method allows to see what frames have been cached and is useful for debugging.

Supported Datatypes
-------------------

``tf2`` implements templated datatype support.
This allows the core packages to have minimal dependencies and there be packages which add support for converting to and from different datatypes as well as transforming those data types.
``tf2`` does have an internal datatypes which are based on ``bullet``'s LinearMath library.
However it's recommended to use a fully supported math datatype which best supports your application.
``tf2`` conversion methods also support converting between and transforming between multiple different datatypes too.

At it's core ``tf2`` relies on the `stamped data types <https://docs.ros2.org/latest/api/tf2/classtf2_1_1Stamped.html>`_ which can be conveniently correlated to ROS 2 messages which have a Header.

Data Type Support Packages
^^^^^^^^^^^^^^^^^^^^^^^^^^

These packages provide methods to allow ``tf2`` to work natively with data types of any external library.
Most are either C++ or Python specific.

  * `tf2_bullet <https://docs.ros2.org/latest/api/tf2_bullet/>`_

    - ``tf2`` methods to work with bullet datatypes natively in C++.

  * `tf2_eigen <https://docs.ros2.org/latest/api/tf2_eigen/>`_

    - ``tf2`` methods to work with Eigen datatypes natively in C++.

  * `tf2_geometry_msgs <https://docs.ros2.org/latest/api/tf2_geometry_msgs/>`_

    - ``tf2`` methods to work with geometry_msgs datatypes natively in C++ or Python.

  * `tf2_kdl <https://docs.ros2.org/latest/api/tf2_kdl/>`_

    - ``tf2`` methods to work with kdl datatypes natively in C++ or Python.

  * tf2_sensor_msgs

    - ``tf2`` methods to work with sensor_msgs datatypes natively in C++ or Python.

Coordinate Frame Conventions
----------------------------

An important part of using ``tf2`` is to use standard conventions for coordinate frames.
There are several sources of conventions for using coordinate frames.

- Units, orientation conventions, chirality, rotation representations, and covariance representations are covered in `REP 103 <https://www.ros.org/reps/rep-0103.html>`_.

- Standard names for mobile base coordinate frames are covered in `REP 105 <https://www.ros.org/reps/rep-0105.html>`_.

- Standard coordinate frames for Humanoid Robots are in `REP 120 <https://www.ros.org/reps/rep-0120.html>`_.

Geometry
--------

``tf2`` provides basic geometry data types, such as

  - `Vector3 <https://docs.ros2.org/latest/api/tf2/Vector3_8h.html>`_

  - `Matrix3x3 <https://docs.ros2.org/latest/api/tf2/Matrix3x3_8h.html>`_

  - `Quaternion <https://docs.ros2.org/latest/api/tf2/Quaternion_8h.html>`_

  - `Transform <https://docs.ros2.org/latest/api/tf2/Transform_8h.html>`_

These data types support linear algebra operations between each other.

High level Design
-----------------

- A distributed system:

  - Purpose: No bottle neck process and all processes are one step away for minimal latency.
  - Implementation: Everything is broadcast and reassembled at end consumer points.
    There can be multiple data sources for tf information.
    Data is not required to be synchronized by using interpolation, so data can arrive out of order.

- Only transform data between coordinate frames at the time of use:

  - Purpose: Efficiency, both computational, bandwidth, and simplicity.
  - Implementation: Transform data between given frames only when required.

- Support queries on data which are timestamped at times other than the current time:

  - Purpose: Handle data processing lag gracefully.
  - Implementation: Interface class stores all transform data in memory and traverses tree on request.

- Only have to know the name of the coordinate frame to work with data:

  - Purpose: Ease of use for users/developers.
  - Implementation: Use string ``frame_ids`` as unique identifiers.

- The system doesn't need to know about the configuration before hand and can handle reconfiguring on the fly:

  - Purpose: Generic system for any configuration.
  - Implementation: Use directed tree structure.
    It allows fast traversal (order ``n`` where ``n`` is the depth of the tree) when evaluating a transform.
    It can be reconfigured simply by redefining a link.
    It does not require any structure verification or maintenance of the data structure, except for maintaining a sorted linked list of data for each link.

- Core is ROS agnostic:

  - Purpose: Code reuse.
  - Implementation: Core library is C++ class.
    A second class provides ROS interface and instantiates the core library.

- Thread Safe Interface:

  - Purpose: Can be used in a multithreaded program.
  - Implementation: Mutexes around data storage for each frame.
    Mutexes around ``frame_id`` lookup map.
    Each are individually locked and unlocked, neither can block the other.

- Multi-Robot Support:

  - Purpose: Can be used with multiple robots with the same or similar configuration.
  - Implementation: Use a ``tf_prefix`` similar to a namespace for each robot.

- Native Datatype Interfaces:

  - Purpose: Users can interact with ``tf2_ros`` in their native datatypes, the conversion is handled implicitly by the library.
  - Implementation: There is a ``tf2::convert(A, B)`` templated method that converts from type A to type B using the ``geometry_msgs`` types as the common factor.

  And as long as any datatype provides the methods ``msgType toMsg(datatype)`` and ``fromMsg(msgType, datatype)`` it can be automatically converted to any other datatype with the same methods defined and a matching ``msgType``.
  All ``tf2_ros`` interfaces can then be called with native type in and native type out.
  Note, the native type in and out do not need to match.
