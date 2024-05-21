Overview
========

This package contains the ROS 2 bindings for the tf2 library, for both Python and C++.

1.1 Broadcasting Transforms
---------------------------
  * :class:`tf2_ros::TransformBroadcaster()` constructor of
    (:ref:`exhale_class_classtf2__ros_1_1_transform_broadcaster`)
  * :class:`tf2_ros::TransformBroadcaster::sendTransform` to send transforms

Similarly static transforms can be sent by:

  * :class:`tf2_ros::StaticTransformBroadcaster()`, constructor of
    (:ref:`exhale_class_classtf2__ros_1_1_static_transform_broadcaster`)
  * :class:`tf2_ros::StaticTransformBroadcaster::sendTransform` to send static transforms

1.2 Using Published Transforms
------------------------------
For most purposes using tf2_ros will be done using tf2_ros::Buffer (:ref:`exhale_class_classtf2__ros_1_1_buffer`). It's main public API is defined by tf2_ros::BufferInterface (:ref:`exhale_class_classtf2__ros_1_1_buffer_interface`). Typically it will be populated using a tf2_ros::TransformListener (:ref:`exhale_class_classtf2__ros_1_1_transform_listener`) which subscribes to the appropriate topics.

  * :meth:`tf2_ros::Buffer::transform` is the main method for applying transforms.
  * :meth:`canTransform` allows to know if a transform is available
  * :meth:`lookupTransform` is a lower level method which returns the transform between two coordinate frames. This method is the core functionality of the tf2 library.
  * :meth:`getFrames` is a service method providing the frames in the graph as a yaml tree

1.3 Filtering Transforms
------------------------

``tf2_ros`` provides a feature which allows to pass only the messages once there is transform data available. This follows the pattern from the ``message_filters`` package. Here is a brief list of functions that the user is most likely to use.

  * :class:`tf2_ros::MessageFilter()` constructor of (:ref:`exhale_class_classtf2__ros_1_1_message_filter`)

  * :meth:`connectInput()` allows to connect filters together

  * :meth:`setTargetFrame()` set the frame you want to be able to transform to before getting a message callback

  * :meth:`setTargetFrames()` set the frames you want to be able to transform to before getting a message callback

  * :meth:`setTolerance()` specifies the time tolerance for the transform data

  * :meth:`clear()` flushes the message queue

  * :meth:`setQueueSize()` creates a maximum number of messages in the queue

1.4 Exceptions
--------------

Here is the list of exceptions that can be thrown by ``tf2_ros`` and are inherited from tf2.

  * tf2::ConnectivityException

  * tf2::LookupException

  * tf2::ExtrapolationException

  * tf2::InvalidArgumentException

  * tf2::TimeoutException

  * tf2::TransformException

Tree
====

.. toctree::
    :maxdepth: 2

    self
    cli_tools
    tf2_ros2
    C++ API Docs <generated/index>
    Python Modules <modules>


Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
