Overview
========

This package contains the ROS 2 bindings for the tf2 library, for both Python and C++.

1.1 Broadcasting Transforms
---------------------------
  * :class:`tf2_ros::TransformBroadcaster()`, constructor
  * :class:`tf2_ros::TransformBroadcaster::sendTransform` to send transforms

Similarly static transforms can be sent by:

  * :class:`tf2_ros::StaticTransformBroadcaster()`, constructor,
  * :class:`tf2_ros::StaticTransformBroadcaster::sendTransform` to send static transforms

1.2 Using Published Transforms
------------------------------
For most purposes using tf2_ros will be done using tf2_ros::Buffer. It's main public API is defined by tf2_ros::BufferInterface. Typically it will be populated using a tf2_ros::TransformListener which subscribes to the appropriate topics.

  * :class:`tf2_ros::Buffer::transform` is the main method for applying transforms.
  * :class:`canTransform` allows to know if a transform is available
  * :class:`lookupTransform` is a lower level method which returns the transform between two coordinate frames. This method is the core functionality of the tf2 library.
  * :class:`getFrames` is a service method providing the frames in the graph as a yaml tree

1.3 Filtering Transforms
------------------------

tf2_ros provides a feature which allows to pass only the messages once there is transform data available. This follows the pattern from the message_filters package. Here is a brief list of functions that the user is most likely to use.

  * :class:`tf2_ros::MessageFilter()`, constructor

  * :class:`connectInput()` allows to connect filters together

  * :class:`setTargetFrame()` set the frame you want to be able to transform to before getting a message callback

  * :class:`setTargetFrames()` set the frames you want to be able to transform to before getting a message callback

  * :class:`setTolerance()` specifies the time tolerance for the transform data

  * :class:`clear()` flushes the message queue

  * :class:`setQueueSize()` creates a maximum number of messages in the queue

1.4 Exceptions
--------------

Here is the list of exceptions that can be thrown by tf2_ros and are inherited from tf2.

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
    api/index



Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
