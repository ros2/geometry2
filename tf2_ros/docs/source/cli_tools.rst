Command Line Tools
==================

There are a number of tools to help you debug tf related problems. Most of them are located inside the bin directory or the scripts directory. This page gives a description of each tool, and explains what type of problems you can resolve with each tool.

Frame Poses
===========

1. tf2_echo
-----------
tf2_echo is the simplest tool to look at the numeric values of a specific transform. tf2_echo takes two arguments: the reference frame and the target frame. The output of tf2_echo is the target frame represented in the reference frame. E.g. to get the transformation from turtle1 to turtle2, type:

.. code-block:: bash

  ros2 run tf2_ros tf2_echo turtle1 turtle2

The expected output looks something like this:

.. code-block:: bash

   At time 1253924110.083
    - Translation: [-1.877, 0.415, 0.000]
    - Rotation: in Quaternion [0.000, 0.000, -0.162, 0.987]
            in RPY [0.000, -0.000, -0.325]
   At time 1253924111.082
    - Translation: [-1.989, 0.151, 0.000]
    - Rotation: in Quaternion [0.000, 0.000, -0.046, 0.999]
            in RPY [0.000, -0.000, -0.092]

2. RVIZ2
--------
Run `rviz2` with `tf` enabled and begin viewing frames to see transforms

.. image:: images/rviz_screenshot.png

TF tree introspection
======================
1. Viewing TF trees
---------------------
1.1 Save as a file by view_frames
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
View frames can generate a pdf file with a graphical representation of the complete tf tree. It also generates a number of time-related statistics. To run view frames, type:

.. code-block:: bash

   ros2 run tf2_tools view_frames

In the current working folder, you should now have a file called "frames_$(data)_$(time).pdf". Open the file, you should see something like this:

.. image:: images/view_frames.png

Fields
^^^^^^^^^
  * Recorded at time: shows the absolute timestamp when this graph was generated.
  * Broadcaster: gives the name of the node that broadcasted the corresponding transform.
  * Average rate: gives the average frequency at which the broadcaster sent out the corresponding transform. Note that this is an average, and does not guarantee that the broadcaster was sending transforms the whole time.
  * Buffer length: tells you how many seconds of data is available in the tf buffer. When you run view frames without specifying a node, this buffer length should be about 5 seconds.
  * Most recent transform: states how long ago the last transform was received. This is the time delay of a transform.
  * Oldest transform: states how long ago the first transform was received.

1.2 Query a running node
^^^^^^^^^^^^^^^^^^^^^^^^
If a specific node is having trouble its exact data can be queried using the following command:

.. code-block:: bash

  ros2 run tf2_tools view_frames --node=NODE_NAME

1.3 Dynamically introspect during runtime
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`rqt_tf_tree <https://github.com/ros-visualization/rqt_tf_tree/tree/master>`_  provides a GUI to introspect tf tree during runtime.

2. tf2_monitor
--------------
tf2_monitor can give you a lot of detailed information about a specific transformation you care about. The monitor will break down the chain between two frames into individual transforms, and provide statistics about timing, broadcasters, etc.

E.g. you want more information about the transformation between the frame "turtle1" and the frame "turtle2", simply type:

.. code-block:: bash

   ros2 run tf2_ros tf2_monitor turtle1 turtle2

The output should look something like this:

.. code-block:: bash

  RESULTS: for turtle1 to turtle2
  Chain is: turtle2
  Net delay     avg = 0.00296015: max = 0.0239079

  Frames:
  Frame: turtle2, published by <no authority available>, Average Delay: 0.00385465, Max Delay: 0.00637698

  Broadcasters:
  Node: /turtle1_tf_broadcaster 40.01705 Hz, Average Delay: 0.0001427 Max Delay: 0.0003479
  Node: /turtle2_tf_broadcaster 40.01705 Hz, Average Delay: 0.0001515 Max Delay: 0.00034

Each of these frames can be published by a different broadcaster.

