^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_sensor_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.12.5 (2020-01-17)
-------------------

0.12.4 (2019-11-19)
-------------------

0.12.3 (2019-11-18)
-------------------
* Remove unused setup.py files (`#190 <https://github.com/ros2/geometry2/issues/190>`_)
* Contributors: Vasilii Artemev

0.12.2 (2019-11-18)
-------------------

0.12.1 (2019-10-23)
-------------------
* Use smart pointers for global buffer variables in tests
* Contributors: Josh Langsfeld

0.12.0 (2019-09-26)
-------------------
* Use eigen3_cmake_module (`#144 <https://github.com/ros2/geometry2/issues/144>`_)
* Added missing header (for tf2::fromMsg) (`#126 <https://github.com/ros2/geometry2/issues/126>`_)
* Contributors: Esteve Fernandez, Shane Loretz

0.11.3 (2019-05-24)
-------------------

0.11.2 (2019-05-20)
-------------------

0.11.1 (2019-05-09)
-------------------

0.11.0 (2019-04-14)
-------------------

0.10.1 (2018-12-06)
-------------------

0.10.0 (2018-11-22)
-------------------
* Remove cmake_modules dependency from package.xml (`#83 <https://github.com/ros2/geometry2/issues/83>`_)
* Fix Eigen3 dependency. (`#77 <https://github.com/ros2/geometry2/issues/77>`_)
* Porting tf2_sensor_msgs in C++ (`#2 <https://github.com/ros2/geometry2/issues/2>`_) (`#75 <https://github.com/ros2/geometry2/issues/75>`_)
* Contributors: Jacob Perron, Michael Carroll, Ruffin, Steven Macenski

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------

0.5.13 (2016-03-04)
-------------------
* add missing Python runtime dependency
* fix wrong comment
* Adding tests to package
* Fixing do_transform_cloud for python
  The previous code was not used at all (it was a mistake in the __init_\_.py so
  the do_transform_cloud was not available to the python users).
  The python code need some little correction (e.g there is no method named
  read_cloud but it's read_points for instance, and as we are in python we can't
  use the same trick as in c++ when we got an immutable)
* Contributors: Laurent GEORGE, Vincent Rabaud

0.5.12 (2015-08-05)
-------------------

0.5.11 (2015-04-22)
-------------------

0.5.10 (2015-04-21)
-------------------

0.5.9 (2015-03-25)
------------------

0.5.8 (2015-03-17)
------------------
* ODR violation fixes and more conversions
* Fix keeping original pointcloud header in transformed pointcloud
* Contributors: Paul Bovbel, Tully Foote, Vincent Rabaud

0.5.7 (2014-12-23)
------------------
* add support for transforming sensor_msgs::PointCloud2
* Contributors: Vincent Rabaud
