^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_geometry_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.13.10 (2021-04-14)
--------------------

0.13.9 (2020-12-09)
-------------------

0.13.8 (2020-12-08)
-------------------

0.13.7 (2020-11-18)
-------------------

0.13.6 (2020-10-28)
-------------------
* [Foxy backport] Add PoseWithCovarianceStamped transform support (`#336 <https://github.com/ros2/geometry2/issues/336>`_)
* Update maintainers of the ros2/geometry2 fork. (`#328 <https://github.com/ros2/geometry2/issues/328>`_) (`#332 <https://github.com/ros2/geometry2/issues/332>`_)
* Don't install python tf2_geometry_msgs (`#299 <https://github.com/ros2/geometry2/issues/299>`_) (`#302 <https://github.com/ros2/geometry2/issues/302>`_)
* Contributors: Alejandro Hernández Cordero, Nikolai Morin, Shane Loretz

0.13.5 (2020-08-05)
-------------------

0.13.4 (2020-06-03)
-------------------
* export targets in addition to include directories / libraries (`#271 <https://github.com/ros2/geometry2/issues/271>`_)
* Contributors: Dirk Thomas

0.13.3 (2020-05-26)
-------------------

0.13.2 (2020-05-18)
-------------------

0.13.1 (2020-05-08)
-------------------

0.13.0 (2020-04-30)
-------------------
* Added doxyfiles and sphinx Makefiles (`#257 <https://github.com/ros2/geometry2/issues/257>`_)
* add missing test dependency (`#256 <https://github.com/ros2/geometry2/issues/256>`_)
* use target_include_directories (`#231 <https://github.com/ros2/geometry2/issues/231>`_)
* installed python tf2_geometry_msgs (`#207 <https://github.com/ros2/geometry2/issues/207>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Karsten Knese

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
* Remove template specialization for toMsg functions (`#179 <https://github.com/ros2/geometry2/issues/179>`_)
* Use smart pointers for global buffer variables in tests
* Don't assume quaternions init to all zeros
* Contributors: Jacob Perron, Josh Langsfeld

0.12.0 (2019-09-26)
-------------------

0.11.3 (2019-05-24)
-------------------

0.11.2 (2019-05-20)
-------------------

0.11.1 (2019-05-09)
-------------------

0.11.0 (2019-04-14)
-------------------
* Updated to use ament_target_dependencies where possible. (`#98 <https://github.com/ros2/geometry2/issues/98>`_)

0.10.1 (2018-12-06)
-------------------

0.10.0 (2018-11-22)
-------------------
* Use ros2 time (`#67 <https://github.com/ros2/geometry2/issues/67>`_)
  * Buffer constructor accepts a clock
  * Use rclcpp::Time::seconds()
* revert now unnecessary message initializations (`#64 <https://github.com/ros2/geometry2/issues/64>`_)
* Contributors: Mikael Arguedas, Shane Loretz

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------
* Add doxygen documentation for tf2_geometry_msgs
* Contributors: Jackie Kay

0.5.13 (2016-03-04)
-------------------
* Add missing python_orocos_kdl dependency
* make example into unit test
* vector3 not affected by translation
* Contributors: Daniel Claes, chapulina

0.5.12 (2015-08-05)
-------------------
* Merge pull request `#112 <https://github.com/ros/geometry_experimental/issues/112>`_ from vrabaud/getYaw
  Get yaw
* add toMsg and fromMsg for QuaternionStamped
* Contributors: Tully Foote, Vincent Rabaud

0.5.11 (2015-04-22)
-------------------

0.5.10 (2015-04-21)
-------------------

0.5.9 (2015-03-25)
------------------

0.5.8 (2015-03-17)
------------------
* remove useless Makefile files
* tf2 optimizations
* add conversions of type between tf2 and geometry_msgs
* fix ODR violations
* Contributors: Vincent Rabaud

0.5.7 (2014-12-23)
------------------
* fixing transitive dependency for kdl. Fixes `#53 <https://github.com/ros/geometry_experimental/issues/53>`_
* Contributors: Tully Foote

0.5.6 (2014-09-18)
------------------

0.5.5 (2014-06-23)
------------------

0.5.4 (2014-05-07)
------------------

0.5.3 (2014-02-21)
------------------

0.5.2 (2014-02-20)
------------------

0.5.1 (2014-02-14)
------------------

0.5.0 (2014-02-14)
------------------

0.4.10 (2013-12-26)
-------------------

0.4.9 (2013-11-06)
------------------

0.4.8 (2013-11-06)
------------------

0.4.7 (2013-08-28)
------------------

0.4.6 (2013-08-28)
------------------

0.4.5 (2013-07-11)
------------------

0.4.4 (2013-07-09)
------------------
* making repo use CATKIN_ENABLE_TESTING correctly and switching rostest to be a test_depend with that change.

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------

0.4.0 (2013-06-27)
------------------
* moving convert methods back into tf2 because it does not have any ros dependencies beyond ros::Time which is already a dependency of tf2
* Cleaning up unnecessary dependency on roscpp
* converting contents of tf2_ros to be properly namespaced in the tf2_ros namespace
* Cleaning up packaging of tf2 including:
  removing unused nodehandle
  cleaning up a few dependencies and linking
  removing old backup of package.xml
  making diff minimally different from tf version of library
* Restoring test packages and bullet packages.
  reverting 3570e8c42f9b394ecbfd9db076b920b41300ad55 to get back more of the packages previously implemented
  reverting 04cf29d1b58c660fdc999ab83563a5d4b76ab331 to fix `#7 <https://github.com/ros/geometry_experimental/issues/7>`_

0.3.6 (2013-03-03)
------------------

0.3.5 (2013-02-15 14:46)
------------------------
* 0.3.4 -> 0.3.5

0.3.4 (2013-02-15 13:14)
------------------------
* 0.3.3 -> 0.3.4

0.3.3 (2013-02-15 11:30)
------------------------
* 0.3.2 -> 0.3.3

0.3.2 (2013-02-15 00:42)
------------------------
* 0.3.1 -> 0.3.2

0.3.1 (2013-02-14)
------------------
* 0.3.0 -> 0.3.1

0.3.0 (2013-02-13)
------------------
* switching to version 0.3.0
* add setup.py
* added setup.py etc to tf2_geometry_msgs
* adding tf2 dependency to tf2_geometry_msgs
* adding tf2_geometry_msgs to groovy-devel (unit tests disabled)
* fixing groovy-devel
* removing bullet and kdl related packages
* disabling tf2_geometry_msgs due to missing kdl dependency
* catkinizing geometry-experimental
* catkinizing tf2_geometry_msgs
* add twist, wrench and pose conversion to kdl, fix message to message conversion by adding specific conversion functions
* merge tf2_cpp and tf2_py into tf2_ros
* Got transform with types working in python
* A working first version of transforming and converting between different types
* Moving from camelCase to undescores to be in line with python style guides
* Fixing tests now that Buffer creates a NodeHandle
* add posestamped
* import vector3stamped
* add support for Vector3Stamped and PoseStamped
* add support for PointStamped geometry_msgs
* add regression tests for geometry_msgs point, vector and pose
* Fixing missing export, compiling version of buffer_client test
* add bullet transforms, and create tests for bullet and kdl
* working transformations of messages
* add support for PoseStamped message
* test for pointstamped
* add PointStamped message transform methods
* transform for vector3stamped message
