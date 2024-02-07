^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_tf2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.36.0 (2024-02-07)
-------------------
* normalize quaternions on tf2_eigen (`#644 <https://github.com/ros2/geometry2/issues/644>`_)
* Contributors: Paul Gesel

0.35.1 (2024-01-24)
-------------------

0.35.0 (2023-12-26)
-------------------

0.34.0 (2023-11-06)
-------------------

0.33.2 (2023-10-04)
-------------------

0.33.1 (2023-09-07)
-------------------

0.33.0 (2023-08-21)
-------------------

0.32.2 (2023-07-11)
-------------------

0.32.1 (2023-05-11)
-------------------

0.32.0 (2023-04-27)
-------------------

0.31.2 (2023-04-13)
-------------------

0.31.1 (2023-04-12)
-------------------

0.31.0 (2023-04-11)
-------------------

0.30.0 (2023-02-14)
-------------------
* Update the demos to C++17. (`#578 <https://github.com/ros2/geometry2/issues/578>`_)
* Contributors: Chris Lalancette

0.29.0 (2022-11-21)
-------------------
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`_)
* Contributors: Audrow Nash

0.28.0 (2022-11-02)
-------------------

0.27.0 (2022-09-13)
-------------------

0.26.2 (2022-08-15)
-------------------

0.26.1 (2022-06-24)
-------------------

0.26.0 (2022-04-29)
-------------------

0.25.0 (2022-04-05)
-------------------
* Fix more instances of Eigen problems on RHEL. (`#515 <https://github.com/ros2/geometry2/issues/515>`_)
* Install includes to include/${PROJECT_NAME} and use modern CMake (`#493 <https://github.com/ros2/geometry2/issues/493>`_)
* Contributors: Chris Lalancette, Shane Loretz

0.24.0 (2022-03-31)
-------------------

0.23.0 (2022-03-28)
-------------------
* Fix precision loss from using rclcpp::Time::seconds() (`#511 <https://github.com/ros2/geometry2/issues/511>`_)
* Contributors: Kenji Brameld

0.22.0 (2022-03-01)
-------------------

0.21.0 (2022-01-14)
-------------------

0.20.0 (2021-12-17)
-------------------

0.19.0 (2021-10-15)
-------------------
* More Intuitive CLI for Static Transform Publisher (`#392 <https://github.com/ros2/geometry2/issues/392>`_)
* Contributors: Hunter L. Allen

0.18.0 (2021-06-01)
-------------------
* Conversion tests for toMsg() (`#423 <https://github.com/ros2/geometry2/issues/423>`_)
* Deprecate tf2_geometry_msgs.h (`#418 <https://github.com/ros2/geometry2/issues/418>`_)
* Deprecate tf2_kdl.h (`#414 <https://github.com/ros2/geometry2/issues/414>`_)
* Deprecate tf2_bullet.h (`#412 <https://github.com/ros2/geometry2/issues/412>`_)
* Contributors: Bjar Ne, Chris Lalancette

0.17.1 (2021-04-06)
-------------------

0.17.0 (2021-03-19)
-------------------

0.16.0 (2021-01-25)
-------------------

0.15.1 (2020-12-08)
-------------------

0.15.0 (2020-11-02)
-------------------
* Update maintainers of the ros2/geometry2 fork. (`#328 <https://github.com/ros2/geometry2/issues/328>`_)
* Contributors: Chris Lalancette

0.14.1 (2020-09-21)
-------------------
* Activate usual compiler warnings and fix errors (`#270 <https://github.com/ros2/geometry2/issues/270>`_)
* Contributors: Ivan Santiago Paunovic

0.14.0 (2020-08-14)
-------------------
* Fix a TOCTTOU race in tf2. (`#307 <https://github.com/ros2/geometry2/issues/307>`_)
* Fixed memory leak in Buffer::waitForTransform (`#281 <https://github.com/ros2/geometry2/issues/281>`_)
* relax test timings to pass with Connext (`#304 <https://github.com/ros2/geometry2/issues/304>`_)
* Explicitly initialize instances of tf2::Duration (`#291 <https://github.com/ros2/geometry2/issues/291>`_)
* Generate callbacks after updating message\_ (`#274 <https://github.com/ros2/geometry2/issues/274>`_)
* fix test_static_publisher in macos (`#284 <https://github.com/ros2/geometry2/issues/284>`_)
* Fix up the dependencies in test_tf2. (`#277 <https://github.com/ros2/geometry2/issues/277>`_)
* Split tf2_ros in tf2_ros and tf2_ros_py (`#210 <https://github.com/ros2/geometry2/issues/210>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Dirk Thomas, Martin Ganeff, Michael Carroll, ymd-stella

0.13.4 (2020-06-03)
-------------------

0.13.3 (2020-05-26)
-------------------

0.13.2 (2020-05-18)
-------------------
* Fix deprecation warnings from launch (`#264 <https://github.com/ros2/geometry2/issues/264>`_)
* Contributors: Chris Lalancette

0.13.1 (2020-05-08)
-------------------

0.13.0 (2020-04-30)
-------------------
* Replace deprecated launch_ros usage (`#250 <https://github.com/ros2/geometry2/issues/250>`_)
* Remote ready_fn from launch_testing tests (`#243 <https://github.com/ros2/geometry2/issues/243>`_)
* [test_tf2] Call project() and ament_package() if not building tests (`#233 <https://github.com/ros2/geometry2/issues/233>`_)
* Porting test_tf2  (`#203 <https://github.com/ros2/geometry2/issues/203>`_)
* Contributors: Alejandro Hernández Cordero, Jacob Perron, Peter Baughman

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------
* Typos.
* Adds unit tests for TF loaded from parameter server.
  This tests both success (loading a valid TF into the param server) and
  failures (parameter does not exist, parameter contents are invalid).
* Code linting & reorganization
  - whitespace
  - indentation
  - re-organized code to remove duplications.
  whitespace & indentation changes only.
  simplified (de-duplicated) duplicate code.
  missing a duplicate variable.
  whitespace changes only.
* Contributors: Felix Duvallet

0.5.13 (2016-03-04)
-------------------
* Remove LGPL from license tags
  LGPL was erroneously included in 2a38724. As there are no files with it
  in the package.
* Contributors: Jochen Sprickerhof

0.5.12 (2015-08-05)
-------------------
* add utilities to get yaw, pitch, roll and identity transform
* provide more conversions between types
  The previous conversion always assumed that it was converting a
  non-message type to a non-message type. Now, one, both or none
  can be a message or a non-message.
* Contributors: Vincent Rabaud

0.5.11 (2015-04-22)
-------------------

0.5.10 (2015-04-21)
-------------------

0.5.9 (2015-03-25)
------------------

0.5.8 (2015-03-17)
------------------
* remove useless Makefile files
* Contributors: Vincent Rabaud

0.5.7 (2014-12-23)
------------------

0.5.6 (2014-09-18)
------------------

0.5.5 (2014-06-23)
------------------
* Removed AsyncSpinner workaround
* Contributors: Esteve Fernandez

0.5.4 (2014-05-07)
------------------
* Clean up warnings about autostart and add some assertions for coverage
* Contributors: Tully Foote

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
* fixing kdl linking for tests
* Contributors: Tully Foote

0.4.9 (2013-11-06)
------------------

0.4.8 (2013-11-06)
------------------
* Fixed static_transform_publisher duplicate check, added rostest.

0.4.7 (2013-08-28)
------------------

0.4.6 (2013-08-28)
------------------

0.4.5 (2013-07-11)
------------------
* fixing quaternion in unit test and adding a timeout on the waitForServer
* fixing usage string to show quaternions and using quaternions in the test app
* removing redundant declaration
* disabling whole cmake invocation in test_tf2 when not CATKIN_ENABLE_TESTING

0.4.4 (2013-07-09)
------------------

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------
* fixing test target dependencies
* fixing colliding target names between geometry and geometry_experimental
* stripping tf2_ros dependency from tf2_bullet.  Test was moved to test_tf2

0.4.0 (2013-06-27)
------------------
* splitting rospy dependency into tf2_py so tf2 is pure c++ library.
* switching to console_bridge from rosconsole
* moving convert methods back into tf2 because it does not have any ros dependencies beyond ros::Time which is already a dependency of tf2
* Cleaning up unnecessary dependency on roscpp
* converting contents of tf2_ros to be properly namespaced in the tf2_ros namespace
* Cleaning up packaging of tf2 including:
  removing unused nodehandle
  fixing overmatch on search and replace
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

0.3.4 (2013-02-15 13:14)
------------------------

0.3.3 (2013-02-15 11:30)
------------------------

0.3.2 (2013-02-15 00:42)
------------------------

0.3.1 (2013-02-14)
------------------

0.3.0 (2013-02-13)
------------------
* removing packages with missing deps
* catkinizing geometry-experimental
* add boost linkage
* fixing test for header cleanup
* fixing usage of bullet for migration to native bullet
* Cleanup on test code, all tests pass
* cleanup on optimized tests, still failing
* Cleanup in compound transform test
* Adding more frames to compound transform case
* Compound transform test fails on optimized case after more frames added
* Compound transform test has more frames in it
* Cleanup of compount transform test
* Compound transform at root node test fails for optimized branch
* compount transform test, non-optimized
* time-varying tests with different time-steps for optimized case
* Time-varying test inserts data at different time-steps for non-optimized case
* Helix (time-varying) test works on optimized branch
* Adding more complicated case to helix test
* Adding helix test for time-varying transforms in non-optimized case
* Corrected ring45 values in buffer core test
* Corrected values of ring45 test for non-optimized case
* Ring 45 test running on non-optimized tf2 branch, from Tully's commit r880
* filling out ring test case which finds errors in the optimization
* Add option to use a callback queue in the message filter
* another out-the-back test
* move the message filter to tf2_ros
* fix warnings
* merge from tf_rework
* tf2::MessageFilter + tests.  Still need to change it around to pass in a callback queue, since we're being triggered directly from the tf2 buffer
* adding in y configuration test
* a little more realistic
* Don't add the request if the transform is already available.  Add some new tests
* working transformable callbacks with a simple (incomplete) test case
* cleaning up test setup
* check_v implemented and passing v test and multi tree test
* working toward multi configuration tests
* removing restructuring for it won't nest like I thought
* continuing restructuring and filling in test case setup
* restructuring before scaling
* Completely remove lookupLists().  canTransform() now uses the same walking code as lookupTransform().  Also fixed a bug in the static transform publisher test
* testing chaining in a ring
* test dataset generator
* more complicated test with interleaving static and dynamic frames passing
* static transform tested and working
* test in progress, need to unshelve changes.
* tests passing and all throw catches removed too\!
* move to tf2_ros completed. tests pass again
* merge tf2_cpp and tf2_py into tf2_ros
* merging and fixing broken unittest
* Got transform with types working in python
* A working first version of transforming and converting between different types
* removing unused datatypes
* removing include of old tf from tf2
* testing new argument validation and catching bug
* unit test of single link one to try to debug eitan's client bug
* working towards interpolation too
* A working version of a test case for the python buffer client
* merging
* adding else to catch uncovered cases, and changing time for easier use
* Adding a test for the python buffer client
* using permuter now and doing a,b,c to a,b,c, at three different times including 0
* Moving tf2_tests to test_tf2
* moving test to new package
* initial package created for testing tf2
