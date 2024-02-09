^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2
^^^^^^^^^^^^^^^^^^^^^^^^^

0.36.0 (2024-02-07)
-------------------

0.35.1 (2024-01-24)
-------------------
* Fix constantly increasing memory in std::list (`#636 <https://github.com/ros2/geometry2/issues/636>`_)
* Contributors: Ignacio Vizzo

0.35.0 (2023-12-26)
-------------------
* Update the tf2 documentation (`#638 <https://github.com/ros2/geometry2/issues/638>`_)
* Contributors: Chris Lalancette

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
* Fix error code returned in BufferCore::walkToTopParent (`#601 <https://github.com/ros2/geometry2/issues/601>`_)
* Contributors: Patrick Roncagliolo

0.32.0 (2023-04-27)
-------------------

0.31.2 (2023-04-13)
-------------------

0.31.1 (2023-04-12)
-------------------

0.31.0 (2023-04-11)
-------------------
* Depend on ament_cmake_ros to default SHARED to ON (`#591 <https://github.com/ros2/geometry2/issues/591>`_)
* Fix a potential crash in TimeCache::findClosest (`#592 <https://github.com/ros2/geometry2/issues/592>`_)
* Extend TimeCache API to provide rich ExtrapolationException infos (`#586 <https://github.com/ros2/geometry2/issues/586>`_)
* Contributors: Chris Lalancette, Patrick Roncagliolo, Tyler Weaver

0.30.0 (2023-02-14)
-------------------
* Update geometry2 to C++17 (`#584 <https://github.com/ros2/geometry2/issues/584>`_)
* Contributors: Chris Lalancette

0.29.0 (2022-11-21)
-------------------
* Include required header Scalar.h (`#559 <https://github.com/ros2/geometry2/issues/559>`_)
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`_)
* Contributors: Audrow Nash, Shane Loretz

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
* Install includes to include/${PROJECT_NAME} and use modern CMake (`#493 <https://github.com/ros2/geometry2/issues/493>`_)
* Contributors: Shane Loretz

0.24.0 (2022-03-31)
-------------------

0.23.0 (2022-03-28)
-------------------
* forward declare fromMsg to avoid missing symbols in downstream libraries (`#485 <https://github.com/ros2/geometry2/issues/485>`_)
* Contributors: João C. Monteiro

0.22.0 (2022-03-01)
-------------------

0.21.0 (2022-01-14)
-------------------
* tf2: Enable common linter tests (`#469 <https://github.com/ros2/geometry2/issues/469>`_)
* Contributors: Abrar Rahman Protyasha

0.20.0 (2021-12-17)
-------------------
* Move time functions into time.cpp.
* Change a for loop to a while loop.
* Switch to C++-style casts.
* Remove totally unused (and unreachable) code.
* Replace NULL with nullptr.
* Fix up some comments.
* Use std::make_shared where we can.
* Replace two comparisons with empty string to empty().
* Make sure to include-what-you-use.
* Remove unnecessary internal method.
* Remove long-deprecated walkToTopParent overload.
* Contributors: Chris Lalancette

0.19.0 (2021-10-15)
-------------------
* Remove unnecessary test dependencies.
* Remove some references to the ROS 1 wiki.
* Add rosidl_runtime_cpp as build_depend and build_export_depend.
* Minor cleanups in CMakeLists.txt.
* Remove include directory that doesn't exist.
* Remove completely unnecessary target_link_libraries.
* Remove unused speed_test from tf2.
* Suppress clang warnings about enumerator attributes. (`#463 <https://github.com/ros2/geometry2/issues/463>`_)
* Change TF2Error names to be a bit more descriptive. (`#349 <https://github.com/ros2/geometry2/issues/349>`_)
* Fixed errors due to missing header link. (`#432 <https://github.com/ros2/geometry2/issues/432>`_)
* Contributors: Chris Lalancette, Shivam Pandey

0.18.0 (2021-06-01)
-------------------
* Deprecate tf2_geometry_msgs.h (`#418 <https://github.com/ros2/geometry2/issues/418>`_)
* Speedup covariance unwrapping (`#399 <https://github.com/ros2/geometry2/issues/399>`_)
* Contributors: Chris Lalancette, Dima Dorezyuk

0.17.1 (2021-04-06)
-------------------
* Change index.ros.org -> docs.ros.org. (`#394 <https://github.com/ros2/geometry2/issues/394>`_)
* Contributors: Chris Lalancette

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
* Active usual compiler warnings in tf2 (`#322 <https://github.com/ros2/geometry2/issues/322>`_)
* Cleanups in buffer_core.cpp. (`#301 <https://github.com/ros2/geometry2/issues/301>`_)
* Add PoseWithCovarianceStamped transform support (`#312 <https://github.com/ros2/geometry2/issues/312>`_)
* Contributors: Chris Lalancette, Ivan Santiago Paunovic, Joshua Whitley

0.14.0 (2020-08-14)
-------------------
* Fix a TOCTTOU race in tf2. (`#307 <https://github.com/ros2/geometry2/issues/307>`_)
* Fixed memory leak in Buffer::waitForTransform (`#281 <https://github.com/ros2/geometry2/issues/281>`_)
* Add common linters to tf2. (`#258 <https://github.com/ros2/geometry2/issues/258>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Martin Ganeff

0.13.4 (2020-06-03)
-------------------
* Add missing virtual destructors (`#272 <https://github.com/ros2/geometry2/issues/272>`_)
* Contributors: Ivan Santiago Paunovic

0.13.3 (2020-05-26)
-------------------

0.13.2 (2020-05-18)
-------------------
* Modify error message to not match the pattern for Jenkins MSBuild errors (`#265 <https://github.com/ros2/geometry2/issues/265>`_)
* Contributors: Dirk Thomas

0.13.1 (2020-05-08)
-------------------
* export modern CMake interface target (`#263 <https://github.com/ros2/geometry2/issues/263>`_)
* Contributors: Dirk Thomas

0.13.0 (2020-04-30)
-------------------
* Added doxyfiles and sphinx Makefiles (`#257 <https://github.com/ros2/geometry2/issues/257>`_)
* Fix displayTimePoint truncation error (`#253 <https://github.com/ros2/geometry2/issues/253>`_)
* rename rosidl_generator_cpp namespace to rosidl_runtime_cpp (`#248 <https://github.com/ros2/geometry2/issues/248>`_)
* Use the new rcutils_strerror function. (`#239 <https://github.com/ros2/geometry2/issues/239>`_)
* Remove unnecessary semicolons. (`#235 <https://github.com/ros2/geometry2/issues/235>`_)
* Export all tf2 dependencies. (`#238 <https://github.com/ros2/geometry2/issues/238>`_)
* Fix a deprecated copy warning by implementing the assignment operator (`#201 <https://github.com/ros2/geometry2/issues/201>`_)
* tf2 add windows cmath constants (`#217 <https://github.com/ros2/geometry2/issues/217>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Dirk Thomas, Hunter L. Allen, Michel Hidalgo, brawner

0.12.4 (2019-11-19)
-------------------

0.12.3 (2019-11-18)
-------------------
* Provide more available error messaging for nonexistent and invalid frames in canTransform (`ros2 #187 <https://github.com/ros2/geometry2/issues/187>`_)
* Contributors: Emerson Knapp

0.12.2 (2019-11-18)
-------------------
* Fix up -Wcast-qual warning (`#193 <https://github.com/ros2/geometry2/issues/193>`_) (`#197 <https://github.com/ros2/geometry2/issues/197>`_)
* Contributors: Chris Lalancette

0.12.1 (2019-10-23)
-------------------
* Overwrite TimeCacheInterface type with a current input (`#151 <https://github.com/ros2/geometry2/issues/151>`_)
* [tf2] Use ament_target_dependencies where possible
* Restore conversion via message traits (`#167 <https://github.com/ros2/geometry2/issues/167>`_)
* Contributors: Jacob Perron, Michael Carroll, Vinnam Kim

0.12.0 (2019-09-26)
-------------------
* Add pure virtual interface tf2::BufferCoreInterface
* Guard against invalid iterator (`#127 <https://github.com/ros2/geometry2/issues/127>`_)
* Contributors: Jacob Perron

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
* Merge pull request `#65 <https://github.com/ros2/geometry2/issues/65>`_ from bsinno/bugfix/fix_identity_transform_behaviour
  Fix bug in lookupTransform()
* Fix lookupTransform() behaviour when transforming from a frame to itself
* revert now unnecessary message initializations (`#64 <https://github.com/ros2/geometry2/issues/64>`_)
* use console_bridge_vendor (`#63 <https://github.com/ros2/geometry2/issues/63>`_)
* Contributors: Alessandro Bottero, Mikael Arguedas, Tully Foote

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------
* fixes `#194 <https://github.com/ros/geometry2/issues/194>`_ check for quaternion normalization before inserting into storage (`#196 <https://github.com/ros/geometry2/issues/196>`_)
  * check for quaternion normalization before inserting into storage
  * Add test to check for transform failure on invalid quaternion input
* updating getAngleShortestPath() (`#187 <https://github.com/ros/geometry2/issues/187>`_)
* Move internal cache functions into a namespace
  Fixes https://github.com/ros/geometry2/issues/175
* Link properly to convert.h
* Landing page for tf2 describing the conversion interface
* Fix comment on BufferCore::MAX_GRAPH_DEPTH.
* Contributors: Jackie Kay, Phil Osteen, Tully Foote, alex, gavanderhoorn

0.5.13 (2016-03-04)
-------------------

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
* move lct_cache into function local memoryfor `#92 <https://github.com/ros/geometry_experimental/issues/92>`_
* Clean up range checking. Re: `#92 <https://github.com/ros/geometry_experimental/issues/92>`_
* Fixed chainToVector
* release lock before possibly invoking user callbacks. Fixes `#91 <https://github.com/ros/geometry_experimental/issues/91>`_
* Contributors: Jackie Kay, Tully Foote

0.5.9 (2015-03-25)
------------------
* fixing edge case where two no frame id lookups matched in getLatestCommonTime
* Contributors: Tully Foote

0.5.8 (2015-03-17)
------------------
* change from default argument to overload to avoid linking issue `#84 <https://github.com/ros/geometry_experimental/issues/84>`_
* remove useless Makefile files
* Remove unused assignments in max/min functions
* change _allFramesAsDot() -> _allFramesAsDot(double current_time)
* Contributors: Jon Binney, Kei Okada, Tully Foote, Vincent Rabaud

0.5.7 (2014-12-23)
------------------

0.5.6 (2014-09-18)
------------------

0.5.5 (2014-06-23)
------------------
* convert to use console bridge from upstream debian package https://github.com/ros/rosdistro/issues/4633
* Fix format string
* Contributors: Austin, Tully Foote

0.5.4 (2014-05-07)
------------------
* switch to boost signals2 following `ros/ros_comm#267 <https://github.com/ros/ros_comm/issues/267>`_, blocking `ros/geometry#23 <https://github.com/ros/geometry/issues/23>`_
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
* updated error message. fixes `#38 <https://github.com/ros/geometry_experimental/issues/38>`_
* tf2: add missing console bridge include directories (fix `#48 <https://github.com/ros/geometry_experimental/issues/48>`_)
* Fix const correctness of tf2::Vector3 rotate() method
  The method does not modify the class thus should be const.
  This has already been fixed in Bullet itself.
* Contributors: Dirk Thomas, Timo Rohling, Tully Foote

0.4.9 (2013-11-06)
------------------

0.4.8 (2013-11-06)
------------------
* moving python documentation to tf2_ros from tf2 to follow the code
* removing legacy rospy dependency. implementation removed in 0.4.0 fixes `#27 <https://github.com/ros/geometry_experimental/issues/27>`_

0.4.7 (2013-08-28)
------------------
* switching to use allFramesAsStringNoLock inside of getLatestCommonTime and walkToParent and locking in public API _getLatestCommonTime instead re `#23 <https://github.com/ros/geometry_experimental/issues/23>`_
* Fixes a crash in tf's view_frames related to dot code generation in allFramesAsDot

0.4.6 (2013-08-28)
------------------
* cleaner fix for `#19 <https://github.com/ros/geometry_experimental/issues/19>`_
* fix pointer initialization.  Fixes `#19 <https://github.com/ros/geometry_experimental/issues/19>`_
* fixes `#18 <https://github.com/ros/geometry_experimental/issues/18>`_ for hydro
* package.xml: corrected typo in description

0.4.5 (2013-07-11)
------------------
* adding _chainAsVector method for https://github.com/ros/geometry/issues/18
* adding _allFramesAsDot for backwards compatability https://github.com/ros/geometry/issues/18

0.4.4 (2013-07-09)
------------------
* making repo use CATKIN_ENABLE_TESTING correctly and switching rostest to be a test_depend with that change.
* tf2: Fixes a warning on OS X, but generally safer
  Replaces the use of pointers with shared_ptrs,
  this allows the polymorphism and makes it so that
  the compiler doesn't yell at us about calling
  delete on a class with a public non-virtual
  destructor.
* tf2: Fixes compiler warnings on OS X
  This exploited a gcc specific extension and is not
  C++ standard compliant. There used to be a "fix"
  for OS X which no longer applies. I think it is ok
  to use this as an int instead of a double, but
  another way to fix it would be to use a define.
* tf2: Fixes linkedit errors on OS X

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------
* adding getCacheLength() to parallel old tf API
* removing legacy static const variable MAX_EXTRAPOLATION_DISTANCE copied from tf unnecessesarily

0.4.1 (2013-07-05)
------------------
* adding old style callback notifications to BufferCore to enable backwards compatability of message filters
* exposing dedicated thread logic in BufferCore and checking in Buffer
* more methods to expose, and check for empty cache before getting latest timestamp
* adding methods to enable backwards compatability for passing through to tf::Transformer

0.4.0 (2013-06-27)
------------------
* splitting rospy dependency into tf2_py so tf2 is pure c++ library.
* switching to console_bridge from rosconsole
* moving convert methods back into tf2 because it does not have any ros dependencies beyond ros::Time which is already a dependency of tf2
* Cleaning up unnecessary dependency on roscpp
* Cleaning up packaging of tf2 including:
  removing unused nodehandle
  fixing overmatch on search and replace
  cleaning up a few dependencies and linking
  removing old backup of package.xml
  making diff minimally different from tf version of library
* suppressing bullet LinearMath copy inside of tf2, so it will not collide, and should not be used externally.
* Restoring test packages and bullet packages.
  reverting 3570e8c42f9b394ecbfd9db076b920b41300ad55 to get back more of the packages previously implemented
  reverting 04cf29d1b58c660fdc999ab83563a5d4b76ab331 to fix `#7 <https://github.com/ros/geometry_experimental/issues/7>`_
* fixing includes in unit tests
* Make PythonLibs find_package python2 specific
  On systems with python 3 installed and default, find_package(PythonLibs) will find the python 3 paths and libraries. However, the c++ include structure seems to be different in python 3 and tf2 uses includes that are no longer present or deprecated.
  Until the includes are made to be python 3 compliant, we should specify that the version of python found must be python 2.

0.3.6 (2013-03-03)
------------------

0.3.5 (2013-02-15 14:46)
------------------------
* 0.3.4 -> 0.3.5

0.3.4 (2013-02-15 13:14)
------------------------
* 0.3.3 -> 0.3.4
* moving LinearMath includes to include/tf2

0.3.3 (2013-02-15 11:30)
------------------------
* 0.3.2 -> 0.3.3
* fixing include installation of tf2

0.3.2 (2013-02-15 00:42)
------------------------
* 0.3.1 -> 0.3.2
* fixed missing include export & tf2_ros dependecy

0.3.1 (2013-02-14)
------------------
* 0.3.0 -> 0.3.1
* fixing PYTHON installation directory

0.3.0 (2013-02-13)
------------------
* switching to version 0.3.0
* adding setup.py to tf2 package
* fixed tf2 exposing python functionality
* removed line that was killing tf2_ros.so
* fixing catkin message dependencies
* removing packages with missing deps
* adding missing package.xml
* adding missing package.xml
* adding missing package.xml
* catkinizing geometry-experimental
* removing bullet headers from use in header files
* removing bullet headers from use in header files
* merging my recent changes
* setting child_frame_id overlooked in revision 6a0eec022be0 which fixed failing tests
* allFramesAsString public and internal methods seperated.  Public method is locked, private method is not
* fixing another scoped lock
* fixing one scoped lock
* fixing test compilation
* merge
* Error message fix, ros-pkg5085
* Check if target equals to source before validation
* When target_frame == source_frame, just returns an identity transform.
* adding addition ros header includes for strictness
* Fixed optimized lookups with compound transforms
* Fixed problem in tf2 optimized branch. Quaternion multiplication order was incorrect
* fix compilation on 32-bit
* Josh fix: Final inverse transform composition (missed multiplying the sourcd->top vector by the target->top inverse orientation). b44877d2b054
* Josh change: fix first/last time case. 46bf33868e0d
* fix transform accumulation to parent
* fix parent lookup, now works on the real pr2's tree
* move the message filter to tf2_ros
* tf2::MessageFilter + tests.  Still need to change it around to pass in a callback queue, since we're being triggered directly from the tf2 buffer
* Don't add the request if the transform is already available.  Add some new tests
* working transformable callbacks with a simple (incomplete) test case
* first pass at a transformable callback api, not tested yet
* add interpolation cases
* fix getLatestCommonTime -- no longer returns the latest of any of the times
* Some more optimization -- allow findClosest to inline
* another minor speedup
* Minorly speed up canTransform by not requiring the full data lookup, and only looking up the parent
* Add explicit operator= so that we can see the time in it on a profile graph.  Also some minor cleanup
* minor cleanup
* add 3 more cases to the speed test
* Remove use of btTransform at all from transform accumulation, since the conversion to/from is unnecessary, expensive, and can introduce floating point error
* Don't use btTransform as an intermediate when accumulating transforms, as constructing them takes quite a bit of time
* Completely remove lookupLists().  canTransform() now uses the same walking code as lookupTransform().  Also fixed a bug in the static transform publisher test
* Genericise the walk-to-top-parent code in lookupTransform so that it will be able to be used by canTransform as well (minus the cost of actually computing the transform)
* remove id lookup that wasn't doing anything
* Some more optimization:
  * Reduce # of TransformStorage copies made in TimeCache::getData()
  * Remove use of lookupLists from getLatestCommonTime
* lookupTransform() no longer uses lookupLists unless it's called with Time(0).  Removes lots of object construction/destruction due to removal of pushing back on the lists
* Remove CompactFrameID in favor of a typedef
* these mode checks are no longer necessary
* Fix crash when testing extrapolation on the forward transforms
* Update cache unit tests to work with the changes TransformStorage.
  Also make sure that BT_USE_DOUBLE_PRECISION is set for tf2.
* remove exposure of time_cache.h from buffer_core.h
* Removed the mutex from TimeCache, as it's unnecessary (BufferCore needs to have its own mutex locked anyway), and this speeds things up by about 20%
  Also fixed a number of thread-safety problems
* Optimize test_extrapolation a bit, 25% speedup of lookupTransform
* use a hash map for looking up frame numbers, speeds up lookupTransform by ~8%
* Cache vectors used for looking up transforms.  Speeds up lookupTransform by another 10%
* speed up lookupTransform by another 25%
* speed up lookupTransform by another 2x.  also reduces the memory footprint of the cache significantly
* sped up lookupTransform by another 2x
* First add of a simple speed test
  Sped up lookupTransform 2x
* roscpp dependency explicit, instead of relying on implicit
* static transform tested and working
* tests passing and all throw catches removed too\!
* validating frame_ids up front for lookup exceptions
* working with single base class vector
* tests passing for static storage
* making method private for clarity
* static cache implementation and test
* cleaning up API doc typos
* sphinx docs for Buffer
* new dox mainpage
* update tf2 manifest
* commenting out twist
* Changed cache_time to cache_time to follow C++ style guide, also initialized it to actually get things to work
* no more rand in cache tests
* Changing tf2_py.cpp to use underscores instead of camelCase
* removing all old converter functions from transform_datatypes.h
* removing last references to transform_datatypes.h in tf2
* transform conversions internalized
* removing unused datatypes
* copying bullet transform headers into tf2 and breaking bullet dependency
* merge
* removing dependency on tf
* removing include of old tf from tf2
* update doc
* merge
* kdl unittest passing
* Spaces instead of tabs in YAML grrrr
* Adding quotes for parent
* canTransform advanced ported
* Hopefully fixing YAML syntax
* new version of view_frames in new tf2_tools package
* testing new argument validation and catching bug
* Python support for debugging
* merge
* adding validation of frame_ids in queries with warnings and exceptions where appropriate
* Exposing ability to get frames as a string
* A compiling version of YAML debugging interface for BufferCore
* placeholder for tf debug
* fixing tf:: to tf2:: ns issues and stripping slashes on set in tf2 for backwards compatiabily
* Adding a python version of the BufferClient
* moving test to new package
* merging
* working unit test for BufferCore::lookupTransform
* removing unused method test and converting NO_PARENT test to new API
* Adding some comments
* Moving the python bindings for tf2 to the tf2 package from the tf2_py package
* buffercore tests upgraded
* porting tf_unittest while running incrmentally instead of block copy
* BufferCore::clear ported forward
* successfully changed lookupTransform advanced to new version
* switching to new implementation of lookupTransform tests still passing
* compiling lookupTransform new version
* removing tf_prefix from BufferCore.  BuferCore is independent of any frame_ids.  tf_prefix should be implemented at the ROS API level.
* initializing tf_prefix
* adding missing initialization
* suppressing warnings
* more tests ported
* removing tests for apis not ported forward
* setTransform tests ported
* old tests in new package passing due to backwards dependency.  now for the fun, port all 1500 lines :-)
* setTransform working in new framework as well as old
* porting more methods
* more compatability
* bringing in helper functions for buffer_core from tf.h/cpp
* rethrowing to new exceptions
* converting Storage to geometry_msgs::TransformStamped
* removing deprecated useage
* cleaning up includes
* moving all implementations into cpp file
* switching test to new class from old one
* Compiling version of the buffer client
* moving listener to tf_cpp
* removing listener, it should be in another package
* most of listener
* add cantransform implementation
* removing deprecated API usage
* initial import of listener header
* move implementation into library
* 2 tests of buffer
* moving executables back into bin
* compiling again with new design
* rename tfcore to buffercore
* almost compiling version of template code
* compiling tf2_core simple test
* add test to start compiling
* copying in tf_unittest for tf_core testing template
* prototype of tf2_core implemented using old tf.
* first version of template functions
* remove timeouts
* properly naming tf2_core.h from tf_core.h
* working cache test with tf2 lib
* first unit test passing, not yet ported
* tf_core api
* tf2 v2
* aborting port
* moving across time cache tf and datatypes headers
* copying exceptions from tf
* switching to tf2 from tf_core
