^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.36.0 (2024-02-07)
-------------------
* Enable intra-process (`#649 <https://github.com/ros2/geometry2/issues/649>`_) (`#642 <https://github.com/ros2/geometry2/issues/642>`_)
* Contributors: Patrick Roncagliolo

0.35.1 (2024-01-24)
-------------------

0.35.0 (2023-12-26)
-------------------
* Avoid unecessary time conversions. (`#635 <https://github.com/ros2/geometry2/issues/635>`_)
* Contributors: AiVerisimilitude

0.34.0 (2023-11-06)
-------------------
* Expose TF2 listener CB (`#632 <https://github.com/ros2/geometry2/issues/632>`_)
* Contributors: Steve Macenski

0.33.2 (2023-10-04)
-------------------

0.33.1 (2023-09-07)
-------------------

0.33.0 (2023-08-21)
-------------------
* Fix invalid timer handle exception (`#474 <https://github.com/ros2/geometry2/issues/474>`_)
* Fix for `#589 <https://github.com/ros2/geometry2/issues/589>`_ - Should be able to transform with default timeout (`#593 <https://github.com/ros2/geometry2/issues/593>`_)
* Contributors: Cliff Wu, vineet131

0.32.2 (2023-07-11)
-------------------
* Enable StaticTransformBroadcaster in Intra-process enabled components (`#607 <https://github.com/ros2/geometry2/issues/607>`_)
* Contributors: Patrick Roncagliolo

0.32.1 (2023-05-11)
-------------------

0.32.0 (2023-04-27)
-------------------

0.31.2 (2023-04-13)
-------------------
* Destroy callback group before node (`#595 <https://github.com/ros2/geometry2/issues/595>`_)
* Contributors: Michael Carroll

0.31.1 (2023-04-12)
-------------------

0.31.0 (2023-04-11)
-------------------
* Enable TransformListener node-based constructor in Intra-process enabled components (`#572 <https://github.com/ros2/geometry2/issues/572>`_)
* Contributors: Patrick Roncagliolo

0.30.0 (2023-02-14)
-------------------
* Fix use-after-free bug in BufferServer::cancelCB (`#579 <https://github.com/ros2/geometry2/issues/579>`_)
* Update the demos to C++17. (`#578 <https://github.com/ros2/geometry2/issues/578>`_)
* add constructor to static tf broadcaster accepting node interfaces (`#576 <https://github.com/ros2/geometry2/issues/576>`_)
* Contributors: Alberto Soragna, Alexander Hans, Chris Lalancette

0.29.0 (2022-11-21)
-------------------
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`_)
* Contributors: Audrow Nash

0.28.0 (2022-11-02)
-------------------
* Switching from sstream to c string formatting to fix ros arg issue (`#557 <https://github.com/ros2/geometry2/issues/557>`_)
* allow construction of tf broadcaster from node object (not a pointer) (`#555 <https://github.com/ros2/geometry2/issues/555>`_)
* Allow to construct `TransformBroadcaster` and `TransformListener` from node interfaces (`#552 <https://github.com/ros2/geometry2/issues/552>`_)
* Contributors: Alberto Soragna, Chris Lalancette

0.27.0 (2022-09-13)
-------------------
* Suppress spam from calling canTransform (`#529 <https://github.com/ros2/geometry2/issues/529>`_)
* Contributors: Gonzo

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
* use dedicated callback group and executor to isolate timer (`#447 <https://github.com/ros2/geometry2/issues/447>`_)
* Adding shared pointer definition to tf2 buffer (`#508 <https://github.com/ros2/geometry2/issues/508>`_)
* fix for a basic logic (`#510 <https://github.com/ros2/geometry2/issues/510>`_)
* Fix precision loss from using rclcpp::Time::seconds() (`#511 <https://github.com/ros2/geometry2/issues/511>`_)
* Contributors: Chen Lihui, Kenji Brameld, Steve Macenski, Zhenpeng Ge

0.22.0 (2022-03-01)
-------------------
* clear relative callback of Buffer if MessageFilter is destroyed (`#490 <https://github.com/ros2/geometry2/issues/490>`_)
* More info in tf2_echo output (`#468 <https://github.com/ros2/geometry2/issues/468>`_)
* Contributors: Chen Lihui, simulacrus

0.21.0 (2022-01-14)
-------------------
* Fix cpplint errors (`#497 <https://github.com/ros2/geometry2/issues/497>`_)
* Contributors: Jacob Perron

0.20.0 (2021-12-17)
-------------------
* Fixes for uncrustify 0.72 (`#486 <https://github.com/ros2/geometry2/issues/486>`_)
* Contributors: Chris Lalancette

0.19.0 (2021-10-15)
-------------------
* More Intuitive CLI for Static Transform Publisher (`#392 <https://github.com/ros2/geometry2/issues/392>`_)
* Reduce transform listener nodes (`#442 <https://github.com/ros2/geometry2/issues/442>`_)
* `tf2_ros`: Fix deprecated subscriber callbacks (`#448 <https://github.com/ros2/geometry2/issues/448>`_)
* Contributors: Abrar Rahman Protyasha, Hunter L. Allen, gezp

0.18.0 (2021-06-01)
-------------------
* Fix tf2_echo does not work with ros-args (`#407 <https://github.com/ros2/geometry2/issues/407>`_) (`#408 <https://github.com/ros2/geometry2/issues/408>`_)
* Contributors: PGotzmann

0.17.1 (2021-04-06)
-------------------

0.17.0 (2021-03-19)
-------------------
* Guard against access to null node pointer (`#393 <https://github.com/ros2/geometry2/issues/393>`_)
* Allow to reconfigure durability for /tf topic broadcaster/listener (`#383 <https://github.com/ros2/geometry2/issues/383>`_)
* Fix the rcl type used in the time jump. (`#391 <https://github.com/ros2/geometry2/issues/391>`_)
* Fix linter errors (`#385 <https://github.com/ros2/geometry2/issues/385>`_)
* fix accessing freed resources (`#386 <https://github.com/ros2/geometry2/issues/386>`_)
* Allow reconfiguring qos of tf and tf_static topics through parameters (`#381 <https://github.com/ros2/geometry2/issues/381>`_)
* Replace ROS\_* logging macros and use RCLCPP\_* instead (`#380 <https://github.com/ros2/geometry2/issues/380>`_)
* Contributors: Audrow Nash, Chris Lalancette, Ivan Santiago Paunovic, Jacob Perron, Kazunari Tanaka

0.16.0 (2021-01-25)
-------------------
* Improve message filters error messages (`#364 <https://github.com/ros2/geometry2/issues/364>`_)
* Contributors: Ivan Santiago Paunovic

0.15.1 (2020-12-08)
-------------------
* Clarify the role of child_frame_id and header.frame_id in the documentation. (`#345 <https://github.com/ros2/geometry2/issues/345>`_)
* Contributors: Vikas Dhiman

0.15.0 (2020-11-02)
-------------------
* Remove usage of deprecated rclcpp::Duration constructor (`#340 <https://github.com/ros2/geometry2/issues/340>`_)
* Remove messages_count member from tf2_ros::MessageFilter. (`#335 <https://github.com/ros2/geometry2/issues/335>`_)
* Style fixup in tf2_ros. (`#325 <https://github.com/ros2/geometry2/issues/325>`_)
* Update maintainers of the ros2/geometry2 fork. (`#328 <https://github.com/ros2/geometry2/issues/328>`_)
* Contributors: Chris Lalancette, Ivan Santiago Paunovic

0.14.1 (2020-09-21)
-------------------
* Update goal response callback signature (`#323 <https://github.com/ros2/geometry2/issues/323>`_)
* Activate usual compiler warnings and fix errors (`#270 <https://github.com/ros2/geometry2/issues/270>`_)
* Contributors: Ivan Santiago Paunovic, Jacob Perron

0.14.0 (2020-08-14)
-------------------
* Fixed memory leak in Buffer::waitForTransform (`#281 <https://github.com/ros2/geometry2/issues/281>`_)
* fix time-reset test with Connext (`#306 <https://github.com/ros2/geometry2/issues/306>`_)
* reenable FrameGraph server (`#198 <https://github.com/ros2/geometry2/issues/198>`_)
* Use the usual style of parameters for static_transform_program (`#300 <https://github.com/ros2/geometry2/issues/300>`_)
* Make static_transform_broadcaster consistent with its command line description (`#294 <https://github.com/ros2/geometry2/issues/294>`_)
* Avoid using invalid std::list iterators (`#293 <https://github.com/ros2/geometry2/issues/293>`_)
* Generate callbacks after updating message\_ (`#274 <https://github.com/ros2/geometry2/issues/274>`_)
* Moved unique_lock of messages_mutex\_ to guarantee pointer (`#279 <https://github.com/ros2/geometry2/issues/279>`_)
* Fix dependencies in tf2_ros. (`#269 <https://github.com/ros2/geometry2/issues/269>`_)
* Split tf2_ros in tf2_ros and tf2_ros_py (`#210 <https://github.com/ros2/geometry2/issues/210>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette, Dirk Thomas, Hunter L. Allen, Martin Ganeff, Michael Carroll, ymd-stella

0.13.4 (2020-06-03)
-------------------
* export targets in addition to include directories / libraries (`#271 <https://github.com/ros2/geometry2/issues/271>`_)
* Add missing virtual destructors (`#272 <https://github.com/ros2/geometry2/issues/272>`_)
* Contributors: Dirk Thomas, Ivan Santiago Paunovic

0.13.3 (2020-05-26)
-------------------
* Fixed in Exception constructor tf2_ros (`#267 <https://github.com/ros2/geometry2/issues/267>`_)
* Contributors: Alejandro Hernández Cordero

0.13.2 (2020-05-18)
-------------------

0.13.1 (2020-05-08)
-------------------

0.13.0 (2020-04-30)
-------------------
* Added doxyfiles and sphinx Makefiles (`#257 <https://github.com/ros2/geometry2/issues/257>`_)
* avoid more deprecations (`#255 <https://github.com/ros2/geometry2/issues/255>`_)
* create_timer takes shared pointers (`#251 <https://github.com/ros2/geometry2/issues/251>`_)
* Improve tf2_echo and tf2_monitor messages while waiting for data (`#254 <https://github.com/ros2/geometry2/issues/254>`_)
* Add missing visibility header include (`#246 <https://github.com/ros2/geometry2/issues/246>`_)
* Fix `-Wrange-loop-construct` (`#245 <https://github.com/ros2/geometry2/issues/245>`_)
  ```
  --- stderr: tf2_ros
  /opt/ros/master/src/ros2/geometry2/tf2_ros/test/test_buffer.cpp:84:21: warning: loop variable 'elem' creates a copy from type 'const std::pair<const unsigned long, std::function<void (const unsigned long &)> >' [-Wrange-loop-construct]
  for (const auto elem : timer_to_callback_map\_) {
  ^
  /opt/ros/master/src/ros2/geometry2/tf2_ros/test/test_buffer.cpp:84:10: note: use reference type 'const std::pair<const unsigned long, std::function<void (const unsigned long &)> > &' to prevent copying
  for (const auto elem : timer_to_callback_map\_) {
  ^~~~~~~~~~~~~~~~~
  &
  1 warning generated.
  ---
  ```
* Remove TODO (`#234 <https://github.com/ros2/geometry2/issues/234>`_)
  The TODO is done; The publisher is using QoS durability setting 'transient local' which is the closest thing to the 'latched' concept in ROS 1.
  Signed-off-by: Jacob Perron <jacob@openrobotics.org>
* Remove virtual keyword from overridden functions (`#214 <https://github.com/ros2/geometry2/issues/214>`_)
  Signed-off-by: Hunter L. Allen <hunterlallen@protonmail.com>
* message filter fix (`#216 <https://github.com/ros2/geometry2/issues/216>`_)
  * Fixed meesage_filter add method
  * removed using builtin_interfaces::msg::Time in tf2_ros
* Porting more tests to tf2_ros (`#202 <https://github.com/ros2/geometry2/issues/202>`_)
  * Added more tests to tf2_ros
  * improving tf2_ros time_reset_test
  * tf2_ros fixed failed test_buffer_client.cpp
  * added some EXPECT to listener unittest
  * reviews
  * Update listener_unittest.cpp
  * fixed tf2_ros time_reset_test
  * tf2_ros removed ROS launch files
  * Added TODO to fix test_buffer_client in CI
  * tf2_ros added feedback
* Add static transform component (`#182 <https://github.com/ros2/geometry2/issues/182>`_)
  * Create a static transform component for composition
  Signed-off-by: Hunter L. Allen <hunterlallen@protonmail.com>
  * Suffix node name with randomly generated alpha-numeric string
  Signed-off-by: Hunter L. Allen <hunterlallen@protonmail.com>
  * Fix windows build
  Signed-off-by: Hunter L. Allen <hunterlallen@protonmail.com>
  * Switch to much more readable and more performant implementation by @clalancette
  Signed-off-by: Hunter L. Allen <hunterlallen@protonmail.com>
* Adding support for view_frame (`#192 <https://github.com/ros2/geometry2/issues/192>`_)
  * Adding tf2_tools support for view_frames
  * Changelog
  * tf2_tools 0.12.1 package version
  * tf2_tools common linters
  * tf2_tools changelog Forthcoming
  * tf2_tools log error and destroy client and node when a exception raised
  * tf2_tools 0.12.4 package version
  * tf2_tools revert some changes
  * tf2_tools - reduce changes
  * tf2_tools: finally block and passing the time instead of the node
  * tf2_tools: buffer with less arguments
  * tf2_tools: Fix condition
* Contributors: Alejandro Hernández Cordero, Dan Rose, Hunter L. Allen, Jacob Perron, Karsten Knese, Shane Loretz, William Woodall

0.12.4 (2019-11-19)
-------------------

0.12.3 (2019-11-18)
-------------------
* Remove unused setup.py files (`#190 <https://github.com/ros2/geometry2/issues/190>`_)
* Print out the name of the signalFailure reason instead of just its enum value (`#186 <https://github.com/ros2/geometry2/issues/186>`_)
* Contributors: Emerson Knapp, Vasilii Artemev

0.12.2 (2019-11-18)
-------------------
* Fix tf2_monitor subscriptions QoS settings. (`#196 <https://github.com/ros2/geometry2/issues/196>`_)
* Contributors: Michel Hidalgo

0.12.1 (2019-10-23)
-------------------
* Add convenience methods using rclcpp time types (`#180 <https://github.com/ros2/geometry2/issues/180>`_)
* Don't assume quaternions init to all zeros
* Make BufferClient destructor virtual
* Contributors: Josh Langsfeld, Shane Loretz, Thomas Moulard

0.12.0 (2019-09-26)
-------------------
* Simulate work in the acceptedCallback.
* Make Windows Debug to run the correct python executable.
* Make BufferInterface destructor virtual.
* Remove unnecessary and blacklisted actionlib_msgs dependency.
* More test fixes for tf2_ros python.
* class Clock is in clock not timer.
* tf2_ros is not built for Python (`#99 <https://github.com/ros2/geometry2/issues/99>`_)
* Migrate buffer action server to ROS 2
* Add conversion functions for durations
* Make /tf_static use transient_local durability (`#160 <https://github.com/ros2/geometry2/issues/160>`_)
* Force explicit --ros-args in NodeOptions::arguments(). (`#162 <https://github.com/ros2/geometry2/issues/162>`_)
* Use of -r/--remap flags where appropriate. (`#159 <https://github.com/ros2/geometry2/issues/159>`_)
* Include tf2 headers in message_filter.h (`#157 <https://github.com/ros2/geometry2/issues/157>`_)
* Use ament_target_dependencies to ensure correct dependency order (`#156 <https://github.com/ros2/geometry2/issues/156>`_)
* Make sure that TransformListener's node gets a unique name (`#129 <https://github.com/ros2/geometry2/issues/129>`_)
* Fix compiler warning (`#148 <https://github.com/ros2/geometry2/issues/148>`_)
* Do not timeout when waiting for transforms (`#146 <https://github.com/ros2/geometry2/issues/146>`_)
* Fix race between timeout and transform ready callback (`#143 <https://github.com/ros2/geometry2/issues/143>`_)
* Fix high CPU - Use executor to spin and stop node in tf_listener thread (`#119 <https://github.com/ros2/geometry2/issues/119>`_)
* Catch polymorphic exceptions by reference (`#138 <https://github.com/ros2/geometry2/issues/138>`_)
* Add missing export build dependencies (`#135 <https://github.com/ros2/geometry2/issues/135>`_)
* avoid delete-non-virtual-dtor warning (`#134 <https://github.com/ros2/geometry2/issues/134>`_)
* Template tf2_ros::MessageFilter on the buffer type
* Add pure virtual interface tf2_ros::AsyncBufferInterface
* Add pure virtual interface tf2_ros::CreateTimerInterface
* Allow tf2_monitor to be run with ROS command line args (`#122 <https://github.com/ros2/geometry2/issues/122>`_)
* Drop misleading ROS\_* logging macros from tf2_monitor (`#123 <https://github.com/ros2/geometry2/issues/123>`_)
* Fix the MessageFilter init order. (`#120 <https://github.com/ros2/geometry2/issues/120>`_)
* Contributors: Chris Lalancette, Dan Rose, Jacob Perron, Karsten Knese, Michel Hidalgo, Scott K Logan, Shane Loretz, Vinnam Kim, bpwilcox, evshary

0.11.3 (2019-05-24)
-------------------
* stop spinning TransformListener thread node in destructor (`#114 <https://github.com/ros2/geometry2/issues/114>`_)
* Store dedicated transform listener thread as a std::unique_ptr (`#111 <https://github.com/ros2/geometry2/issues/111>`_)
* enable pedantic for tf2_ros (`#115 <https://github.com/ros2/geometry2/issues/115>`_)
* Contributors: Hunter L. Allen, Karsten Knese, bpwilcox

0.11.2 (2019-05-20)
-------------------
* Remove stray semicolon which causes compiler error when using -Werror=pedantic (`#112 <https://github.com/ros2/geometry2/issues/112>`_)
* Contributors: Michael Jeronimo

0.11.1 (2019-05-09)
-------------------
* use node interfaces throughout tf2_ros (`#108 <https://github.com/ros2/geometry2/issues/108>`_)
* changes to avoid deprecated API's (`#107 <https://github.com/ros2/geometry2/issues/107>`_)
* Fix call to create_publisher after API changed (`#105 <https://github.com/ros2/geometry2/issues/105>`_)
* Use node interfaces for static transform broadcaster (`#104 <https://github.com/ros2/geometry2/issues/104>`_)
* Contributors: Emerson Knapp, Karsten Knese, William Woodall

0.11.0 (2019-04-14)
-------------------
* Updated to use node inteface pointers in the MessageFilter class. (`#96 <https://github.com/ros2/geometry2/pull/96>`_)
* Updated message_filter.h. (`#91 <https://github.com/ros2/geometry2/issues/91>`_)
* Contributors: Michael Jeronimo, Sagnik Basu

0.10.1 (2018-12-06)
-------------------
* Allow static_transform_publisher to be run with ros arguments ros2`#80 <https://github.com/ros2/geometry2/issues/80>`_ (`#82 <https://github.com/ros2/geometry2/issues/82>`_)
* Contributors: Lucas Walter

0.10.0 (2018-11-22)
-------------------
* Port tf2 ros message filter with ros2 tf2 and message filters (`#81 <https://github.com/ros2/geometry2/issues/81>`_)
  * Port tf2 message filter to ros2
  - remove APIs to node callback queue due to no callback queue
  in ros2 now
  - Change failure callback register with failure prompting due to
  no corresponding boost signal2 in C++11 and later
  - Fix expected transform count in case of time tolerance
  - Upgrade all message counts to 64 bitThis should resolve C4267 warnings about downgrading a size_t.
* Export tf2 dependency from tf2_ros (`#72 <https://github.com/ros2/geometry2/issues/72>`_)
* rclcpp time jump callback signature (`#69 <https://github.com/ros2/geometry2/issues/69>`_)
* Use ros2 time (`#67 <https://github.com/ros2/geometry2/issues/67>`_)
* Contributors: Carl Delsey, Ethan Gao, Shane Loretz

0.5.15 (2017-01-24)
-------------------
* tf2_ros: add option to unregister TransformListener (`#201 <https://github.com/ros/geometry2/issues/201>`_)
* Contributors: Hans-Joachim Krauch

0.5.14 (2017-01-16)
-------------------
* Drop roslib.load_manifest (`#191 <https://github.com/ros/geometry2/issues/191>`_)
* Adds ability to load TF from the ROS parameter server.
* Code linting & reorganization
* Fix indexing beyond end of array
* added a static transform broadcaster in python
* lots more documentation
* remove BufferCore doc, add BufferClient/BufferServer doc for C++, add Buffer/BufferInterface Python documentation
* Better overview for Python
* Contributors: Eric Wieser, Felix Duvallet, Jackie Kay, Mikael Arguedas, Mike Purvis

0.5.13 (2016-03-04)
-------------------
* fix documentation warnings
* Adding tests to package
* Contributors: Laurent GEORGE, Vincent Rabaud

0.5.12 (2015-08-05)
-------------------
* remove annoying gcc warning
  This is because the roslog macro cannot have two arguments that are
  formatting strings: we need to concatenate them first.
* break canTransform loop only for non-tiny negative time deltas
  (At least) with Python 2 ros.Time.now() is not necessarily monotonic
  and one can experience negative time deltas (usually well below 1s)
  on real hardware under full load. This check was originally introduced
  to allow for backjumps with rosbag replays, and only there it makes sense.
  So we'll add a small duration threshold to ignore backjumps due to
  non-monotonic clocks.
* Contributors: Vincent Rabaud, v4hn

0.5.11 (2015-04-22)
-------------------
* do not short circuit waitForTransform timeout when running inside pytf. Fixes `#102 <https://github.com/ros/geometry_experimental/issues/102>`_
  roscpp is not initialized inside pytf which means that ros::ok is not
  valid. This was causing the timer to abort immediately.
  This breaks support for pytf with respect to early breaking out of a loop re `#26 <https://github.com/ros/geometry_experimental/issues/26>`_.
  This is conceptually broken in pytf, and is fixed in tf2_ros python implementation.
  If you want this behavior I recommend switching to the tf2 python bindings.
* inject timeout information into error string for canTransform with timeout
* Contributors: Tully Foote

0.5.10 (2015-04-21)
-------------------
* switch to use a shared lock with upgrade instead of only a unique lock. For `#91 <https://github.com/ros/geometry_experimental/issues/91>`__
* Update message_filter.h
* filters: fix unsupported old messages with frame_id starting with '/'
* Enabled tf2 documentation
* make sure the messages get processed before testing the effects. Fixes `#88 <https://github.com/ros/geometry_experimental/issues/88>`_
* allowing to use message filters with PCL types
* Contributors: Brice Rebsamen, Jackie Kay, Tully Foote, Vincent Rabaud, jmtatsch

0.5.9 (2015-03-25)
------------------
* changed queue_size in Python transform boradcaster to match that in c++
* Contributors: mrath

0.5.8 (2015-03-17)
------------------
* fix deadlock `#79 <https://github.com/ros/geometry_experimental/issues/79>`_
* break out of loop if ros is shutdown. Fixes `#26 <https://github.com/ros/geometry_experimental/issues/26>`_
* remove useless Makefile files
* Fix static broadcaster with rpy args
* Contributors: Paul Bovbel, Tully Foote, Vincent Rabaud

0.5.7 (2014-12-23)
------------------
* Added 6 param transform again
  Yes, using Euler angles is a bad habit. But it is much more convenient if you just need a rotation by 90° somewhere to set it up in Euler angles. So I added the option to supply only the 3 angles.
* Remove tf2_py dependency for Android
* Contributors: Achim Königs, Gary Servin

0.5.6 (2014-09-18)
------------------
* support if canTransform(...): in python `#57 <https://github.com/ros/geometry_experimental/issues/57>`_
* Support clearing the cache when time jumps backwards `#68 <https://github.com/ros/geometry_experimental/issues/68>`_
* Contributors: Tully Foote

0.5.5 (2014-06-23)
------------------

0.5.4 (2014-05-07)
------------------
* surpressing autostart on the server objects to not incur warnings
* switch to boost signals2 following `ros/ros_comm#267 <https://github.com/ros/ros_comm/issues/267>`_, blocking `ros/geometry#23 <https://github.com/ros/geometry/issues/23>`_
* fix compilation with gcc 4.9
* make can_transform correctly wait
* explicitly set the publish queue size for rospy
* Contributors: Tully Foote, Vincent Rabaud, v4hn

0.5.3 (2014-02-21)
------------------

0.5.2 (2014-02-20)
------------------

0.5.1 (2014-02-14)
------------------
* adding const to MessageEvent data
* Contributors: Tully Foote

0.5.0 (2014-02-14)
------------------
* TF2 uses message events to get connection header information
* Contributors: Kevin Watts

0.4.10 (2013-12-26)
-------------------
* adding support for static transforms in python listener. Fixes `#46 <https://github.com/ros/geometry_experimental/issues/46>`_
* Contributors: Tully Foote

0.4.9 (2013-11-06)
------------------

0.4.8 (2013-11-06)
------------------
* fixing pytf failing to sleep https://github.com/ros/geometry/issues/30
* moving python documentation to tf2_ros from tf2 to follow the code
* Fixed static_transform_publisher duplicate check, added rostest.

0.4.7 (2013-08-28)
------------------
* fixing new conditional to cover the case that time has not progressed yet port forward of `ros/geometry#35 <https://github.com/ros/geometry/issues/35>`_ in the python implementation
* fixing new conditional to cover the case that time has not progressed yet port forward of `ros/geometry#35 <https://github.com/ros/geometry/issues/35>`_

0.4.6 (2013-08-28)
------------------
* patching python implementation for `#24 <https://github.com/ros/geometry_experimental/issues/24>`_ as well
* Stop waiting if time jumps backwards.  fixes `#24 <https://github.com/ros/geometry_experimental/issues/24>`_
* patch to work around uninitiaized time. `#30 <https://github.com/ros/geometry/issues/30>`_
* Removing unnecessary CATKIN_DEPENDS  `#18 <https://github.com/ros/geometry_experimental/issues/18>`_

0.4.5 (2013-07-11)
------------------
* Revert "cherrypicking groovy patch for `#10 <https://github.com/ros/geometry_experimental/issues/10>`_ into hydro"
  This reverts commit 296d4916706d64f719b8c1592ab60d3686f994e1.
  It was not starting up correctly.
* fixing usage string to show quaternions and using quaternions in the test app
* cherrypicking groovy patch for `#10 <https://github.com/ros/geometry_experimental/issues/10>`_ into hydro

0.4.4 (2013-07-09)
------------------
* making repo use CATKIN_ENABLE_TESTING correctly and switching rostest to be a test_depend with that change.
* reviving unrun unittest and adding CATKIN_ENABLE_TESTING guards

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------
* adding queue accessors lost in the new API
* exposing dedicated thread logic in BufferCore and checking in Buffer
* adding methods to enable backwards compatability for passing through to tf::Transformer

0.4.0 (2013-06-27)
------------------
* splitting rospy dependency into tf2_py so tf2 is pure c++ library.
* moving convert methods back into tf2 because it does not have any ros dependencies beyond ros::Time which is already a dependency of tf2
* Cleaning up unnecessary dependency on roscpp
* converting contents of tf2_ros to be properly namespaced in the tf2_ros namespace
* fixing return by value for tranform method without preallocatoin
* Cleaning up packaging of tf2 including:
  removing unused nodehandle
  cleaning up a few dependencies and linking
  removing old backup of package.xml
  making diff minimally different from tf version of library
* Restoring test packages and bullet packages.
  reverting 3570e8c42f9b394ecbfd9db076b920b41300ad55 to get back more of the packages previously implemented
  reverting 04cf29d1b58c660fdc999ab83563a5d4b76ab331 to fix `#7 <https://github.com/ros/geometry_experimental/issues/7>`_
* Added link against catkin_LIBRARIES for tf2_ros lib, also CMakeLists.txt clean up

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
* Merge pull request `#2 <https://github.com/ros/geometry_experimental/issues/2>`_ from KaijenHsiao/groovy-devel
  added setup.py and catkin_python_setup() to tf2_ros
* added setup.py and catkin_python_setup() to tf2_ros
* fixing cmake target collisions
* fixing catkin message dependencies
* removing packages with missing deps
* catkin fixes
* catkinizing geometry-experimental
* catkinizing tf2_ros
* catching None result in buffer client before it becomes an AttributeError, raising tf2.TransformException instead
* oneiric linker fixes, bump version to 0.2.3
* fix deprecated use of Header
* merged faust's changes 864 and 865 into non_optimized branch: BufferCore instead of Buffer in TransformListener, and added a constructor that takes a NodeHandle.
* add buffer server binary
* fix compilation on 32bit
* add missing file
* build buffer server
* TransformListener only needs a BufferCore
* Add TransformListener constructor that takes a NodeHandle so you can specify a callback queue to use
* Add option to use a callback queue in the message filter
* move the message filter to tf2_ros
* add missing std_msgs dependency
* missed 2 lines in last commit
* removing auto clearing from listener for it's unexpected from a library
* static transform tested and working
* subscriptions to tf_static unshelved
* static transform publisher executable running
* latching static transform publisher
* cleaning out old commented code
* Only query rospy.Time.now() when the timeout is greater than 0
* debug comments removed
* move to tf2_ros completed. tests pass again
* merge tf2_cpp and tf2_py into tf2_ros
