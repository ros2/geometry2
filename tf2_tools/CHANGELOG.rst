^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.17.1 (2021-04-06)
-------------------

0.17.0 (2021-03-19)
-------------------
* Add wait time option to view_frames (`#374 <https://github.com/ros2/geometry2/issues/374>`_)
* Contributors: Jacob Perron

0.16.0 (2021-01-25)
-------------------

0.15.1 (2020-12-08)
-------------------
* Cleanup tf2_tools to be more modern. (`#351 <https://github.com/ros2/geometry2/issues/351>`_)
* Contributors: Chris Lalancette

0.15.0 (2020-11-02)
-------------------
* Update maintainers of the ros2/geometry2 fork. (`#328 <https://github.com/ros2/geometry2/issues/328>`_)
* Contributors: Chris Lalancette

0.14.1 (2020-09-21)
-------------------
* Address security bug in yaml loading (`#313 <https://github.com/ros2/geometry2/issues/313>`_)
* Contributors: Víctor Mayoral Vilches

0.14.0 (2020-08-14)
-------------------
* Split tf2_ros in tf2_ros and tf2_ros_py (`#210 <https://github.com/ros2/geometry2/issues/210>`_)
* Contributors: Alejandro Hernández Cordero

0.13.4 (2020-06-03)
-------------------

0.13.3 (2020-05-26)
-------------------

0.13.2 (2020-05-18)
-------------------

0.13.1 (2020-05-08)
-------------------

0.13.0 (2020-04-30)
-------------------
* Added doxyfiles and sphinx Makefiles (`#257 <https://github.com/ros2/geometry2/issues/257>`_)
* tf2_tools update the shebang line (`#226 <https://github.com/ros2/geometry2/issues/226>`_)
* Adding support for view_frame (`#192 <https://github.com/ros2/geometry2/issues/192>`_)
* Contributors: Alejandro Hernández Cordero

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------
* Remove old load_manifest from view_frames (`#182 <https://github.com/ros/geometry2/issues/182>`_)
* Contributors: Jochen Sprickerhof

0.5.13 (2016-03-04)
-------------------
* casted el to string in view_frames
* Contributors: g_gemignani

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
* remove useless Makefile files
* Contributors: Vincent Rabaud

0.5.7 (2014-12-23)
------------------

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
* updating install rule for view_frames.py fixes `#44 <https://github.com/ros/geometry_experimental/issues/44>`_

0.4.7 (2013-08-28)
------------------

0.4.6 (2013-08-28)
------------------

0.4.5 (2013-07-11)
------------------

0.4.4 (2013-07-09)
------------------

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------

0.4.0 (2013-06-27)
------------------
* splitting rospy dependency into tf2_py so tf2 is pure c++ library.
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
* removing packages with missing deps
* catkinizing geometry-experimental
* catkinizing tf2_tools
* strip out rx dependencies
* Some fixes to make things work with rxbag
* Threading ns list
* merge tf2_cpp and tf2_py into tf2_ros
* Now catching exceptions correctly with echo
* Working version of tf echo
* Making sure to clear details when switching frames
* Changing file format to tf
* First cut at loading, saving, and exporting support
* tf frame viewer is now an rxbag plugin
* Can now connect to any node in the system that has a tf2 buffer
* Now populates namespaces as well
* Now populates a frame list on the fly
* Got the GUI set up for a bunch of features, now just have to implement the backend of them
* Persistent service call to speed things up. Also, coloring on click
* Adding a first version of frame_viewer
* Adding xdot as a dep in prep for frame_viewer
* working view frames
* call new service
* new version of view_frames in new tf2_tools package
