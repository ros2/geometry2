^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_ros_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.36.0 (2024-02-07)
-------------------

0.35.1 (2024-01-24)
-------------------

0.35.0 (2023-12-26)
-------------------

0.34.0 (2023-11-06)
-------------------
* Make sure to cache transforms in tf2_ros_py. (`#634 <https://github.com/ros2/geometry2/issues/634>`_)
* Contributors: Chris Lalancette

0.33.2 (2023-10-04)
-------------------

0.33.1 (2023-09-07)
-------------------
* Remove 'efficient copy' prints (`#625 <https://github.com/ros2/geometry2/issues/625>`_)
* Contributors: Matthijs van der Burgh

0.33.0 (2023-08-21)
-------------------

0.32.2 (2023-07-11)
-------------------
* Add time jump callback (`#608 <https://github.com/ros2/geometry2/issues/608>`_)
* Contributors: Erich L Foster

0.32.1 (2023-05-11)
-------------------

0.32.0 (2023-04-27)
-------------------

0.31.2 (2023-04-13)
-------------------

0.31.1 (2023-04-12)
-------------------
* Update sys.path with wokring directory (`#594 <https://github.com/ros2/geometry2/issues/594>`_)
* Contributors: Yadu

0.31.0 (2023-04-11)
-------------------
* Enable document generation using rosdoc2 for ament_python pkgs (`#587 <https://github.com/ros2/geometry2/issues/587>`_)
* Contributors: Yadu

0.30.0 (2023-02-14)
-------------------

0.29.0 (2022-11-21)
-------------------
* Update maintainers (`#560 <https://github.com/ros2/geometry2/issues/560>`_)
* Use pytest rather than unittest to enable repeat (`#558 <https://github.com/ros2/geometry2/issues/558>`_)
* Contributors: Audrow Nash, Michael Carroll

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

0.24.0 (2022-03-31)
-------------------

0.23.0 (2022-03-28)
-------------------
* Drop PyKDL dependency in tf2_geometry_msgs (`#509 <https://github.com/ros2/geometry2/issues/509>`_)
* Contributors: Florian Vahl

0.22.0 (2022-03-01)
-------------------
* Add in one more destroy call that was missed in testing. (`#504 <https://github.com/ros2/geometry2/issues/504>`_)
* Contributors: Chris Lalancette

0.21.0 (2022-01-14)
-------------------
* Be much more careful about cleanup in the tf2_ros_py tests. (`#499 <https://github.com/ros2/geometry2/issues/499>`_)
* Use the correct type for BufferClient timeout_padding. (`#498 <https://github.com/ros2/geometry2/issues/498>`_)
  It should be a duration, not a float.
* Contributors: Chris Lalancette

0.20.0 (2021-12-17)
-------------------
* Update maintainers to Alejandro Hernandez Cordero and Chris Lalancette (`#481 <https://github.com/ros2/geometry2/issues/481>`_)
* Contributors: Audrow Nash

0.19.0 (2021-10-15)
-------------------
* Fix buffer_client.py using default timeout_padding (`#437 <https://github.com/ros2/geometry2/issues/437>`_)
* Contributors: Carlos Andrés Álvarez Restrepo

0.18.0 (2021-06-01)
-------------------
* Use underscores instead of dashes in setup.cfg. (`#403 <https://github.com/ros2/geometry2/issues/403>`_)
* Contributors: Chris Lalancette

0.17.1 (2021-04-06)
-------------------

0.17.0 (2021-03-19)
-------------------
* Use global namespace for TransformListener topics (`#390 <https://github.com/ros2/geometry2/issues/390>`_)
* Fix indentation of a comment in buffer.py (`#371 <https://github.com/ros2/geometry2/issues/371>`_)
* Contributors: Chris Lalancette, Jacob Perron

0.16.0 (2021-01-25)
-------------------

0.15.1 (2020-12-08)
-------------------

0.15.0 (2020-11-02)
-------------------
* Update rclpy.Rate TODO with url to issue (`#324 <https://github.com/ros2/geometry2/issues/324>`_)
* Update maintainers of the ros2/geometry2 fork. (`#328 <https://github.com/ros2/geometry2/issues/328>`_)
* Contributors: Chris Lalancette, surfertas

0.14.1 (2020-09-21)
-------------------
* Add deprecation warnings to lookup_transform to handle the passing of the incorrect Time object. (`#319 <https://github.com/ros2/geometry2/issues/319>`_)
* change signature to show true arguments (`#321 <https://github.com/ros2/geometry2/issues/321>`_)
* Handle when None passed to qos argument in the constructor of TransformBroadcaster. (`#320 <https://github.com/ros2/geometry2/issues/320>`_)
* Add type hints to tf2_ros_py code(`#275 <https://github.com/ros2/geometry2/issues/275>`_) (`#315 <https://github.com/ros2/geometry2/issues/315>`_)
* Contributors: surfertas

0.14.0 (2020-08-14)
-------------------
* Clear callbacks_to_remove variable after removing (`#303 <https://github.com/ros2/geometry2/issues/303>`_)
* Fix cache_time None check in buffer.py (`#297 <https://github.com/ros2/geometry2/issues/297>`_)
* Split tf2_ros in tf2_ros and tf2_ros_py (`#210 <https://github.com/ros2/geometry2/issues/210>`_)
* Contributors: Alejandro Hernández Cordero, Matthijs den Toom, ScottMcMichael

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

0.12.4 (2019-11-19)
-------------------

0.12.3 (2019-11-18 16:39)
-------------------------

0.12.2 (2019-11-18 22:25)
-------------------------

0.12.1 (2019-10-23)
-------------------

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

0.10.1 (2018-12-06)
-------------------

0.10.0 (2018-11-22 14:27)
-------------------------

0.9.2 (2018-11-22 13:46)
------------------------

0.9.1 (2018-06-27 15:46)
------------------------

0.9.0 (2018-06-27 10:07)
------------------------

0.8.0 (2017-12-08)
------------------

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------

0.5.13 (2016-03-04)
-------------------

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

0.5.1 (2014-02-14 14:57)
------------------------

0.5.0 (2014-02-14 13:36)
------------------------

0.4.10 (2013-12-26)
-------------------

0.4.9 (2013-11-06 16:21)
------------------------

0.4.8 (2013-11-06 14:32)
------------------------

0.4.7 (2013-08-28 18:21)
------------------------

0.4.6 (2013-08-28 01:06)
------------------------

0.4.5 (2013-07-11)
------------------

0.4.4 (2013-07-09)
------------------

0.4.3 (2013-07-05 19:14)
------------------------

0.4.2 (2013-07-05 19:09)
------------------------

0.4.1 (2013-07-05 11:22)
------------------------

0.4.0 (2013-06-27)
------------------

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
