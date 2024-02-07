^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.36.0 (2024-02-07)
-------------------

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
* Update geometry2 to C++17 (`#584 <https://github.com/ros2/geometry2/issues/584>`_)
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

0.24.0 (2022-03-31)
-------------------

0.23.0 (2022-03-28)
-------------------
* Make sure to finalize tf2_py BufferCore. (`#505 <https://github.com/ros2/geometry2/issues/505>`_)
* Contributors: Chris Lalancette

0.22.0 (2022-03-01)
-------------------

0.21.0 (2022-01-14)
-------------------
* Make tf2_py Use FindPython3 (`#494 <https://github.com/ros2/geometry2/issues/494>`_)
* Contributors: Shane Loretz

0.20.0 (2021-12-17)
-------------------

0.19.0 (2021-10-15)
-------------------
* Change TF2Error names to be a bit more descriptive. (`#349 <https://github.com/ros2/geometry2/issues/349>`_)
* Contributors: Chris Lalancette

0.18.0 (2021-06-01)
-------------------
* Remove python_compat.h (`#417 <https://github.com/ros2/geometry2/issues/417>`_)
* Contributors: Chris Lalancette

0.17.1 (2021-04-06)
-------------------

0.17.0 (2021-03-19)
-------------------

0.16.0 (2021-01-25)
-------------------
* Adapt to Python 3.9 (`#362 <https://github.com/ros2/geometry2/issues/362>`_)
* Contributors: Homalozoa X

0.15.1 (2020-12-08)
-------------------

0.15.0 (2020-11-02)
-------------------
* Update maintainers of the ros2/geometry2 fork. (`#328 <https://github.com/ros2/geometry2/issues/328>`_)
* Contributors: Chris Lalancette

0.14.1 (2020-09-21)
-------------------

0.14.0 (2020-08-14)
-------------------
* Add in pytest.ini so tests succeed locally. (`#280 <https://github.com/ros2/geometry2/issues/280>`_)
* Contributors: Chris Lalancette

0.13.4 (2020-06-03)
-------------------

0.13.3 (2020-05-26)
-------------------

0.13.2 (2020-05-18)
-------------------
* Explicitly add DLL directories for Windows before importing (`#266 <https://github.com/ros2/geometry2/issues/266>`_)
* Contributors: Jacob Perron

0.13.1 (2020-05-08)
-------------------

0.13.0 (2020-04-30)
-------------------
* Fix build error in Focal (`#241 <https://github.com/ros2/geometry2/issues/241>`_)
* Style cleanup on tf2_py.cpp (`#222 <https://github.com/ros2/geometry2/issues/222>`_)
* Contributors: Alejandro Hernández Cordero, Ivan Santiago Paunovic

0.12.4 (2019-11-19)
-------------------

0.12.3 (2019-11-18)
-------------------

0.12.2 (2019-11-18)
-------------------

0.12.1 (2019-10-23)
-------------------
* Quiet the cast-function-type warning on GCC 8.
* Contributors: Chris Lalancette

0.12.0 (2019-09-26)
-------------------
* Properly keep references to Python objects.
* Don't use borrowString in time or duration conversions.
* Minor fix to use CMake variable.
* tf2_ros is not built for Python (`#99 <https://github.com/ros2/geometry2/issues/99>`_)
* Contributors: Chris Lalancette, Vinnam Kim

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------
* Improve tf compatibility (`#192 <https://github.com/ros/geometry2/issues/192>`_)
  getLatestCommonTime() is needed to implement the TF API.
  See `ros/geometry#134 <https://github.com/ros/geometry/issues/134>`_
* Add missing type checks at Python/C++ tf2 transform interface `#159 <https://github.com/ros/geometry2/issues/159>`_ (`#197 <https://github.com/ros/geometry2/issues/197>`_)
* Make tf2_py compatible with python3. (`#173 <https://github.com/ros/geometry2/issues/173>`_)
  * tf2_py: Use PyUnicode objects for text in python3.
  * tf2_py: Make module initialization python3 compatible.
  * tf2_py: Fix type definition for python3.
  * tf2_py: Move and rename PyObject_BorrowAttrString.
* Contributors: Maarten de Vries, Timo Röhling, alex

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

0.5.1 (2014-02-14)
------------------

0.5.0 (2014-02-14)
------------------

0.4.10 (2013-12-26)
-------------------
* adding support for static transforms in python listener. Fixes `#46 <https://github.com/ros/geometry_experimental/issues/46>`_
* Contributors: Tully Foote

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
* tf2_py: Fixes warning, implicit conversion of NULL

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------

0.4.0 (2013-06-27)
------------------
* splitting rospy dependency into tf2_py so tf2 is pure c++ library.

