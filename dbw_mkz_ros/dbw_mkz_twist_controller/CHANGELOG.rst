^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dbw_mkz_twist_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.6 (2019-11-11)
------------------

1.2.5 (2019-10-30)
------------------

1.2.4 (2019-09-13)
------------------

1.2.3 (2019-08-13)
------------------

1.2.2 (2019-07-24)
------------------

1.2.1 (2019-07-11)
------------------

1.2.0 (2019-05-03)
------------------

1.1.2 (2019-03-14)
------------------

1.1.1 (2019-03-01)
------------------

1.1.0 (2018-11-30)
------------------
* Deprecated the dbw_mkz_twist_controller package
* Use the ${catkin_EXPORTED_TARGETS} macro for target dependencies
* Contributors: Kevin Hallenbeck

1.0.17 (2018-10-27)
-------------------

1.0.16 (2018-08-29)
-------------------

1.0.15 (2018-08-21)
-------------------

1.0.14 (2018-08-20)
-------------------

1.0.13 (2018-06-06)
-------------------
* Fixed compile error on ROS Melodic and Ubuntu Bionic
* Contributors: Kevin Hallenbeck

1.0.12 (2018-01-30)
-------------------
* Modifications to improve yaw rate tracking at low speeds
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.11 (2017-10-19)
-------------------

1.0.10 (2017-10-03)
-------------------

1.0.9 (2017-09-19)
------------------

1.0.8 (2017-09-07)
------------------

1.0.7 (2017-08-21)
------------------
* Updated ackermann steering parameters (including steering ratio)
* Contributors: Kevin Hallenbeck

1.0.6 (2017-06-21)
------------------
* Removed yaw rate control on speed, this also disables the lateral acceleration limit for now
* Updated wheel radius with measured value
* Contributors: Kevin Hallenbeck

1.0.5 (2017-04-25)
------------------
* Updated package.xml format to version 2
* Add missing msg dependencies
* Contributors: Kevin Hallenbeck, P. J. Reed

1.0.4 (2016-12-06)
------------------

1.0.3 (2016-11-17)
------------------

1.0.2 (2016-11-07)
------------------
* Twist tester option for yaw rate or radius for steering
* Cleanup of twist controller dynamic reconfigure
* Removed tf dependency for dbw_mkz_twist_controller
* Configurable steering ratio
* Contributors: Kevin Hallenbeck <khallenbeck@dataspeedinc.com>

1.0.1 (2016-10-10)
------------------

1.0.0 (2016-09-28)
------------------
* Initial release
* Contributors: Kevin Hallenbeck, Micho Radovnikovich
