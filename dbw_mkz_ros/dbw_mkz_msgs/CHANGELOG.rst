^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dbw_mkz_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2021-05-12)
------------------

1.3.2 (2021-03-09)
------------------

1.3.1 (2021-01-14)
------------------
* Add hazard light enumeration to turn signal status
* Contributors: Kevin Hallenbeck

1.3.0 (2020-11-18)
------------------
* Add brake pedal actuator brake type
* Add battery current measurement
* Contributors: Kevin Hallenbeck, Sun Hwang

1.2.11 (2020-08-17)
-------------------

1.2.10 (2020-08-05)
-------------------

1.2.9 (2020-07-09)
------------------
* Add gear reject enumerations
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
* Contributors: Kevin Hallenbeck

1.2.8 (2020-02-20)
------------------

1.2.7 (2020-02-14)
------------------

1.2.6 (2019-11-11)
------------------

1.2.5 (2019-10-30)
------------------
* Add steering wheel buttons in Misc1Report
* Contributors: Kevin Hallenbeck, Sun Hwang

1.2.4 (2019-09-13)
------------------

1.2.3 (2019-08-13)
------------------

1.2.2 (2019-07-24)
------------------

1.2.1 (2019-07-11)
------------------
* Added support for non-hybrid brake report values
* Added gear number to throttle info message
* Added throttle and brake limp-home statuses
* Contributors: Kevin Hallenbeck, Sun Hwang

1.2.0 (2019-05-03)
------------------
* Added angle/torque steering command modes (not supported on all platforms)
* Added odometer and battery voltage to fuel level report
* Contributors: Kevin Hallenbeck

1.1.2 (2019-03-14)
------------------

1.1.1 (2019-03-01)
------------------

1.1.0 (2018-11-30)
------------------
* Removed boo_cmd from BrakeCommand message
* Fixed old bag migration rule
* Added CMD_DECEL brake command type (only for non-hybrid platforms)
* Added DriverAssistReport message
* Contributors: Kevin Hallenbeck

1.0.17 (2018-10-27)
-------------------
* Added outside air temperature to Misc1Report
* Fixed copy-paste mistake in old bag migration rule
* Contributors: Kevin Hallenbeck

1.0.16 (2018-08-29)
-------------------

1.0.15 (2018-08-21)
-------------------

1.0.14 (2018-08-20)
-------------------
* Added brake pedal CMD_TORQUE_RQ command type for closed loop brake torque request
* Contributors: Kevin Hallenbeck

1.0.13 (2018-06-06)
-------------------
* Added support for the RES+ and RES- buttons
* Contributors: Kevin Hallenbeck

1.0.12 (2018-01-30)
-------------------
* Added power fault bit to report when modules lose power
* Added bag migration rules for converting bag files with old message types
* Contributors: Kevin Hallenbeck

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
* Removed steering report driver activity bit
* Replaced connector fault with timeout, and warn on timeout
* Added gear rejection enumeration to gear report
* Added wheel positions report (replaces suspension report)
* Added steering wheel left D-Pad buttons
* Contributors: Kevin Hallenbeck

1.0.6 (2017-06-21)
------------------
* Removed SuspensionReport (data was unintelligible)
* Added clear bit to command messages
* Contributors: Kevin Hallenbeck

1.0.5 (2017-04-25)
------------------
* Updated package.xml format to version 2
* Contributors: Kevin Hallenbeck

1.0.4 (2016-12-06)
------------------

1.0.3 (2016-11-17)
------------------
* Added QUIET bit to disable driver override audible warning
* Contributors: Kevin Hallenbeck

1.0.2 (2016-11-07)
------------------

1.0.1 (2016-10-10)
------------------

1.0.0 (2016-09-28)
------------------
* Initial release
* Contributors: Kevin Hallenbeck
