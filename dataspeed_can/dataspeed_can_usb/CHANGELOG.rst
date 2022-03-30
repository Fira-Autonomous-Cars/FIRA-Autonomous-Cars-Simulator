^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dataspeed_can_usb
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.16 (2020-07-07)
-------------------
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
* Contributors: Kevin Hallenbeck

1.0.15 (2020-01-24)
-------------------

1.0.14 (2020-01-16)
-------------------

1.0.13 (2019-01-08)
-------------------
* Added advance compiler error checking and fixed newly exposed errors
* Contributors: Kevin Hallenbeck, Lincoln Lorenz

1.0.12 (2018-12-19)
-------------------
* Added tcpNoDelay() for subscribers
* Added firmware version publisher
* Contributors: Kevin Hallenbeck, Lincoln Lorenz

1.0.11 (2018-09-06)
-------------------
* Added option to connect to a specific USB device by MAC address
* Added normal/listen-only mode options
* Only subscribe to can_tx in normal mode, listen-only mode cannot transmit
* Updated module firmware to 10.4.0
* Contributors: Kevin Hallenbeck

1.0.10 (2018-06-13)
-------------------

1.0.9 (2018-06-12)
------------------
* Removed the private dataspeed_boot_usb package as an exec dependency
* Contributors: Kevin Hallenbeck

1.0.8 (2018-04-09)
------------------
* Detect old firmware versions and recommend update script
* Added script to update module firmware
* Updated module firmware to 10.3.0
* Added build dependency on roslib, needed for the ROS_DISTRO environment variable
* Use the simple filename 'udev' when installing rules, instead of the debian package name
* Added roslaunch test
* Contributors: Kevin Hallenbeck

1.0.7 (2017-10-19)
------------------
* Added missing roslaunch dependency
* Added support for CAN error frames
* Contributors: Kevin Hallenbeck

1.0.6 (2017-09-07)
------------------
* Only warn about dropped messages when the drop count changes
* Migrated from dataspeed_can_msgs to can_msgs (this will break dependent packages)
* Contributors: Kevin Hallenbeck

1.0.5 (2017-08-21)
------------------
* Prioritize the local include folder (there were issues with catkin workspace overlays)
* Updated nodelets to the PLUGINLIB_EXPORT_CLASS macro
* Contributors: Kevin Hallenbeck

1.0.4 (2017-05-08)
------------------
* Priority fix for udev on Indigo
* Contributors: Kevin Hallenbeck

1.0.3 (2017-04-25)
------------------
* Install udev rules with dh_installudev
* Updated package.xml format to version 2
* Fixed copy paste error in example launch file
* Contributors: Kevin Hallenbeck, Lincoln Lorenz

1.0.2 (2017-01-24)
------------------
* Throttle the 'not found' warning to every 10 seconds
* Install headers for use by other packages
* Option to specify USB device in constructor, option to specify textual name
* Contributors: Kevin Hallenbeck

1.0.1 (2016-11-17)
------------------
* Added example launch file
* Contributors: Kevin Hallenbeck

1.0.0 (2016-09-28)
------------------
* Initial release
* Contributors: Kevin Hallenbeck, Michael Lohrer
