^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dataspeed_can_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.16 (2020-07-07)
-------------------
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
* Fix processing bags with DBC files for extended CAN IDs
* Contributors: Kevin Hallenbeck

1.0.15 (2020-01-24)
-------------------
* Use %zu for portable printf() size type
* Contributors: Kevin Hallenbeck

1.0.14 (2020-01-16)
-------------------

1.0.13 (2019-01-08)
-------------------
* Added advance compiler error checking and fixed newly exposed errors
* Contributors: Kevin Hallenbeck, Lincoln Lorenz

1.0.12 (2018-12-19)
-------------------

1.0.11 (2018-09-06)
-------------------
* Added support for multiple DBC files
* Added support for multiplexed signals
* Many changes to the command line arguments
* Fixed bug with signals greater than 31 bits
* Set CXX_STANDARD to C++11 when necessary
* Contributors: Kevin Hallenbeck, Eric Myllyoja

1.0.10 (2018-06-13)
-------------------

1.0.9 (2018-06-12)
------------------
* Preliminary support for extended IDs
* Contributors: Kevin Hallenbeck

1.0.8 (2018-04-09)
------------------
* Added timestamp
* Added option to expand dbc signals or not
* Handle messages not defined in the dbc file
* Contributors: Kevin Hallenbeck

1.0.7 (2017-10-19)
------------------

1.0.6 (2017-09-07)
------------------
* Migrated from dataspeed_can_msgs to can_msgs (this will break dependent packages)
* Contributors: Kevin Hallenbeck

1.0.5 (2017-08-21)
------------------
* Updated nodelets to the PLUGINLIB_EXPORT_CLASS macro
* Contributors: Kevin Hallenbeck

1.0.4 (2017-05-08)
------------------

1.0.3 (2017-04-25)
------------------
* Updated package.xml format to version 2
* Contributors: Kevin Hallenbeck

1.0.2 (2017-01-24)
------------------

1.0.1 (2016-11-17)
------------------

1.0.0 (2016-09-28)
------------------
* Initial release
* Contributors: Kevin Hallenbeck, Micho Radovnikovich
