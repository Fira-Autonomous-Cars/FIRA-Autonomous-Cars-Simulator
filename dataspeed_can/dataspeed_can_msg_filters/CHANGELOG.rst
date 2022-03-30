^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dataspeed_can_msg_filters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.16 (2020-07-07)
-------------------
* Increase CMake minimum version to 3.0.2 to avoid warning about CMP0048
  http://wiki.ros.org/noetic/Migration#Increase_required_CMake_version_to_avoid_author_warning
* Contributors: Kevin Hallenbeck

1.0.15 (2020-01-24)
-------------------

1.0.14 (2020-01-16)
-------------------
* Make setting the intermessage lowerbound simpler
* Document Approximate Time synchronizer using ros wiki
* Contributors: Kevin Hallenbeck, Lincoln Lorenz

1.0.13 (2019-01-08)
-------------------
* Added advance compiler error checking and fixed newly exposed errors
* Contributors: Kevin Hallenbeck, Lincoln Lorenz

1.0.12 (2018-12-19)
-------------------

1.0.11 (2018-09-06)
-------------------

1.0.10 (2018-06-13)
-------------------
* Removed unnecessary rostest dependency
* Contributors: Kevin Hallenbeck

1.0.9 (2018-06-12)
------------------
* Added tests for extended IDs, error frames, and RTR frames
* Ported tests from ros_comm/message_filters/approximate_time_policy
* Distinguish between standard and extended IDs with the MSB of the ID
* Contributors: Kevin Hallenbeck

1.0.8 (2018-04-09)
------------------

1.0.7 (2017-10-19)
------------------

1.0.6 (2017-09-07)
------------------
* Migrated from dataspeed_can_msgs to can_msgs (this will break dependent packages)
* Contributors: Kevin Hallenbeck

1.0.5 (2017-08-21)
------------------

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
* Contributors: Kevin Hallenbeck
