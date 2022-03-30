# Migration to can_msgs

The messages in dataspeed_can_msgs have been removed and should be migrated to [can_msgs/Frame](http://docs.ros.org/api/can_msgs/html/msg/Frame.html).

* dataspeed_can_msgs/CanMessage -> [can_msgs/Frame](http://docs.ros.org/api/can_msgs/html/msg/Frame.html)
* dataspeed_can_msgs/CanMessageStamped -> [can_msgs/Frame](http://docs.ros.org/api/can_msgs/html/msg/Frame.html)

# Bag files

Bag [migration](http://wiki.ros.org/rosbag/migration) rules are included to assist converting bag files with old message types to the new message type:

* [CanMessageStamped](dataspeed_can_msgs/bmr/rule1.bmr)
* [CanMessage](dataspeed_can_msgs/bmr/rule2.bmr)

Convert an old bag with the following terminal command:

```
sudo apt-get install ros-$ROS_DISTRO-dataspeed-can-msgs
rosbag fix old.bag new.bag
```

# Source code

```c++
#include <dataspeed_can_msgs/CanMessageStamped.h>
void function_old() {
  dataspeed_can_msgs::CanMessage msg1;
  dataspeed_can_msgs::CanMessageStamped msg2;
  msg2.header.stamp = ros::Time::now();
  msg1.id = 0x123;
  msg2.msg.id = 0x123;
  msg1.dlc = 8;
  msg2.msg.dlc = 8;
  msg1.extended = false;
  msg2.msg.extended = false;
  msg1.data[0] = 0;
  msg2.msg.data[0] = 0;
}
```

```c++
#include <can_msgs/Frame.h>
void function_new() {
  can_msgs::Frame msg1, msg2;
  msg1.header.stamp = ros::Time::now();
  msg2.header.stamp = ros::Time::now();
  msg1.id = 0x123;
  msg2.id = 0x123;
  msg1.dlc = 8;
  msg2.dlc = 8;
  msg1.is_extended = false;
  msg2.is_extended = false;
  msg1.is_rtr = false;
  msg2.is_rtr = false;
  msg1.is_error = false;
  msg2.is_error = false;
  msg1.data[0] = 0;
  msg2.data[0] = 0;
}
```

