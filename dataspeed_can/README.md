# Dataspeed CAN

[dataspeed_can_usb](dataspeed_can_usb): Interface with up to four CAN buses with Dataspeed USB CAN Tool  
[dataspeed_can_msg_filters](dataspeed_can_msg_filters): Time synchronize multiple CAN messages to get a single callback  

# Example

```bash
roslaunch dataspeed_can_usb example.launch
```

# Parameters

Global parameters:

* ```error_topic```: Separate topic for CAN error frames. Default: ```True```
* ```mac_addr```: Connect to specific USB device by MAC address. Default: ```""``` for disabled
* ```bitrate```: CAN bitrate in bit/s up to 1Mbit/s (if not set by channel). Default: ```0``` for disabled
* ```mode```: CAN mode (normal/listen-only). Default: ```""``` for normal

CAN channel parameters:

* ```bitrate_X```: CAN bitrate in bit/s up to 1Mbit/s, 0 for disabled
* ```channel_X_mask_Y```: Up to 32 mask/match filter pairs to filter received CAN messages
* ```channel_X_match_Y```: Up to 32 mask/match filter pairs to filter received CAN messages

# Troubleshooting

* ```[WARN]: Dataspeed USB CAN Tool: not found```
    * Is the device plugged in to a USB port and powered with 12V?
        * The power LED should blink green every two seconds.
    * Can Linux see the USB device?
        * ```lsusb | grep 6923:0112```
    * Only one executable can use a hardware device at a time.
        * Make sure no other running software is using the hardware.
    * By default, Linux USB devices have very limited permissions. Udev rules are used to modify device permissions when connected.
        * When installing with binaries with [apt-get](https://bitbucket.org/DataspeedInc/ros_binaries), the udev rules are installed automatically.
        * Check if this package's udev rules are installed: ```ls /etc/udev/rules.d/ | grep 90-DataspeedUsbCanToolRules.rules``` or ```ls /lib/udev/rules.d/ | grep dataspeed-can-usb.rules```
        * Otherwise, follow the directions in [90-DataspeedUsbCanToolRules.rules](dataspeed_can_usb/udev/90-DataspeedUsbCanToolRules.rules) to perform a manual install.
* Why is the power LED blinking green?
    * The power LED blinks green every two seconds under normal operation.

# Migration

In September 2017, the ROS message type was migrated to [can_msgs/Frame](http://docs.ros.org/api/can_msgs/html/msg/Frame.html). See [can_msgs_migration](can_msgs_migration.md) for more details.

