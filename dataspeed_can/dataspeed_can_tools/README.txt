
===can_node===
* START: node.cpp
	* Create a CanDriver.cpp instance
* CanDriver.cpp
	* TO CHANGE: Add Sync Time support
	* Create a CanUsb.cpp instance
		* This creates a reference to a data stream which provides a callback whenever USB data is read from the CAN device.
	* Set the CanUsb callback to CanDriver.cpp::recvDevice
	* Get bitrate and error_topic information
		* Each channel is fetched individually and placed in a vector. bitrate[0] is modified by the launch parameter "bitrate_1", like so:
			<remap from="can_bus_1" to="can_bus_1"/>
    		<param name="bitrate_1" value="250000" /> <!-- 250kbit/s -->
    		<param name="channel_1_mask_0"  value="0x80000000" />
    		<param name="channel_1_match_0" value="0x80000000" /> 
    	* Same for filter_masks and matches.
    * Open the CAN driver (run CanUsb.cpp::open)
    * Read version number and channel count
    * For each channel, print the filter masks, and filter matches to the console
    * For each channel, attempt to set the bitrates, then print to console
    * If all channel bitrates are set successfully, set up ROS pub/subs:
    	* For each channel with nonzero bitrate, create a node (can_bus_#)
    		* Create a subscriber (can_tx) to receive messages to send to the device
    		* Create a publisher (can_rx) to send messages whenever the device sends data
    		* Create an error publisher (can_err) if argument is set
    * Whenever data is received from the device, publish a can_msgs::Frame to the appropriate channel
    	* can_msgs is a standard message type: http://docs.ros.org/kinetic/api/can_msgs/html/msg/Frame.html
    	Contains a unique ID, data length code (dlc), and data (up to 8 bytes)

===dbc_node===
* START: dbc_node.cpp
	* Create a Node and NodeHandle
	* Get dbc_file parameter; if it doesn't exist, exit.
	* Create a CanExtractor.cpp object, passing the dbc_file value
	* Create a subscriber to can_rx (which is published by can_node)
		* When a can_msgs::Frame message is received, get the ID (OR it with 0x80000000 if it's an extended msg) and put it in a RosCanMsgStruct
		* Run CanExtractor::getMessage(RosCanMsgStruct), which returns true/false
			* If true, run CanExtractor::initPublishers(RosCanMsgStruct, NodeHandle)
		* Run CanExtractor::pubMessage(can_msgs::Frame), which posts the data to the appropriate ROS topic 
* CanExtractor.cpp
	* dbc_
		* A DBCIterator.cpp object
	* msgs_
		* A map between CAN message IDs and RosCanMsgStructs
	* getMessage(RosCanMsgStruct)
		* Check if the RosCanMsgStruct is already in the msgs_ map.
		* If it is, set the RosCanMsgStruct reference to the stored one, and return false (the message type has already been processed).
		* If it isn't, iterate over dbc_, searching for the CAN ID.
			* For each matching DBCMessage::Message found, create a RosCanSigStruct based on it and append it to the RosCanMsgStruct
			* Then store the RosCanMsgStruct and return it
		* Either way, the result is a RosCanMsgStruct, containing a CAN ID, and message name.
		* RosCanMsgStruct contains a can_msgs::Frame publisher which sends whenever a message is received, and a vector of RosCanSigStructs, each containing a publisher which sends individual fields whenever a message is received.
	* initPublishers(RosCanMsgStruct, NodeHandle)
		* Creates a publisher for the message as a whole (can_msgs::Frame) and attaches it to RosCanMsgStruct
		* Creates publishers for each signal with registerCanSignalPublisher(RosCanSigStruct NodeHandle)
	* registerCanSignalPublisher(RosCanSigStruct NodeHandle)
		* Creates an std_msgs::Int/Float64/Bool publisher (signed/unsigned/size based on RosCanSigStruct)
		* Attaches the publisher to the RosCanSigStruct
	* pubMessage()
		* Retrieve the RosCanMsgStruct from the msgs_ map based on message ID.
			* If the messageID isn't in the msgs_ map, return.
		* An 8-byte integer (uint64) stores all message data.
		* TO CHANGE: Loop through the list of signals for a signal multiplexor enum value that is MULTIPLEXOR. Save this value and break if found, set to -1 otherwise.
		* For each signal, the data is retrieved based on the start bit and length (signed or unsigned).
		* TO CHANGE: For each signal, if the multiplexor is not -1, use an if conditional to check if the signal's multiplexor enum is MULTIPLEXED; if it is, check if the multiplexNum is equal to the multiplexor value from the loop above; only publish if the values match.
* DBCIterator.cpp
	* messageList: Vector of DBCMessage.Message objects
	* init
		* TO CHANGE: Support multiple file streams
		* Open the file stream
		* DBCMessage::Message overrides the >> operator, and uses this to take a stream (up to the next )
		* If the process fails and the stream errors out, clear it (which will cause the loop to exit)
		* Else, add the message to the messageList.
		* DBCIterator iterates the file's messages, DBCMessage iterates the message's signals
* DBCMessage.cpp
	* If the line does not start in BO_, throw an error to get the parent iterator to clear to the next line.
	* If it does, fetch the BO data:
		BO_ <ID> <Messagename> <ByteLength> <Sender>
	* Then, iterate over the next lines;
	* While the stream is creating valid signal objects, >> create more DbcSignal.cpp objects.
* DBCSignal.cpp
	* Get the next line, if (after cleanup) the line is empty or does not start with SG_, throw an error to get the parent iterator to finish parsing signals.
	* If it does, fetch the signal data:
		SG_ <Fieldname> <Multiplexor> : <StartBit>|<BitLength>@<Order><Signed> (<Factor>,<Offset>) [<Minimum>|<Maximum>] "<Unit>" <Receivers>
	* Multiplexor may be blank (meaning value always appears in the message), in which case there will be no value between the Fieldname and the :
	* If Multiplexor is not blank, it is either:
		* 'M', meaning the value is the multiplexor
		* 'm###', with numbers, meaning the signal only appears in the message when the multiplexor value is equal to that of the multiplexed signal
	* Byteorder is either Motorola (big-endian/network) (0) or Intel (little-endian/math) (1)
	* Signed is either + (unsigned value) or - (signed value)
	* phys = digits * factor + offset

===DBC EXAMPLE===
BO_ 126 License: 8 EPAS
 SG_ EXPIRED : 10|1@1+ (1,0) [0|0] ""  MAB
 SG_ MUX M : 0|8@1+ (1,0) [0|0] ""  MAB
 SG_ FEAT_BASE_TRIALS_USED m0 : 32|16@1+ (1,0) [0|0] ""  MAB
 SG_ DATE1 m129 : 24|8@1+ (1,0) [0|0] ""  MAB

 
 SG_ FEAT_BASE_TRIALS_REMAINING m0 : 48|16@1+ (1,0) [0|0] ""  MAB
 SG_ FEAT_BASE_TRIAL m0 : 17|1@1+ (1,0) [0|0] ""  MAB
 SG_ FEAT_BASE_ENABLED m0 : 16|1@1+ (1,0) [0|0] ""  MAB
 SG_ TRIAL : 9|1@1+ (1,0) [0|0] ""  MAB
 SG_ READY : 8|1@1+ (1,0) [0|0] ""  MAB
 SG_ DATE0 m129 : 16|8@1+ (1,0) [0|0] ""  MAB
 SG_ VIN16 m133 : 48|8@1+ (1,0) [0|0] ""  MAB
 SG_ VIN15 m133 : 40|8@1+ (1,0) [0|0] ""  MAB
 SG_ VIN14 m133 : 32|8@1+ (1,0) [0|0] ""  MAB
 SG_ VIN01 m131 : 24|8@1+ (1,0) [0|0] ""  MAB
 SG_ VIN00 m131 : 16|8@1+ (1,0) [0|0] ""  MAB