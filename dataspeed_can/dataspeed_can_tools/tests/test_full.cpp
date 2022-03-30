/*********************************************************************
 * C++ unit test for dataspeed_lm/version.h
 *********************************************************************/

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <can_msgs/Frame.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>

// String stream
#include <iostream>
#include <fstream>
#include <string>

// File under test
#include "../src/DbcIterator.hpp"

// Duration constants for waits
const ros::WallDuration DUR_A = ros::WallDuration(3.0); // Expecting success
const ros::WallDuration DUR_B = ros::WallDuration(0.1); // Expecting timeout

////////
// https://github.com/ros/ros_comm/blob/ebd9e491e71947889eb81089306698775ab5d2a2/test/test_roscpp/test/src/subscribe_star.cpp#L123

template <typename MsgT>
class MsgHelper {
public:
  MsgHelper(std::string tag) : count_(0) {
    tag_ = tag;
  }

  void set(const MsgT& msg) { latest_ = msg; stamp_ = ros::Time::now(); }
  
  void cb(const boost::shared_ptr<MsgT const>& msg) {
    set(*msg);
    ++count_;
  }

  bool waitForMessage(ros::WallDuration timeout) const {
    const ros::WallTime start = ros::WallTime::now();
    while (true) {
      if (count_ > 0) {
        ROS_DEBUG("Received message (%s)", tag_.c_str());
        return true;
      }
      if ((ros::WallTime::now() - start) > timeout) {
        ROS_DEBUG("Message TIMED OUT (%s)", tag_.c_str());
        break;
      }
      ros::WallDuration(0.001).sleep();
    }
    return false;
  }

  void clear() {
    stamp_ = ros::Time();
    count_ = 0;
    latest_ = MsgT();
    // tag_ is an identifier for the MsgHelper; it stays the same.
  }

  MsgT getLatest() const {return latest_;}
  uint32_t getCount() const {return count_;}
  ros::Time getStamp() const {return stamp_;}
private:
  MsgT latest_;
  std::string tag_;
  uint32_t count_;
  ros::Time stamp_;
};


bool waitPublisher(const ros::Publisher &pub, ros::WallDuration timeout) {
  const ros::WallTime start = ros::WallTime::now();
  while (true) {
    if ((pub.getNumSubscribers() > 0)) {
      return true;
    }
    if ((ros::WallTime::now() - start) > timeout) {
      break;
    }
    ros::WallDuration(0.001).sleep();
  }
  return false;
}

bool waitSubscriber(const ros::Subscriber &sub, ros::WallDuration timeout) {
  const ros::WallTime start = ros::WallTime::now();
  while (true) {
    if ((sub.getNumPublishers() > 0)) {
      return true;
    }
    if ((ros::WallTime::now() - start) > timeout) {
      break;
    }
    ros::WallDuration(0.001).sleep();
  }
  return false;
}

// Check the values output by parsing valid signals
TEST(FULL, Basic)
{
  // Message receive helpers
  typedef std_msgs::Int8 Msg1;
  typedef std_msgs::Int8 Msg2;
  typedef std_msgs::Int8 Msg3;
  typedef std_msgs::Bool Msg4;
  MsgHelper<can_msgs::Frame> rx0("basictest");
  MsgHelper<Msg1> rx1("basictest/basicsignal1");
  MsgHelper<Msg2> rx2("basictest/basicsignal2");
  MsgHelper<Msg3> rx3("basictest/basicsignal3");
  MsgHelper<Msg4> rx4("basictest/basicsignal4");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("BasicTest", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("BasicTest/BasicSignal1", 10, &MsgHelper<Msg1>::cb, &rx1);
  ros::Subscriber sub2 = nh.subscribe("BasicTest/BasicSignal2", 10, &MsgHelper<Msg2>::cb, &rx2);
  ros::Subscriber sub3 = nh.subscribe("BasicTest/BasicSignal3", 10, &MsgHelper<Msg3>::cb, &rx3);
  ros::Subscriber sub4 = nh.subscribe("BasicTest/BasicSignal4", 10, &MsgHelper<Msg4>::cb, &rx4);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Message ID 161 0xA1
  // Signal 1: 23
  // Signal 2: 43 (Only 7 bits to accommodate +8 offset)
  // Signal 3: 53 (Only 7 bits to accommodate +18 offset)
  // Signal 4: 1
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 161;
  msg.data = {23, 43, 53, 1, 0, 0, 0, 0};

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub2, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub3, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub4, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear(); rx2.clear(); rx3.clear(); rx4.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());
  ASSERT_TRUE(rx2.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx2.getCount());
  ASSERT_TRUE(rx3.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx3.getCount());
  ASSERT_TRUE(rx4.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx4.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, 23);
  EXPECT_EQ(rx2.getLatest().data, 43 + 8);
  EXPECT_EQ(rx3.getLatest().data, 53 + 18);
  EXPECT_EQ(rx4.getLatest().data, true);
}

// Check the values output by parsing valid signals
TEST(FULL, IntelA)
{
  // Message receive helpers
  typedef std_msgs::Int8   Msg1;
  typedef std_msgs::UInt8  Msg2;
  typedef std_msgs::Int16  Msg3;
  typedef std_msgs::UInt16 Msg4;
  MsgHelper<can_msgs::Frame> rx0("inteltesta");
  MsgHelper<Msg1> rx1("inteltesta/intelsignal8");
  MsgHelper<Msg2> rx2("inteltesta/intelsignalU8");
  MsgHelper<Msg3> rx3("inteltesta/intelsignal16");
  MsgHelper<Msg4> rx4("inteltesta/intelsignalU16");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("IntelTestA", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("IntelTestA/IntelSignal8", 10, &MsgHelper<Msg1>::cb, &rx1);
  ros::Subscriber sub2 = nh.subscribe("IntelTestA/IntelSignalU8", 10, &MsgHelper<Msg2>::cb, &rx2);
  ros::Subscriber sub3 = nh.subscribe("IntelTestA/IntelSignal16", 10, &MsgHelper<Msg3>::cb, &rx3);
  ros::Subscriber sub4 = nh.subscribe("IntelTestA/IntelSignalU16", 10, &MsgHelper<Msg4>::cb, &rx4);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Message ID 177 0xB1
  // Signal 1: 11 8 bit
  // Signal 2: 22 unsigned 8 bit
  // Signal 3: 0xCDEF 16 bit
  // Signal 4: 0x89AB unsigned 16 bit
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 177;
  msg.data = {11, 22, 0xEF, 0xCD, 0xAB, 0x89, 0, 0};

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub2, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub3, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub4, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear(); rx2.clear(); rx3.clear(); rx4.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());
  ASSERT_TRUE(rx2.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx2.getCount());
  ASSERT_TRUE(rx3.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx3.getCount());
  ASSERT_TRUE(rx4.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx4.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, (Msg1::_data_type)11);
  EXPECT_EQ(rx2.getLatest().data, (Msg2::_data_type)22);
  EXPECT_EQ(rx3.getLatest().data, (Msg3::_data_type)0xCDEF);
  EXPECT_EQ(rx4.getLatest().data, (Msg4::_data_type)0x89AB);
}

// Check the values output by parsing valid signals
TEST(FULL, IntelB)
{
  // Message receive helpers
  typedef std_msgs::Int32  Msg1;
  typedef std_msgs::UInt32 Msg2;
  MsgHelper<can_msgs::Frame> rx0("inteltestb");
  MsgHelper<Msg1> rx1("inteltestb/intelsignal32");
  MsgHelper<Msg2> rx2("inteltestb/intelsignalU32");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("IntelTestB", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("IntelTestB/IntelSignal32", 10, &MsgHelper<Msg1>::cb, &rx1);
  ros::Subscriber sub2 = nh.subscribe("IntelTestB/IntelSignalU32", 10, &MsgHelper<Msg2>::cb, &rx2);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Message ID 178 0xB2
  // Signal 1: 0x89ABCDEF 32 bit
  // Signal 2: 0x01234567 unsigned 32 bit
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 178;
  msg.data = {0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01};

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub2, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear(); rx2.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());
  ASSERT_TRUE(rx2.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx2.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, (Msg1::_data_type)0x89ABCDEF);
  EXPECT_EQ(rx2.getLatest().data, (Msg2::_data_type)0x01234567);
}

// Check the values output by parsing valid signals
TEST(FULL, IntelC)
{
  // Message receive helpers
  typedef std_msgs::Int64 Msg1;
  MsgHelper<can_msgs::Frame> rx0("inteltestc");
  MsgHelper<Msg1> rx1("inteltestc/intelsignal64");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("IntelTestC", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("IntelTestC/IntelSignal64", 10, &MsgHelper<Msg1>::cb, &rx1);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Message ID 177 0xB3
  // Signal 1: 0x0123456789ABCDEF 64 bit
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 179;
  msg.data = {0xE0, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01};

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, (Msg1::_data_type)0x0123456789ABCDE0);
  ///@TODO: The last 4 bits are not handled correctly, we should test 0x0123456789ABCDEF
}

// Check the values output by parsing valid signals
TEST(FULL, IntelD)
{
  // Message receive helpers
  typedef std_msgs::UInt64 Msg1;
  MsgHelper<can_msgs::Frame> rx0("inteltestd");
  MsgHelper<Msg1> rx1("inteltestd/intelsignalU64");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("IntelTestD", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("IntelTestD/IntelSignalU64", 10, &MsgHelper<Msg1>::cb, &rx1);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Message ID 180 0xB4
  // Signal 1: 0x0123456789ABCDEF unsigned 64 bit
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 180;
  msg.data = {0xE0, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01};

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, (Msg1::_data_type)0x0123456789ABCDE0);
  ///@TODO: The last 4 bits are not handled correctly, we should test 0x0123456789ABCDEF
}

// Check the values output by parsing valid signals
TEST(FULL, AdvancedA)
{
  // Message receive helpers
  typedef std_msgs::Int32 Msg1;
  typedef std_msgs::Int32 Msg2;
  MsgHelper<can_msgs::Frame> rx0("advancedtesta");
  MsgHelper<Msg1> rx1("advancedtesta/advancedsignal1");
  MsgHelper<Msg2> rx2("advancedtesta/advancedsignal2");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("AdvancedTestA", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("AdvancedTestA/AdvancedSignal1", 10, &MsgHelper<Msg1>::cb, &rx1);
  ros::Subscriber sub2 = nh.subscribe("AdvancedTestA/AdvancedSignal2", 10, &MsgHelper<Msg2>::cb, &rx2);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Message ID 162 0xA2
  // Signal 1: 210 little endian (smallest first)
  // Signal 2: 153 big endian (largest first)
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 162;
  msg.data = {210, 0, 0, 0, 0, 0, 0, 153}; //little

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub2, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear(); rx2.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());
  ASSERT_TRUE(rx2.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx2.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, 210);
  EXPECT_EQ(rx2.getLatest().data, 153);
  ///@TODO: IEEE Float shouldn't generate an Int32
}

// Check the values output by parsing valid signals
TEST(FULL, AdvancedB)
{
  // Message receive helpers
  typedef std_msgs::Int64 Msg1;
  MsgHelper<can_msgs::Frame> rx0("advancedtestb");
  MsgHelper<Msg1> rx1("advancedtestb/advancedsignal3");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("AdvancedTestB", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("AdvancedTestB/AdvancedSignal3", 10, &MsgHelper<Msg1>::cb, &rx1);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Message ID 163 0xA3
  // Signal 1: 210 64 bit little endian (smallest first)
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 163;
  msg.data = {210, 0, 0, 0, 0, 0, 0, 0}; //little

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, 210);
  ///@TODO: IEEE Double shouldn't generate an Int64
}

// Check the values output by parsing valid signals
TEST(FULL, AdvancedC)
{
  // Message receive helpers
  typedef std_msgs::Int64 Msg1;
  MsgHelper<can_msgs::Frame> rx0("advancedtestc");
  MsgHelper<Msg1> rx1("advancedtestc/advancedsignal4");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("AdvancedTestC", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("AdvancedTestC/AdvancedSignal4", 10, &MsgHelper<Msg1>::cb, &rx1);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Message ID 164 0xA4
  // Signal 1: 210 64 bit big endian (largest first)
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 164;
  msg.data = {0, 0, 0, 0, 0, 0, 0, 210}; //little

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, 210);
  ///@TODO: IEEE Double shouldn't generate an Int64
}

// Check the values output by parsing valid signals
TEST(FULL, AdvancedD)
{
  // Message receive helpers
  typedef std_msgs::Int16  Msg1;
  typedef std_msgs::UInt16 Msg2;
  typedef std_msgs::Int16  Msg3;
  typedef std_msgs::Int16  Msg4; // Factor is negative.
  MsgHelper<can_msgs::Frame> rx0("advancedtestd");
  MsgHelper<Msg1> rx1("advancedtestd/advancedsignal5");
  MsgHelper<Msg2> rx2("advancedtestd/advancedsignal6");
  MsgHelper<Msg3> rx3("advancedtestd/advancedsignal7");
  MsgHelper<Msg4> rx4("advancedtestd/advancedsignal8");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("AdvancedTestD", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("AdvancedTestD/AdvancedSignal5", 10, &MsgHelper<Msg1>::cb, &rx1);
  ros::Subscriber sub2 = nh.subscribe("AdvancedTestD/AdvancedSignal6", 10, &MsgHelper<Msg2>::cb, &rx2);
  ros::Subscriber sub3 = nh.subscribe("AdvancedTestD/AdvancedSignal7", 10, &MsgHelper<Msg3>::cb, &rx3);
  ros::Subscriber sub4 = nh.subscribe("AdvancedTestD/AdvancedSignal8", 10, &MsgHelper<Msg4>::cb, &rx4);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Message ID 165 0xA5
  // Signal 1: 320 x10 factor signed
  // Signal 1: 640 x10 factor unsigned
  // Signal 1: -120 x-10 factor signed
  // Signal 1: -240 x-10 factor unsigned changed to signed
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 165;
  msg.data = {32, 64, 12, 24}; //little

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub2, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub3, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub4, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear(); rx2.clear(); rx3.clear(); rx4.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());
  ASSERT_TRUE(rx2.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx2.getCount());
  ASSERT_TRUE(rx3.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx3.getCount());
  ASSERT_TRUE(rx4.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx4.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, 320);
  EXPECT_EQ(rx2.getLatest().data, 640);
  EXPECT_EQ(rx3.getLatest().data, -120);
  EXPECT_EQ(rx4.getLatest().data, -240);
}

// Check the values output by parsing valid signals
TEST(FULL, MotorolaA)
{
  // Message receive helpers
  typedef std_msgs::Int8   Msg1;
  typedef std_msgs::UInt8  Msg2;
  typedef std_msgs::Int16  Msg3;
  typedef std_msgs::UInt16 Msg4;
  MsgHelper<can_msgs::Frame> rx0("motorolatesta");
  MsgHelper<Msg1> rx1("motorolatesta/motorolasignal8");
  MsgHelper<Msg2> rx2("motorolatesta/motorolasignalU8");
  MsgHelper<Msg3> rx3("motorolatesta/motorolasignal16");
  MsgHelper<Msg4> rx4("motorolatesta/motorolasignalU16");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("MotorolaTestA", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("MotorolaTestA/MotorolaSignal8", 10, &MsgHelper<Msg1>::cb, &rx1);
  ros::Subscriber sub2 = nh.subscribe("MotorolaTestA/MotorolaSignalU8", 10, &MsgHelper<Msg2>::cb, &rx2);
  ros::Subscriber sub3 = nh.subscribe("MotorolaTestA/MotorolaSignal16", 10, &MsgHelper<Msg3>::cb, &rx3);
  ros::Subscriber sub4 = nh.subscribe("MotorolaTestA/MotorolaSignalU16", 10, &MsgHelper<Msg4>::cb, &rx4);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Message ID 193 0xC1
  // Signal 1: 11 8 bit
  // Signal 2: 22 unsigned 8 bit
  // Signal 3: 0x89AB 16 bit
  // Signal 4: 0xCDEF unsigned 16 bit
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 193;
  msg.data = {11, 22, 0, 33, 0, 44, 0, 0};
  msg.data = {11, 22, 0x89, 0xAB, 0xCD, 0xEF, 0, 0};

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub2, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub3, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub4, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear(); rx2.clear(); rx3.clear(); rx4.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());
  ASSERT_TRUE(rx2.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx2.getCount());
  ASSERT_TRUE(rx3.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx3.getCount());
  ASSERT_TRUE(rx4.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx4.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, 11);
  EXPECT_EQ(rx2.getLatest().data, 22);
  EXPECT_EQ(rx3.getLatest().data, (Msg3::_data_type)0x89AB);
  EXPECT_EQ(rx4.getLatest().data, (Msg4::_data_type)0xCDEF);
}

// Check the values output by parsing valid signals
TEST(FULL, MotorolaB)
{
  // Message receive helpers
  typedef std_msgs::Int32  Msg1;
  typedef std_msgs::UInt32 Msg2;
  MsgHelper<can_msgs::Frame> rx0("motorolatestb");
  MsgHelper<Msg1> rx1("motorolatestb/motorolasignal32");
  MsgHelper<Msg2> rx2("motorolatestb/motorolasignalU32");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("MotorolaTestB", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("MotorolaTestB/MotorolaSignal32", 10, &MsgHelper<Msg1>::cb, &rx1);
  ros::Subscriber sub2 = nh.subscribe("MotorolaTestB/MotorolaSignalU32", 10, &MsgHelper<Msg2>::cb, &rx2);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Message ID 194 0xC2
  // Signal 1: 0x01234567 32 bit
  // Signal 2: 0x89ABCDEF unsigned 32 bit
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 194;
  msg.data = {0, 0, 0, 55, 0, 0, 0, 66};
  msg.data = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub2, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear(); rx2.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());
  ASSERT_TRUE(rx2.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx2.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, (Msg1::_data_type)0x01234567);
  EXPECT_EQ(rx2.getLatest().data, (Msg2::_data_type)0x89ABCDEF);
}

// Check the values output by parsing valid signals
TEST(FULL, MotorolaC)
{
  // Message receive helpers
  typedef std_msgs::Int64 Msg1;
  MsgHelper<can_msgs::Frame> rx0("motorolatestc");
  MsgHelper<Msg1> rx1("motorolatestc/motorolasignal64");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("MotorolaTestC", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("MotorolaTestC/MotorolaSignal64", 10, &MsgHelper<Msg1>::cb, &rx1);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  //Message ID 195 0xC3
  //Signal 1: 0x0123456789ABCDEF 64 bit
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 195;
  msg.data = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xE0};

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, (Msg1::_data_type)0x0123456789ABCDE0);
  ///@TODO: The last 4 bits are not handled correctly, we should test 0x0123456789ABCDEF
}

// Check the values output by parsing valid signals
TEST(FULL, MotorolaD)
{
  // Message receive helpers
  typedef std_msgs::UInt64 Msg1;
  MsgHelper<can_msgs::Frame> rx0("motorolatestd");
  MsgHelper<Msg1> rx1("motorolatestd/motorolasignalU64");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("MotorolaTestD", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("MotorolaTestD/MotorolaSignalU64", 10, &MsgHelper<Msg1>::cb, &rx1);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Message ID 196 0xC4
  // Signal 1: 0x0123456789ABCDEF unsigned 64 bit
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 196;
  msg.data = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xE0};

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear();

  // Publish the message again and wait for the response (now that the topics are connected)
  pub.publish(msg);
  ASSERT_TRUE(rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  ASSERT_TRUE(rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, (Msg1::_data_type)0x0123456789ABCDE0);
  ///@TODO: The last 4 bits are not handled correctly, we should test 0x0123456789ABCDEF
}

// Check the values output by parsing valid signals
TEST(FULL, Multiplex)
{
  // Message receive helpers
  typedef std_msgs::UInt8  Msg1;
  typedef std_msgs::Int8   Msg2;
  typedef std_msgs::Int8   Msg3;
  typedef std_msgs::UInt16 Msg4;
  typedef std_msgs::Int32  Msg5;
  MsgHelper<can_msgs::Frame> rx0("multiplextest");
  MsgHelper<Msg1> rx1("multiplextest/multiplexor");
  MsgHelper<Msg2> rx2("multiplextest/multiplexeda");
  MsgHelper<Msg3> rx3("multiplextest/multiplexedb");
  MsgHelper<Msg4> rx4("multiplextest/multiplexedc");
  MsgHelper<Msg5> rx5("multiplextest/multiplexedd");

  // Create ROS topics and wait for publisher connection
  ros::NodeHandle nh("can_bus_test");
  ros::Publisher pub = nh.advertise<can_msgs::Frame>("can_rx", 10);
  ros::Subscriber sub0 = nh.subscribe("MultiplexTest", 10, &MsgHelper<can_msgs::Frame>::cb, &rx0);
  ros::Subscriber sub1 = nh.subscribe("MultiplexTest/Multiplexor", 10, &MsgHelper<Msg1>::cb, &rx1);
  ros::Subscriber sub2 = nh.subscribe("MultiplexTest/MultiplexedA", 10, &MsgHelper<Msg2>::cb, &rx2);
  ros::Subscriber sub3 = nh.subscribe("MultiplexTest/MultiplexedB", 10, &MsgHelper<Msg3>::cb, &rx3);
  ros::Subscriber sub4 = nh.subscribe("MultiplexTest/MultiplexedC", 10, &MsgHelper<Msg4>::cb, &rx4);
  ros::Subscriber sub5 = nh.subscribe("MultiplexTest/MultiplexedD", 10, &MsgHelper<Msg5>::cb, &rx5);
  ASSERT_TRUE(waitPublisher(pub, DUR_A)); DUR_B.sleep();

  // Create a message to publish
  // Signal 1: 0 multiplexor
  can_msgs::Frame msg;
  msg.header.stamp = ros::Time::now();
  msg.is_rtr = false;
  msg.is_error = false;
  msg.is_extended = false;
  msg.id = 166;
  msg.data = {0, 0, 0, 0, 0, 0, 0, 0};

  // Publish the message and verify topic creation (but we might not get the data)
  pub.publish(msg);
  ASSERT_TRUE(waitSubscriber(sub0, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub1, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub2, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub3, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub4, DUR_A));
  ASSERT_TRUE(waitSubscriber(sub5, DUR_A));
  DUR_B.sleep(); rx0.clear(); rx1.clear(); rx2.clear(); rx3.clear(); rx4.clear(); rx5.clear();

  // Change multiplexor and data
  // Signal 1: 0x99 multiplexor
  msg.data = {0x99, 0, 0, 0, 0, 0, 0, 0};

  // Publish the new message and wait for the response
  pub.publish(msg);
  EXPECT_TRUE (rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  EXPECT_TRUE (rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());
  EXPECT_FALSE(rx2.waitForMessage(DUR_B)); EXPECT_EQ(0u, rx2.getCount());
  EXPECT_FALSE(rx3.waitForMessage(DUR_B)); EXPECT_EQ(0u, rx3.getCount());
  EXPECT_FALSE(rx4.waitForMessage(DUR_B)); EXPECT_EQ(0u, rx4.getCount());
  EXPECT_FALSE(rx5.waitForMessage(DUR_B)); EXPECT_EQ(0u, rx5.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, 0x99);
  DUR_B.sleep(); rx0.clear(); rx1.clear(); rx2.clear(); rx3.clear(); rx4.clear(); rx5.clear();

  // Change multiplexor and data
  // Signal 1: 0x11 multiplexor
  // Signal 2: 52 multiplexed a
  msg.data = {0x11, 52, 0, 0, 0, 0, 0, 0};

  // Publish the new message and wait for the response
  pub.publish(msg);
  EXPECT_TRUE (rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  EXPECT_TRUE (rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());
  EXPECT_TRUE (rx2.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx2.getCount());
  EXPECT_FALSE(rx3.waitForMessage(DUR_B)); EXPECT_EQ(0u, rx3.getCount());
  EXPECT_FALSE(rx4.waitForMessage(DUR_B)); EXPECT_EQ(0u, rx4.getCount());
  EXPECT_FALSE(rx5.waitForMessage(DUR_B)); EXPECT_EQ(0u, rx5.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, 0x11);
  EXPECT_EQ(rx2.getLatest().data, 52);
  DUR_B.sleep(); rx0.clear(); rx1.clear(); rx2.clear(); rx3.clear(); rx4.clear(); rx5.clear();

  // Change multiplexor and data
  // Signal 1: 0x22 multiplexor
  // Signal 2: 102 multiplexed b 8 bit
  // Signal 3: 103 multiplexed c 16 bit unsigned
  msg.data = {0x22, 102, 103, 0, 0, 0, 0, 0};

  // Publish the new message and wait for the response
  pub.publish(msg);
  EXPECT_TRUE (rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  EXPECT_TRUE (rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());
  EXPECT_FALSE(rx2.waitForMessage(DUR_B)); EXPECT_EQ(0u, rx2.getCount());
  EXPECT_TRUE (rx3.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx3.getCount());
  EXPECT_TRUE (rx4.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx4.getCount());
  EXPECT_FALSE(rx5.waitForMessage(DUR_B)); EXPECT_EQ(0u, rx5.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, 0x22);
  EXPECT_EQ(rx3.getLatest().data, 102);
  EXPECT_EQ(rx4.getLatest().data, 103);
  DUR_B.sleep(); rx0.clear(); rx1.clear(); rx2.clear(); rx3.clear(); rx4.clear(); rx5.clear();

  // Change multiplexor and data
  // Signal 1: 0x33 multiplexor
  // Signal 2: 0x12345678 multiplexed 32 bit
  msg.id = 166;
  msg.data = {0x33, 0x78, 0x56, 0x34, 0x12, 0, 0, 0};

  // Publish the new message and wait for the response
  pub.publish(msg);
  EXPECT_TRUE (rx0.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx0.getCount());
  EXPECT_TRUE (rx1.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx1.getCount());
  EXPECT_FALSE(rx2.waitForMessage(DUR_B)); EXPECT_EQ(0u, rx2.getCount());
  EXPECT_FALSE(rx3.waitForMessage(DUR_B)); EXPECT_EQ(0u, rx3.getCount());
  EXPECT_FALSE(rx4.waitForMessage(DUR_B)); EXPECT_EQ(0u, rx4.getCount());
  EXPECT_TRUE (rx5.waitForMessage(DUR_A)); EXPECT_EQ(1u, rx5.getCount());

  // Compare the response with the expected values
  EXPECT_EQ(rx0.getLatest().header.stamp, msg.header.stamp);
  EXPECT_EQ(rx0.getLatest().data, msg.data);
  EXPECT_EQ(rx1.getLatest().data, 0x33);
  EXPECT_EQ(rx5.getLatest().data, 0x12345678);
  DUR_B.sleep(); rx0.clear(); rx1.clear(); rx2.clear(); rx3.clear(); rx4.clear(); rx5.clear();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_full");

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
 
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
