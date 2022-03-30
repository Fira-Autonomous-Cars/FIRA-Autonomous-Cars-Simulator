/*********************************************************************
 * C++ unit test for ApproximateTime.h
 *********************************************************************/

#include <gtest/gtest.h>

// File under test
#include <dataspeed_can_msg_filters/ApproximateTime.h>
using namespace dataspeed_can_msg_filters;

TEST(ApproxTimeSync, ValidId_PostMask)
{
  // Standard IDs
  ASSERT_TRUE (ApproximateTime::ValidId(0x000007FFu));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000001u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000002u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000004u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000008u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000010u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000020u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000040u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000080u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000010u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000020u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000040u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000080u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000100u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000200u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000400u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00000800u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00001000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00002000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00004000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00008000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00010000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00020000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00040000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00080000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00010000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00020000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00040000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00080000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00100000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00200000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00400000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00800000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x01000000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x02000000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x04000000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x08000000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x10000000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x20000000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0x40000000u));

  // Extended IDs
  ASSERT_TRUE (ApproximateTime::ValidId(0x9FFFFFFFu));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000001u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000002u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000004u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000008u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000010u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000020u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000040u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000080u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000010u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000020u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000040u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000080u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000100u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000200u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000400u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80000800u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80001000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80002000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80004000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80008000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80010000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80020000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80040000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80080000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80010000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80020000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80040000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80080000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80100000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80200000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80400000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x80800000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x81000000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x82000000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x84000000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x88000000u));
  ASSERT_TRUE (ApproximateTime::ValidId(0x90000000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0xA0000000u));
  ASSERT_FALSE(ApproximateTime::ValidId(0xC0000000u));
}

TEST(ApproxTimeSync, ValidId_PreMask)
{
  // Standard IDs
  ASSERT_TRUE (ApproximateTime::ValidId(0x000007FFu, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000000u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000001u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000002u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000004u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000008u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000010u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000020u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000040u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000080u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000010u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000020u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000040u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000080u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000100u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000200u, false));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000400u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00000800u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00001000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00002000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00004000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00008000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00010000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00020000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00040000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00080000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00010000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00020000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00040000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00080000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00100000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00200000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00400000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x00800000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x01000000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x02000000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x04000000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x08000000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x10000000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x20000000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x40000000u, false));
  ASSERT_FALSE(ApproximateTime::ValidId(0x80000000u, false));

  // Extended IDs
  ASSERT_TRUE (ApproximateTime::ValidId(0x1FFFFFFFu, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000001u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000002u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000004u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000008u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000010u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000020u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000040u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000080u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000010u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000020u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000040u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000080u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000100u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000200u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000400u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00000800u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00001000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00002000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00004000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00008000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00010000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00020000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00040000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00080000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00010000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00020000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00040000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00080000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00100000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00200000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00400000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x00800000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x01000000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x02000000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x04000000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x08000000u, true));
  ASSERT_TRUE (ApproximateTime::ValidId(0x10000000u, true));
  ASSERT_FALSE(ApproximateTime::ValidId(0x20000000u, true));
  ASSERT_FALSE(ApproximateTime::ValidId(0x40000000u, true));
  ASSERT_FALSE(ApproximateTime::ValidId(0x80000000u, true));
}

TEST(ApproxTimeSync, BuildId)
{
  // Standard IDs
  ASSERT_EQ(0x000007FFu, ApproximateTime::BuildId(0x000007FFu, false));
  ASSERT_EQ(0x000007FFu, ApproximateTime::BuildId(0xFFFFFFFFu, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00000000u, false));
  ASSERT_EQ(0x00000001u, ApproximateTime::BuildId(0x00000001u, false));
  ASSERT_EQ(0x00000002u, ApproximateTime::BuildId(0x00000002u, false));
  ASSERT_EQ(0x00000004u, ApproximateTime::BuildId(0x00000004u, false));
  ASSERT_EQ(0x00000008u, ApproximateTime::BuildId(0x00000008u, false));
  ASSERT_EQ(0x00000010u, ApproximateTime::BuildId(0x00000010u, false));
  ASSERT_EQ(0x00000020u, ApproximateTime::BuildId(0x00000020u, false));
  ASSERT_EQ(0x00000040u, ApproximateTime::BuildId(0x00000040u, false));
  ASSERT_EQ(0x00000080u, ApproximateTime::BuildId(0x00000080u, false));
  ASSERT_EQ(0x00000100u, ApproximateTime::BuildId(0x00000100u, false));
  ASSERT_EQ(0x00000200u, ApproximateTime::BuildId(0x00000200u, false));
  ASSERT_EQ(0x00000400u, ApproximateTime::BuildId(0x00000400u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00000800u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00001000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00002000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00004000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00008000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00010000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00020000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00040000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00080000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00100000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00200000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00400000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x00800000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x01000000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x02000000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x04000000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x08000000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x10000000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x20000000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x40000000u, false));
  ASSERT_EQ(0x00000000u, ApproximateTime::BuildId(0x80000000u, false));

  // Extended IDs
  ASSERT_EQ(0x9FFFFFFFu, ApproximateTime::BuildId(0x1FFFFFFFu, true));
  ASSERT_EQ(0x9FFFFFFFu, ApproximateTime::BuildId(0xFFFFFFFFu, true));
  ASSERT_EQ(0x80000000u, ApproximateTime::BuildId(0x00000000u, true));
  ASSERT_EQ(0x80000001u, ApproximateTime::BuildId(0x00000001u, true));
  ASSERT_EQ(0x80000002u, ApproximateTime::BuildId(0x00000002u, true));
  ASSERT_EQ(0x80000004u, ApproximateTime::BuildId(0x00000004u, true));
  ASSERT_EQ(0x80000008u, ApproximateTime::BuildId(0x00000008u, true));
  ASSERT_EQ(0x80000010u, ApproximateTime::BuildId(0x00000010u, true));
  ASSERT_EQ(0x80000020u, ApproximateTime::BuildId(0x00000020u, true));
  ASSERT_EQ(0x80000040u, ApproximateTime::BuildId(0x00000040u, true));
  ASSERT_EQ(0x80000080u, ApproximateTime::BuildId(0x00000080u, true));
  ASSERT_EQ(0x80000100u, ApproximateTime::BuildId(0x00000100u, true));
  ASSERT_EQ(0x80000200u, ApproximateTime::BuildId(0x00000200u, true));
  ASSERT_EQ(0x80000400u, ApproximateTime::BuildId(0x00000400u, true));
  ASSERT_EQ(0x80000800u, ApproximateTime::BuildId(0x00000800u, true));
  ASSERT_EQ(0x80001000u, ApproximateTime::BuildId(0x00001000u, true));
  ASSERT_EQ(0x80002000u, ApproximateTime::BuildId(0x00002000u, true));
  ASSERT_EQ(0x80004000u, ApproximateTime::BuildId(0x00004000u, true));
  ASSERT_EQ(0x80008000u, ApproximateTime::BuildId(0x00008000u, true));
  ASSERT_EQ(0x80010000u, ApproximateTime::BuildId(0x00010000u, true));
  ASSERT_EQ(0x80020000u, ApproximateTime::BuildId(0x00020000u, true));
  ASSERT_EQ(0x80040000u, ApproximateTime::BuildId(0x00040000u, true));
  ASSERT_EQ(0x80080000u, ApproximateTime::BuildId(0x00080000u, true));
  ASSERT_EQ(0x80100000u, ApproximateTime::BuildId(0x00100000u, true));
  ASSERT_EQ(0x80200000u, ApproximateTime::BuildId(0x00200000u, true));
  ASSERT_EQ(0x80400000u, ApproximateTime::BuildId(0x00400000u, true));
  ASSERT_EQ(0x80800000u, ApproximateTime::BuildId(0x00800000u, true));
  ASSERT_EQ(0x81000000u, ApproximateTime::BuildId(0x01000000u, true));
  ASSERT_EQ(0x82000000u, ApproximateTime::BuildId(0x02000000u, true));
  ASSERT_EQ(0x84000000u, ApproximateTime::BuildId(0x04000000u, true));
  ASSERT_EQ(0x88000000u, ApproximateTime::BuildId(0x08000000u, true));
  ASSERT_EQ(0x90000000u, ApproximateTime::BuildId(0x10000000u, true));
  ASSERT_EQ(0x80000000u, ApproximateTime::BuildId(0x20000000u, true));
  ASSERT_EQ(0x80000000u, ApproximateTime::BuildId(0x40000000u, true));
  ASSERT_EQ(0x80000000u, ApproximateTime::BuildId(0x80000000u, true));
}


// Tests ported from https://github.com/ros/ros_comm/blob/2e6ac87958b59455aab0442b205163e9a5f43ff2/utilities/message_filters/test/test_approximate_time_policy.cpp
typedef can_msgs::Frame Msg;
typedef boost::shared_ptr<Msg> MsgPtr;
typedef boost::shared_ptr<Msg const> MsgConstPtr;

Msg MsgHelper(ros::Time stamp, uint32_t id, bool is_extended = false, bool is_error = false, bool is_rtr = false) {
  Msg msg;
  msg.header.stamp = stamp;
  msg.id = id;
  msg.is_extended = is_extended;
  msg.is_error = is_error;
  msg.is_rtr = is_rtr;
  return msg;
}

typedef std::pair<ros::Time, ros::Time> TimePair;
struct TimeQuad
{
  TimeQuad(ros::Time p, ros::Time q, ros::Time r, ros::Time s)
  {
    time[0] = p;
    time[1] = q;
    time[2] = r;
    time[3] = s;
  }
  ros::Time time[4];
};

//----------------------------------------------------------
//                Test Class (for 2 inputs)
//----------------------------------------------------------
class ApproximateTimeSynchronizerTest
{
public:
  ApproximateTimeSynchronizerTest(const std::vector<Msg> &input,
          const std::vector<TimePair> &output,
          uint32_t queue_size, uint32_t id1, uint32_t id2) :
    input_(input), output_(output), output_position_(0),
    sync_(queue_size, boost::bind(&ApproximateTimeSynchronizerTest::callback, this, _1), id1, id2)
  {
  }

  void callback(const std::vector<can_msgs::Frame::ConstPtr> &msgs)
  {
    //printf("Call_back called\n");
    //printf("Call back: <%f, %f>\n", msgs[0]->header.stamp.toSec(), msgs[1]->header.stamp.toSec());
    ASSERT_EQ(2u, msgs.size());
    ASSERT_TRUE(msgs[0]);
    ASSERT_TRUE(msgs[1]);
    ASSERT_LT(output_position_, output_.size());
    EXPECT_EQ(output_[output_position_].first,  msgs[0]->header.stamp);
    EXPECT_EQ(output_[output_position_].second, msgs[1]->header.stamp);
    ++output_position_;
  }

  void run()
  {
    for (size_t i = 0; i < input_.size(); i++) {
      sync_.processMsg(boost::make_shared<Msg>(input_[i]));
    }
    //printf("Done running test\n");
    EXPECT_EQ(output_.size(), output_position_);
  }

private:
  const std::vector<Msg> &input_;
  const std::vector<TimePair> &output_;
  unsigned int output_position_;
public:
  ApproximateTime sync_;
};

//----------------------------------------------------------
//                Test Class (for 4 inputs)
//----------------------------------------------------------
class ApproximateTimeSynchronizerTestQuad
{
public:

  ApproximateTimeSynchronizerTestQuad(const std::vector<Msg> &input,
              const std::vector<TimeQuad> &output,
              uint32_t queue_size, uint32_t id1, uint32_t id2, uint32_t id3, uint32_t id4) :
    input_(input), output_(output), output_position_(0),
    sync_(queue_size, boost::bind(&ApproximateTimeSynchronizerTestQuad::callback, this, _1), id1, id2, id3, id4)
  {
  }

  void callback(const std::vector<can_msgs::Frame::ConstPtr> &msgs)
  {
    //printf("Call_back called\n");
    //printf("Call back: <%f, %f>\n", msgs[0]->header.stamp.toSec(), msgs[1]->header.stamp.toSec());
    ASSERT_EQ(4u, msgs.size());
    ASSERT_TRUE(msgs[0]);
    ASSERT_TRUE(msgs[1]);
    ASSERT_TRUE(msgs[2]);
    ASSERT_TRUE(msgs[3]);
    ASSERT_LT(output_position_, output_.size());
    EXPECT_EQ(output_[output_position_].time[0], msgs[0]->header.stamp);
    EXPECT_EQ(output_[output_position_].time[1], msgs[1]->header.stamp);
    EXPECT_EQ(output_[output_position_].time[2], msgs[2]->header.stamp);
    EXPECT_EQ(output_[output_position_].time[3], msgs[3]->header.stamp);
    ++output_position_;
  }

  void run()
  {
    for (size_t i = 0; i < input_.size(); i++) {
      sync_.processMsg(boost::make_shared<Msg>(input_[i]));
    }
    //printf("Done running test\n");
    EXPECT_EQ(output_.size(), output_position_);
  }

private:
  const std::vector<Msg> &input_;
  const std::vector<TimeQuad> &output_;
  unsigned int output_position_;
public:
  ApproximateTime sync_;
};



TEST(ApproxTimeSync, ExactMatch) {
  // Input A:  a..b..c
  // Input B:  A..B..C
  // Output:   a..b..c
  //           A..B..C
  std::vector<Msg> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0)); // a
  input.push_back(MsgHelper(t+s*0,1)); // A
  input.push_back(MsgHelper(t+s*3,0)); // b
  input.push_back(MsgHelper(t+s*3,1)); // B
  input.push_back(MsgHelper(t+s*6,0)); // c
  input.push_back(MsgHelper(t+s*6,1)); // C
  output.push_back(TimePair(t, t));
  output.push_back(TimePair(t+s*3, t+s*3));
  output.push_back(TimePair(t+s*6, t+s*6));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10, 0, 1);
  sync_test.run();
}

TEST(ApproxTimeSync, PerfectMatch) {
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   ...a..b.
  //           ...A..B.
  std::vector<Msg> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0)); // a
  input.push_back(MsgHelper(t+s*1,1)); // A
  input.push_back(MsgHelper(t+s*3,0)); // b
  input.push_back(MsgHelper(t+s*4,1)); // B
  input.push_back(MsgHelper(t+s*6,0)); // c
  input.push_back(MsgHelper(t+s*7,1)); // C
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+s*3, t+s*4));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10, 0, 1);
  sync_test.run();
}

TEST(ApproxTimeSync, ImperfectMatch) {
  // Input A:  a.xb..c.
  // Input B:  .A...B.C
  // Output:   ..a...c.
  //           ..A...B.
  std::vector<Msg> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0)); // a
  input.push_back(MsgHelper(t+s*1,1)); // A
  input.push_back(MsgHelper(t+s*2,0)); // x
  input.push_back(MsgHelper(t+s*3,0)); // b
  input.push_back(MsgHelper(t+s*5,1)); // B
  input.push_back(MsgHelper(t+s*6,0)); // c
  input.push_back(MsgHelper(t+s*7,1)); // C
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+s*6, t+s*5));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10, 0, 1);
  sync_test.run();
}

TEST(ApproxTimeSync, Acceleration) {
  // Time:     0123456789012345678
  // Input A:  a...........b....c.
  // Input B:  .......A.......B..C
  // Output:   ............b.....c
  //           ............A.....C
  std::vector<Msg> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0));  // a
  input.push_back(MsgHelper(t+s*7,1));  // A
  input.push_back(MsgHelper(t+s*12,0)); // b
  input.push_back(MsgHelper(t+s*15,1)); // B
  input.push_back(MsgHelper(t+s*17,0)); // c
  input.push_back(MsgHelper(t+s*18,1)); // C
  output.push_back(TimePair(t+s*12, t+s*7));
  output.push_back(TimePair(t+s*17, t+s*18));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10, 0, 1);
  sync_test.run();
}

TEST(ApproxTimeSync, DroppedMessages) {
  // Queue size 1 (too small)
  // Time:     012345678901234
  // Input A:  a...b...c.d..e.
  // Input B:  .A.B...C...D..E
  // Output:   .......b.....d.
  //           .......B.....D.
  std::vector<Msg> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0));  // a
  input.push_back(MsgHelper(t+s*1,1));  // A
  input.push_back(MsgHelper(t+s*3,1));  // B
  input.push_back(MsgHelper(t+s*4,0));  // b
  input.push_back(MsgHelper(t+s*7,1));  // C
  input.push_back(MsgHelper(t+s*8,0));  // c
  input.push_back(MsgHelper(t+s*10,0)); // d
  input.push_back(MsgHelper(t+s*11,1)); // D
  input.push_back(MsgHelper(t+s*13,0)); // e
  input.push_back(MsgHelper(t+s*14,1)); // E
  output.push_back(TimePair(t+s*4, t+s*3));
  output.push_back(TimePair(t+s*10, t+s*11));

  ApproximateTimeSynchronizerTest sync_test(input, output, 1, 0, 1);
  sync_test.run();

  // Queue size 2 (just enough)
  // Time:     012345678901234
  // Input A:  a...b...c.d..e.
  // Input B:  .A.B...C...D..E
  // Output:   ....a..b...c.d.
  //           ....A..B...C.D.
  std::vector<TimePair> output2;
  output2.push_back(TimePair(t, t+s));
  output2.push_back(TimePair(t+s*4, t+s*3));
  output2.push_back(TimePair(t+s*8, t+s*7));
  output2.push_back(TimePair(t+s*10, t+s*11));

  ApproximateTimeSynchronizerTest sync_test2(input, output2, 2, 0, 1);
  sync_test2.run();
}

TEST(ApproxTimeSync, LongQueue) {
  // Queue size 5
  // Time:     012345678901234
  // Input A:  abcdefghiklmnp.
  // Input B:  ...j......o....
  // Output:   ..........l....
  //           ..........o....
  std::vector<Msg> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0));  // a
  input.push_back(MsgHelper(t+s*1,0));  // b
  input.push_back(MsgHelper(t+s*2,0));  // c
  input.push_back(MsgHelper(t+s*3,0));  // d
  input.push_back(MsgHelper(t+s*4,0));  // e
  input.push_back(MsgHelper(t+s*5,0));  // f
  input.push_back(MsgHelper(t+s*6,0));  // g
  input.push_back(MsgHelper(t+s*7,0));  // h
  input.push_back(MsgHelper(t+s*8,0));  // i
  input.push_back(MsgHelper(t+s*3,1));  // j
  input.push_back(MsgHelper(t+s*9,0));  // k
  input.push_back(MsgHelper(t+s*10,0)); // l
  input.push_back(MsgHelper(t+s*11,0)); // m
  input.push_back(MsgHelper(t+s*12,0)); // n
  input.push_back(MsgHelper(t+s*10,1)); // o
  input.push_back(MsgHelper(t+s*13,0)); // l
  output.push_back(TimePair(t+s*10, t+s*10));

  ApproximateTimeSynchronizerTest sync_test(input, output, 5, 0, 1);
  sync_test.run();
}

TEST(ApproxTimeSync, DoublePublish) {
  // Input A:  a..b
  // Input B:  .A.B
  // Output:   ...b
  //           ...B
  //              +
  //              a
  //              A
  std::vector<Msg> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0)); // a
  input.push_back(MsgHelper(t+s*1,1)); // A
  input.push_back(MsgHelper(t+s*3,1)); // B
  input.push_back(MsgHelper(t+s*3,0)); // b
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+s*3, t+s*3));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10, 0, 1);
  sync_test.run();
}

TEST(ApproxTimeSync, FourTopics) {
  // Time:     012345678901234
  // Input A:  a....e..i.m..n.
  // Input B:  .b....g..j....o
  // Input C:  ..c...h...k....
  // Input D:  ...d.f.....l...
  // Output:   ......a....e..m
  //           ......b....g..j
  //           ......c....h..k
  //           ......d....f..l
  std::vector<Msg> input;
  std::vector<TimeQuad> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0));  // a
  input.push_back(MsgHelper(t+s*1,1));  // b
  input.push_back(MsgHelper(t+s*2,2));  // c
  input.push_back(MsgHelper(t+s*3,3));  // d
  input.push_back(MsgHelper(t+s*5,0));  // e
  input.push_back(MsgHelper(t+s*5,3));  // f
  input.push_back(MsgHelper(t+s*6,1));  // g
  input.push_back(MsgHelper(t+s*6,2));  // h
  input.push_back(MsgHelper(t+s*8,0));  // i
  input.push_back(MsgHelper(t+s*9,1));  // j
  input.push_back(MsgHelper(t+s*10,2)); // k
  input.push_back(MsgHelper(t+s*11,3)); // l
  input.push_back(MsgHelper(t+s*10,0)); // m
  input.push_back(MsgHelper(t+s*13,0)); // n
  input.push_back(MsgHelper(t+s*14,1)); // o
  output.push_back(TimeQuad(t, t+s, t+s*2, t+s*3));
  output.push_back(TimeQuad(t+s*5, t+s*6, t+s*6, t+s*5));
  output.push_back(TimeQuad(t+s*10, t+s*9, t+s*10, t+s*11));

  ApproximateTimeSynchronizerTestQuad sync_test(input, output, 10, 0, 1, 2, 3);
  sync_test.run();
}

TEST(ApproxTimeSync, EarlyPublish) {
  // Time:     012345678901234
  // Input A:  a......e
  // Input B:  .b......
  // Input C:  ..c.....
  // Input D:  ...d....
  // Output:   .......a
  //           .......b
  //           .......c
  //           .......d
  std::vector<Msg> input;
  std::vector<TimeQuad> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0)); // a
  input.push_back(MsgHelper(t+s*1,1)); // b
  input.push_back(MsgHelper(t+s*2,2)); // c
  input.push_back(MsgHelper(t+s*3,3)); // d
  input.push_back(MsgHelper(t+s*7,0)); // e
  output.push_back(TimeQuad(t, t+s, t+s*2, t+s*3));

  ApproximateTimeSynchronizerTestQuad sync_test(input, output, 10, 0, 1, 2, 3);
  sync_test.run();
}

TEST(ApproxTimeSync, RateBound) {
  // Rate bound A: 1.5
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   .a..b...
  //           .A..B...
  std::vector<Msg> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0)); // a
  input.push_back(MsgHelper(t+s*1,1)); // A
  input.push_back(MsgHelper(t+s*3,0)); // b
  input.push_back(MsgHelper(t+s*4,1)); // B
  input.push_back(MsgHelper(t+s*6,0)); // c
  input.push_back(MsgHelper(t+s*7,1)); // C
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+s*3, t+s*4));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10, 0, 1);
  sync_test.sync_.setInterMessageLowerBound(0, s*1.5);
  sync_test.run();

  // Rate bound A: 2
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   .a..b..c
  //           .A..B..C

  output.push_back(TimePair(t+s*6, t+s*7));

  ApproximateTimeSynchronizerTest sync_test2(input, output, 10, 0, 1);
  sync_test2.sync_.setInterMessageLowerBound(0, s*2);
  sync_test2.run();
}

TEST(ApproxTimeSync, RateBoundAll) {
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   .a..b...
  //           .A..B...
  std::vector<Msg> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0)); // a
  input.push_back(MsgHelper(t+s*1,1)); // A
  input.push_back(MsgHelper(t+s*3,0)); // b
  input.push_back(MsgHelper(t+s*4,1)); // B
  input.push_back(MsgHelper(t+s*6,0)); // c
  input.push_back(MsgHelper(t+s*7,1)); // C
  output.push_back(TimePair(t, t+s));
  output.push_back(TimePair(t+s*3, t+s*4));

  ApproximateTimeSynchronizerTest sync_test(input, output, 10, 0, 1);
  sync_test.run();

  // Rate bound: 2
  // Input A:  a..b..c.
  // Input B:  .A..B..C
  // Output:   .a..b..c
  //           .A..B..C

  output.push_back(TimePair(t+s*6, t+s*7));

  ApproximateTimeSynchronizerTest sync_test2(input, output, 10, 0, 1);
  sync_test2.sync_.setInterMessageLowerBound(s*2.0);
  sync_test2.run();
}

TEST(ApproxTimeSync, ExtendedIds) {
  // Input A:  a..b..c
  // Input B:  A..B..C
  // Output:   a..b..c
  //           A..B..C
  std::vector<Msg> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0,true )); // a
  input.push_back(MsgHelper(t+s*0,1,false)); // A
  input.push_back(MsgHelper(t+s*3,0,true )); // b
  input.push_back(MsgHelper(t+s*3,1,false)); // B
  input.push_back(MsgHelper(t+s*6,0,true )); // c
  input.push_back(MsgHelper(t+s*6,1,false)); // C
  output.clear();

  ApproximateTimeSynchronizerTest sync_test1(input, output, 10, 0x00000000, 0x00000001);
  sync_test1.run();

  ApproximateTimeSynchronizerTest sync_test2(input, output, 10, 0x00000000, 0x80000001);
  sync_test2.run();

  output.push_back(TimePair(t, t));
  output.push_back(TimePair(t+s*3, t+s*3));
  output.push_back(TimePair(t+s*6, t+s*6));

  ApproximateTimeSynchronizerTest sync_test3(input, output, 10, 0x80000000, 0x00000001);
  sync_test3.run();
}

TEST(ApproxTimeSync, ErrorFrames) {
  // Input A:  a..b..c
  // Input B:  A..B..C
  // Output:   a..b..c
  //           A..B..C
  std::vector<Msg> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0,false,true)); // a
  input.push_back(MsgHelper(t+s*0,1,false,true)); // A
  input.push_back(MsgHelper(t+s*3,0,false,true)); // b
  input.push_back(MsgHelper(t+s*3,1,false,true)); // B
  input.push_back(MsgHelper(t+s*6,0,false,true)); // c
  input.push_back(MsgHelper(t+s*6,1,false,true)); // C
  output.clear();

  ApproximateTimeSynchronizerTest sync_test(input, output, 10, 0, 1);
  sync_test.run();
}

TEST(ApproxTimeSync, RtrFrames) {
  // Input A:  a..b..c
  // Input B:  A..B..C
  // Output:   a..b..c
  //           A..B..C
  std::vector<Msg> input;
  std::vector<TimePair> output;

  ros::Time t(0, 0);
  ros::Duration s(1, 0);

  input.push_back(MsgHelper(t+s*0,0,false,false,true)); // a
  input.push_back(MsgHelper(t+s*0,1,false,false,true)); // A
  input.push_back(MsgHelper(t+s*3,0,false,false,true)); // b
  input.push_back(MsgHelper(t+s*3,1,false,false,true)); // B
  input.push_back(MsgHelper(t+s*6,0,false,false,true)); // c
  input.push_back(MsgHelper(t+s*6,1,false,false,true)); // C
  output.clear();

  ApproximateTimeSynchronizerTest sync_test(input, output, 10, 0, 1);
  sync_test.run();
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

