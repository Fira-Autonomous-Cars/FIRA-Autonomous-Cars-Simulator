/*********************************************************************
 * C++ unit test for dataspeed_can_tools/DbcMessage.hpp
 *********************************************************************/

#include <gtest/gtest.h>

// String stream
#include <iostream>
#include <sstream>
#include <string>

// File under test
#include "../src/DbcMessage.hpp"

// Check that parsing valid signals does not cause an error.
TEST(MESSAGE, parsing)
{
  Message msg;
  std::istringstream in;

  std::string input = "BO_ 126 License: 8 EPAS\n";
  input += " SG_ EXPIRED : 10|1@1+ (1,0) [0|0] \"\"  MAB\n";
  input += " SG_ MUX M : 0|8@1+ (1,0) [0|0] \"\"  MAB\n";
  input += " SG_ FEAT_BASE_TRIALS_USED m0 : 32|16@1+ (1,0) [0|0] \"\"  MAB\n";
  input += " SG_ DATE1 m129 : 24|8@1+ (1,0) [0|0] \"\"  MAB";
  in.str(input);
  in >> msg;
  ASSERT_FALSE(in.fail());
  in.clear();
}

// Check the values output by parsing valid signals.
TEST(MESSAGE, data)
{
  Message msg;
  std::istringstream in;

  std::string input = "BO_ 126 License: 8 EPAS\n";
  input += " SG_ EXPIRED : 10|1@1+ (1,0) [0|0] \"\"  MAB\n";
  input += " SG_ MUX M : 0|8@1+ (1,0) [0|0] \"\"  MAB\n";
  input += " SG_ FEAT_BASE_TRIALS_USED m0 : 32|16@1+ (1,0) [0|0] \"\"  MAB\n";
  input += " SG_ DATE1 m129 : 24|8@1+ (1,0) [0|0] \"\"  MAB";
  in.str(input);
  in >> msg;
  ASSERT_FALSE(in.fail());
  // Main
  EXPECT_EQ(msg.getId(), 126u);
  EXPECT_EQ(msg.getName(), "License");
  EXPECT_EQ(msg.getDlc(), 8u);
  EXPECT_EQ(msg.getFrom(), "EPAS");
  // sig[0]
  EXPECT_EQ(msg[0].getName(), "EXPIRED");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  //EXPECT_EQ(sig[0].getMultiplexedNumber(), 0);
  EXPECT_EQ(msg[0].getStartbit(), 10);
  EXPECT_EQ(msg[0].getLength(), 1);
  EXPECT_EQ(msg[0].getByteOrder(), INTEL);
  EXPECT_EQ(msg[0].getSign(), UNSIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");
  // Receivers?
  // sig[1]
  EXPECT_EQ(msg[1].getName(), "MUX");
  EXPECT_EQ(msg[1].getMultiplexor(), MULTIPLEXOR);
  //EXPECT_EQ(msg[1].getMultiplexedNumber(), 0);
  EXPECT_EQ(msg[1].getStartbit(), 0);
  EXPECT_EQ(msg[1].getLength(), 8);
  EXPECT_EQ(msg[1].getByteOrder(), INTEL);
  EXPECT_EQ(msg[1].getSign(), UNSIGNED);
  EXPECT_EQ(msg[1].getFactor(), 1);
  EXPECT_EQ(msg[1].getOffset(), 0);
  EXPECT_EQ(msg[1].getMinimum(), 0);
  EXPECT_EQ(msg[1].getMaximum(), 0);
  EXPECT_EQ(msg[1].getUnit(), "");
  // Receivers?
  // sig[2]
  EXPECT_EQ(msg[2].getName(), "FEAT_BASE_TRIALS_USED");
  EXPECT_EQ(msg[2].getMultiplexor(), MULTIPLEXED);
  EXPECT_EQ(msg[2].getMultiplexedNumber(), 0);
  EXPECT_EQ(msg[2].getStartbit(), 32);
  EXPECT_EQ(msg[2].getLength(), 16);
  EXPECT_EQ(msg[2].getByteOrder(), INTEL);
  EXPECT_EQ(msg[2].getSign(), UNSIGNED);
  EXPECT_EQ(msg[2].getFactor(), 1);
  EXPECT_EQ(msg[2].getOffset(), 0);
  EXPECT_EQ(msg[2].getMinimum(), 0);
  EXPECT_EQ(msg[2].getMaximum(), 0);
  EXPECT_EQ(msg[2].getUnit(), "");
  // Receivers?
  // sig[3]
  EXPECT_EQ(msg[3].getName(), "DATE1");
  EXPECT_EQ(msg[3].getMultiplexor(), MULTIPLEXED);
  EXPECT_EQ(msg[3].getMultiplexedNumber(), 129);
  EXPECT_EQ(msg[3].getStartbit(), 24);
  EXPECT_EQ(msg[3].getLength(), 8);
  EXPECT_EQ(msg[3].getByteOrder(), INTEL);
  EXPECT_EQ(msg[3].getSign(), UNSIGNED);
  EXPECT_EQ(msg[3].getFactor(), 1);
  EXPECT_EQ(msg[3].getOffset(), 0);
  EXPECT_EQ(msg[3].getMinimum(), 0);
  EXPECT_EQ(msg[3].getMaximum(), 0);
  EXPECT_EQ(msg[3].getUnit(), "");
  // Receivers?

  in.clear();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

