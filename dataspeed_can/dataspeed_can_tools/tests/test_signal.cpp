/*********************************************************************
 * C++ unit test for dataspeed_can_tools/DbcSignal.hpp
 *********************************************************************/

#include <gtest/gtest.h>

// String stream
#include <iostream>
#include <sstream>
#include <string>

// File under test
#include "../src/DbcSignal.hpp"

// Check that parsing valid signals does not cause an error.
TEST(SIGNAL, parsing)
{
  Signal sig;
  std::istringstream in;

  // Normal (non-Multiplexed) bool
  in.str("SG_ TEST : 8|1@1+ (1,0) [0|0] \"\"  MAB");
  in >> sig;
  ASSERT_FALSE(in.fail());
  in.clear();

  // Multiplexed (0) bool
  in.str("SG_ FEAT_BASE_ENABLED m0 : 16|1@1+ (1,0) [0|0] \"\"  MAB");
  in >> sig;
  ASSERT_FALSE(in.fail());
  in.clear();

  // Multiplexor
  in.str("SG_ MUX M : 0|8@1+ (1,0) [0|0] \"\"  MAB");
  in >> sig;
  ASSERT_FALSE(in.fail());
  in.clear();

  // Multiplexed (129) short
  in.str("SG_ DATE1 m129 : 24|8@1+ (1,0) [0|0] \"\"  MAB");
  in >> sig;
  ASSERT_FALSE(in.fail());
  in.clear();
}

// Check the values output by parsing valid signals.
TEST(SIGNAL, data)
{
  Signal sig;
  std::istringstream in;

  // Normal (non-Multiplexed) bool
  in.str("SG_ TEST : 8|1@1+ (1,0) [0|0] \"\"  MAB");
  in >> sig;
  ASSERT_FALSE(in.fail());
  EXPECT_EQ(sig.getName(), "TEST");
  EXPECT_EQ(sig.getMultiplexor(), NONE);
  //EXPECT_EQ(sig.getMultiplexedNumber(), 0);
  EXPECT_EQ(sig.getStartbit(), 8);
  EXPECT_EQ(sig.getLength(), 1);
  EXPECT_EQ(sig.getByteOrder(), INTEL);
  EXPECT_EQ(sig.getSign(), UNSIGNED);
  EXPECT_EQ(sig.getFactor(), 1);
  EXPECT_EQ(sig.getOffset(), 0);
  EXPECT_EQ(sig.getMinimum(), 0);
  EXPECT_EQ(sig.getMaximum(), 0);
  EXPECT_EQ(sig.getUnit(), "");
  // Receivers?
  in.clear();

  // Multiplexed (0) bool
  in.str("SG_ FEAT_BASE_ENABLED m0 : 16|1@1+ (1,0) [0|0] \"\"  MAB");
  in >> sig;
  ASSERT_FALSE(in.fail());
  EXPECT_EQ(sig.getName(), "FEAT_BASE_ENABLED");
  EXPECT_EQ(sig.getMultiplexor(), MULTIPLEXED);
  EXPECT_EQ(sig.getMultiplexedNumber(), 0);
  EXPECT_EQ(sig.getStartbit(), 16);
  EXPECT_EQ(sig.getLength(), 1);
  EXPECT_EQ(sig.getByteOrder(), INTEL);
  EXPECT_EQ(sig.getSign(), UNSIGNED);
  EXPECT_EQ(sig.getFactor(), 1);
  EXPECT_EQ(sig.getOffset(), 0);
  EXPECT_EQ(sig.getMinimum(), 0);
  EXPECT_EQ(sig.getMaximum(), 0);
  EXPECT_EQ(sig.getUnit(), "");
  // Receivers?
  in.clear();

  // Multiplexor
  in.str("SG_ MUX M : 0|8@1+ (1,0) [0|0] \"\"  MAB");
  in >> sig;
  ASSERT_FALSE(in.fail());
  EXPECT_EQ(sig.getName(), "MUX");
  EXPECT_EQ(sig.getMultiplexor(), MULTIPLEXOR);
  //EXPECT_EQ(sig.getMultiplexedNumber(), 129);
  EXPECT_EQ(sig.getStartbit(), 0);
  EXPECT_EQ(sig.getLength(), 8);
  EXPECT_EQ(sig.getByteOrder(), INTEL);
  EXPECT_EQ(sig.getSign(), UNSIGNED);
  EXPECT_EQ(sig.getFactor(), 1);
  EXPECT_EQ(sig.getOffset(), 0);
  EXPECT_EQ(sig.getMinimum(), 0);
  EXPECT_EQ(sig.getMaximum(), 0);
  EXPECT_EQ(sig.getUnit(), "");
  // Receivers?
  in.clear();

  // Multiplexed (129) short
  in.str("SG_ DATE1 m129 : 24|8@1+ (1,0) [0|0] \"\"  MAB");
  in >> sig;
  ASSERT_FALSE(in.fail());
  EXPECT_EQ(sig.getName(), "DATE1");
  EXPECT_EQ(sig.getMultiplexor(), MULTIPLEXED);
  EXPECT_EQ(sig.getMultiplexedNumber(), 129);
  EXPECT_EQ(sig.getStartbit(), 24);
  EXPECT_EQ(sig.getLength(), 8);
  EXPECT_EQ(sig.getByteOrder(), INTEL);
  EXPECT_EQ(sig.getSign(), UNSIGNED);
  EXPECT_EQ(sig.getFactor(), 1);
  EXPECT_EQ(sig.getOffset(), 0);
  EXPECT_EQ(sig.getMinimum(), 0);
  EXPECT_EQ(sig.getMaximum(), 0);
  EXPECT_EQ(sig.getUnit(), "");
  // Receivers?
  in.clear();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

