/*********************************************************************
 * C++ unit test for dataspeed_can_tools/DbcIterator.hpp
 *********************************************************************/

#include <gtest/gtest.h>

#include <ros/package.h>

// String stream
#include <iostream>
#include <fstream>
#include <string>

// File under test
#include "../src/DbcIterator.hpp"

bool fileExists(const std::string& name) {
  std::ifstream f(name.c_str());
  return f.good();
}

// Check that parsing valid signals does not cause an error.
TEST(ITERATOR, parsing)
{
  ASSERT_TRUE(fileExists(ros::package::getPath("dataspeed_can_tools")+"/tests/Test.dbc")) << "Could not find dbc file.";
  try {
    DBCIterator dbc(ros::package::getPath("dataspeed_can_tools")+"/tests/Test.dbc");
  } catch (const std::exception&) {
    printf("Could not open file.\n");
    FAIL();
  }
}

// Check the values output by parsing valid signals.
TEST(ITERATOR, data)
{
  ASSERT_TRUE(fileExists(ros::package::getPath("dataspeed_can_tools")+"/tests/Test.dbc")) << "Could not find dbc file.";
  DBCIterator dbc(ros::package::getPath("dataspeed_can_tools")+"/tests/Test.dbc");
  
  Message msg = dbc[0];
  EXPECT_EQ(msg.getId(), 166u);
  EXPECT_EQ(msg.getName(), "MultiplexTest");
  EXPECT_EQ(msg.getDlc(), 5u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");

  EXPECT_EQ(msg[0].getName(), "MultiplexedD");
  EXPECT_EQ(msg[0].getMultiplexor(), MULTIPLEXED);
  EXPECT_EQ(msg[0].getMultiplexedNumber(), 51);
  EXPECT_EQ(msg[0].getStartbit(), 8);
  EXPECT_EQ(msg[0].getLength(), 32);
  EXPECT_EQ(msg[0].getByteOrder(), INTEL);
  EXPECT_EQ(msg[0].getSign(), SIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");

  EXPECT_EQ(msg[1].getName(), "MultiplexedC");
  EXPECT_EQ(msg[1].getMultiplexor(), MULTIPLEXED);
  EXPECT_EQ(msg[1].getMultiplexedNumber(), 34);
  EXPECT_EQ(msg[1].getStartbit(), 16);
  EXPECT_EQ(msg[1].getLength(), 16);
  EXPECT_EQ(msg[1].getByteOrder(), INTEL);
  EXPECT_EQ(msg[1].getSign(), UNSIGNED);
  EXPECT_EQ(msg[1].getFactor(), 1);
  EXPECT_EQ(msg[1].getOffset(), 0);
  EXPECT_EQ(msg[1].getMinimum(), 0);
  EXPECT_EQ(msg[1].getMaximum(), 0);
  EXPECT_EQ(msg[1].getUnit(), "");

  EXPECT_EQ(msg[2].getName(), "MultiplexedB");
  EXPECT_EQ(msg[2].getMultiplexor(), MULTIPLEXED);
  EXPECT_EQ(msg[2].getMultiplexedNumber(), 34);
  EXPECT_EQ(msg[2].getStartbit(), 8);
  EXPECT_EQ(msg[2].getLength(), 8);
  EXPECT_EQ(msg[2].getByteOrder(), INTEL);
  EXPECT_EQ(msg[2].getSign(), SIGNED);
  EXPECT_EQ(msg[2].getFactor(), 1);
  EXPECT_EQ(msg[2].getOffset(), 0);
  EXPECT_EQ(msg[2].getMinimum(), 0);
  EXPECT_EQ(msg[2].getMaximum(), 0);
  EXPECT_EQ(msg[2].getUnit(), "");

  EXPECT_EQ(msg[3].getName(), "MultiplexedA");
  EXPECT_EQ(msg[3].getMultiplexor(), MULTIPLEXED);
  EXPECT_EQ(msg[3].getMultiplexedNumber(), 17);
  EXPECT_EQ(msg[3].getStartbit(), 8);
  EXPECT_EQ(msg[3].getLength(), 8);
  EXPECT_EQ(msg[3].getByteOrder(), INTEL);
  EXPECT_EQ(msg[3].getSign(), SIGNED);
  EXPECT_EQ(msg[3].getFactor(), 1);
  EXPECT_EQ(msg[3].getOffset(), 0);
  EXPECT_EQ(msg[3].getMinimum(), 0);
  EXPECT_EQ(msg[3].getMaximum(), 0);
  EXPECT_EQ(msg[3].getUnit(), "");

  EXPECT_EQ(msg[3].getName(), "MultiplexedA");
  EXPECT_EQ(msg[3].getMultiplexor(), MULTIPLEXED);
  EXPECT_EQ(msg[3].getMultiplexedNumber(), 17);
  EXPECT_EQ(msg[3].getStartbit(), 8);
  EXPECT_EQ(msg[3].getLength(), 8);
  EXPECT_EQ(msg[3].getByteOrder(), INTEL);
  EXPECT_EQ(msg[3].getSign(), SIGNED);
  EXPECT_EQ(msg[3].getFactor(), 1);
  EXPECT_EQ(msg[3].getOffset(), 0);
  EXPECT_EQ(msg[3].getMinimum(), 0);
  EXPECT_EQ(msg[3].getMaximum(), 0);
  EXPECT_EQ(msg[3].getUnit(), "");

  EXPECT_EQ(msg[4].getName(), "Multiplexor");
  EXPECT_EQ(msg[4].getMultiplexor(), MULTIPLEXOR);
  EXPECT_EQ(msg[4].getStartbit(), 0);
  EXPECT_EQ(msg[4].getLength(), 8);
  EXPECT_EQ(msg[4].getByteOrder(), INTEL);
  EXPECT_EQ(msg[4].getSign(), UNSIGNED);
  EXPECT_EQ(msg[4].getFactor(), 1);
  EXPECT_EQ(msg[4].getOffset(), 0);
  EXPECT_EQ(msg[4].getMinimum(), 0);
  EXPECT_EQ(msg[4].getMaximum(), 0);
  EXPECT_EQ(msg[4].getUnit(), "");

  // AdvancedTestD
  msg = dbc[1];
  EXPECT_EQ(msg.getId(), 165u);
  EXPECT_EQ(msg.getName(), "AdvancedTestD");
  EXPECT_EQ(msg.getDlc(), 4u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");

  EXPECT_EQ(msg[0].getName(), "AdvancedSignal8");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 24);
  EXPECT_EQ(msg[0].getLength(), 8);
  EXPECT_EQ(msg[0].getByteOrder(), 1);
  EXPECT_EQ(msg[0].getSign(), UNSIGNED);
  EXPECT_EQ(msg[0].getFactor(), -10);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");

  EXPECT_EQ(msg[1].getName(), "AdvancedSignal7");
  EXPECT_EQ(msg[1].getMultiplexor(), NONE);
  EXPECT_EQ(msg[1].getStartbit(), 16);
  EXPECT_EQ(msg[1].getLength(), 8);
  EXPECT_EQ(msg[1].getByteOrder(), 1);
  EXPECT_EQ(msg[1].getSign(), SIGNED);
  EXPECT_EQ(msg[1].getFactor(), -10);
  EXPECT_EQ(msg[1].getOffset(), 0);
  EXPECT_EQ(msg[1].getMinimum(), 0);
  EXPECT_EQ(msg[1].getMaximum(), 0);
  EXPECT_EQ(msg[1].getUnit(), "");

  EXPECT_EQ(msg[2].getName(), "AdvancedSignal6");
  EXPECT_EQ(msg[2].getMultiplexor(), NONE);
  EXPECT_EQ(msg[2].getStartbit(), 8);
  EXPECT_EQ(msg[2].getLength(), 8);
  EXPECT_EQ(msg[2].getByteOrder(), INTEL);
  EXPECT_EQ(msg[2].getSign(), UNSIGNED);
  EXPECT_EQ(msg[2].getFactor(), 10);
  EXPECT_EQ(msg[2].getOffset(), 0);
  EXPECT_EQ(msg[2].getMinimum(), 0);
  EXPECT_EQ(msg[2].getMaximum(), 0);
  EXPECT_EQ(msg[2].getUnit(), "");

  EXPECT_EQ(msg[3].getName(), "AdvancedSignal5");
  EXPECT_EQ(msg[3].getMultiplexor(), NONE);
  EXPECT_EQ(msg[3].getStartbit(), 0);
  EXPECT_EQ(msg[3].getLength(), 8);
  EXPECT_EQ(msg[3].getByteOrder(), INTEL);
  EXPECT_EQ(msg[3].getSign(), SIGNED);
  EXPECT_EQ(msg[3].getFactor(), 10);
  EXPECT_EQ(msg[3].getOffset(), 0);
  EXPECT_EQ(msg[3].getMinimum(), 0);
  EXPECT_EQ(msg[3].getMaximum(), 0);
  EXPECT_EQ(msg[3].getUnit(), "");

  // AdvancedTestC
  msg = dbc[2];
  EXPECT_EQ(msg.getId(), 164u);
  EXPECT_EQ(msg.getName(), "AdvancedTestC");
  EXPECT_EQ(msg.getDlc(), 8u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");

  EXPECT_EQ(msg[0].getName(), "AdvancedSignal4");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 7);
  EXPECT_EQ(msg[0].getLength(), 64);
  EXPECT_EQ(msg[0].getByteOrder(), MOTOROLA);
  EXPECT_EQ(msg[0].getSign(), SIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");

  // AdvancedTestB
  msg = dbc[3];
  EXPECT_EQ(msg.getId(), 163u);
  EXPECT_EQ(msg.getName(), "AdvancedTestB");
  EXPECT_EQ(msg.getDlc(), 8u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");

  EXPECT_EQ(msg[0].getName(), "AdvancedSignal3");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 0);
  EXPECT_EQ(msg[0].getLength(), 64);
  EXPECT_EQ(msg[0].getByteOrder(), INTEL);
  EXPECT_EQ(msg[0].getSign(), SIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");

  // AdvancedTestA
  msg = dbc[4];
  EXPECT_EQ(msg.getId(), 162u);
  EXPECT_EQ(msg.getName(), "AdvancedTestA");
  EXPECT_EQ(msg.getDlc(), 8u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");

  EXPECT_EQ(msg[0].getName(), "AdvancedSignal2");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 39);
  EXPECT_EQ(msg[0].getLength(), 32);
  EXPECT_EQ(msg[0].getByteOrder(), MOTOROLA);
  EXPECT_EQ(msg[0].getSign(), SIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");
  
  EXPECT_EQ(msg[1].getName(), "AdvancedSignal1");
  EXPECT_EQ(msg[1].getMultiplexor(), NONE);
  EXPECT_EQ(msg[1].getStartbit(), 0);
  EXPECT_EQ(msg[1].getLength(), 32);
  EXPECT_EQ(msg[1].getByteOrder(), INTEL);
  EXPECT_EQ(msg[1].getSign(), SIGNED);
  EXPECT_EQ(msg[1].getFactor(), 1);
  EXPECT_EQ(msg[1].getOffset(), 0);
  EXPECT_EQ(msg[1].getMinimum(), 0);
  EXPECT_EQ(msg[1].getMaximum(), 0);
  EXPECT_EQ(msg[1].getUnit(), "");

  // MotorolaTestD
  msg = dbc[5];
  EXPECT_EQ(msg.getId(), 196u);
  EXPECT_EQ(msg.getName(), "MotorolaTestD");
  EXPECT_EQ(msg.getDlc(), 8u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");
  EXPECT_EQ(msg[0].getName(), "MotorolaSignalU64");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 7);
  EXPECT_EQ(msg[0].getLength(), 64);
  EXPECT_EQ(msg[0].getByteOrder(), MOTOROLA);
  EXPECT_EQ(msg[0].getSign(), UNSIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");

  // MotorolaTestC
  msg = dbc[6];
  EXPECT_EQ(msg.getId(), 195u);
  EXPECT_EQ(msg.getName(), "MotorolaTestC");
  EXPECT_EQ(msg.getDlc(), 8u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");
  EXPECT_EQ(msg[0].getName(), "MotorolaSignal64");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 7);
  EXPECT_EQ(msg[0].getLength(), 64);
  EXPECT_EQ(msg[0].getByteOrder(), MOTOROLA);
  EXPECT_EQ(msg[0].getSign(), SIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");

  // MotorolaTestB
  msg = dbc[7];
  EXPECT_EQ(msg.getId(), 194u);
  EXPECT_EQ(msg.getName(), "MotorolaTestB");
  EXPECT_EQ(msg.getDlc(), 8u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");

  EXPECT_EQ(msg[0].getName(), "MotorolaSignalU32");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 39);
  EXPECT_EQ(msg[0].getLength(), 32);
  EXPECT_EQ(msg[0].getByteOrder(), MOTOROLA);
  EXPECT_EQ(msg[0].getSign(), UNSIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");
  
  EXPECT_EQ(msg[1].getName(), "MotorolaSignal32");
  EXPECT_EQ(msg[1].getMultiplexor(), NONE);
  EXPECT_EQ(msg[1].getStartbit(), 7);
  EXPECT_EQ(msg[1].getLength(), 32);
  EXPECT_EQ(msg[1].getByteOrder(), MOTOROLA);
  EXPECT_EQ(msg[1].getSign(), SIGNED);
  EXPECT_EQ(msg[1].getFactor(), 1);
  EXPECT_EQ(msg[1].getOffset(), 0);
  EXPECT_EQ(msg[1].getMinimum(), 0);
  EXPECT_EQ(msg[1].getMaximum(), 0);
  EXPECT_EQ(msg[1].getUnit(), "");

  // MotorolaTestA
  msg = dbc[8];
  EXPECT_EQ(msg.getId(), 193u);
  EXPECT_EQ(msg.getName(), "MotorolaTestA");
  EXPECT_EQ(msg.getDlc(), 6u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");
  
  EXPECT_EQ(msg[0].getName(), "MotorolaSignalU16");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 39);
  EXPECT_EQ(msg[0].getLength(), 16);
  EXPECT_EQ(msg[0].getByteOrder(), MOTOROLA);
  EXPECT_EQ(msg[0].getSign(), UNSIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");
  
  EXPECT_EQ(msg[1].getName(), "MotorolaSignal16");
  EXPECT_EQ(msg[1].getMultiplexor(), NONE);
  EXPECT_EQ(msg[1].getStartbit(), 23);
  EXPECT_EQ(msg[1].getLength(), 16);
  EXPECT_EQ(msg[1].getByteOrder(), MOTOROLA);
  EXPECT_EQ(msg[1].getSign(), SIGNED);
  EXPECT_EQ(msg[1].getFactor(), 1);
  EXPECT_EQ(msg[1].getOffset(), 0);
  EXPECT_EQ(msg[1].getMinimum(), 0);
  EXPECT_EQ(msg[1].getMaximum(), 0);
  EXPECT_EQ(msg[1].getUnit(), "");
  
  EXPECT_EQ(msg[2].getName(), "MotorolaSignalU8");
  EXPECT_EQ(msg[2].getMultiplexor(), NONE);
  EXPECT_EQ(msg[2].getStartbit(), 15);
  EXPECT_EQ(msg[2].getLength(), 8);
  EXPECT_EQ(msg[2].getByteOrder(), MOTOROLA);
  EXPECT_EQ(msg[2].getSign(), UNSIGNED);
  EXPECT_EQ(msg[2].getFactor(), 1);
  EXPECT_EQ(msg[2].getOffset(), 0);
  EXPECT_EQ(msg[2].getMinimum(), 0);
  EXPECT_EQ(msg[2].getMaximum(), 0);
  EXPECT_EQ(msg[2].getUnit(), "");

  EXPECT_EQ(msg[3].getName(), "MotorolaSignal8");
  EXPECT_EQ(msg[3].getMultiplexor(), NONE);
  EXPECT_EQ(msg[3].getStartbit(), 7);
  EXPECT_EQ(msg[3].getLength(), 8);
  EXPECT_EQ(msg[3].getByteOrder(), MOTOROLA);
  EXPECT_EQ(msg[3].getSign(), SIGNED);
  EXPECT_EQ(msg[3].getFactor(), 1);
  EXPECT_EQ(msg[3].getOffset(), 0);
  EXPECT_EQ(msg[3].getMinimum(), 0);
  EXPECT_EQ(msg[3].getMaximum(), 0);
  EXPECT_EQ(msg[3].getUnit(), "");

  // IntelTestD
  msg = dbc[9];
  EXPECT_EQ(msg.getId(), 180u);
  EXPECT_EQ(msg.getName(), "IntelTestD");
  EXPECT_EQ(msg.getDlc(), 8u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");
  EXPECT_EQ(msg[0].getName(), "IntelSignalU64");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 0);
  EXPECT_EQ(msg[0].getLength(), 64);
  EXPECT_EQ(msg[0].getByteOrder(), INTEL);
  EXPECT_EQ(msg[0].getSign(), UNSIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");

  // IntelTestC
  msg = dbc[10];
  EXPECT_EQ(msg.getId(), 179u);
  EXPECT_EQ(msg.getName(), "IntelTestC");
  EXPECT_EQ(msg.getDlc(), 8u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");
  EXPECT_EQ(msg[0].getName(), "IntelSignal64");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 0);
  EXPECT_EQ(msg[0].getLength(), 64);
  EXPECT_EQ(msg[0].getByteOrder(), INTEL);
  EXPECT_EQ(msg[0].getSign(), SIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");

  // IntelTestB
  msg = dbc[11];
  EXPECT_EQ(msg.getId(), 178u);
  EXPECT_EQ(msg.getName(), "IntelTestB");
  EXPECT_EQ(msg.getDlc(), 8u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");

  EXPECT_EQ(msg[0].getName(), "IntelSignalU32");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 32);
  EXPECT_EQ(msg[0].getLength(), 32);
  EXPECT_EQ(msg[0].getByteOrder(), INTEL);
  EXPECT_EQ(msg[0].getSign(), UNSIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");

  EXPECT_EQ(msg[1].getName(), "IntelSignal32");
  EXPECT_EQ(msg[1].getMultiplexor(), NONE);
  EXPECT_EQ(msg[1].getStartbit(), 0);
  EXPECT_EQ(msg[1].getLength(), 32);
  EXPECT_EQ(msg[1].getByteOrder(), INTEL);
  EXPECT_EQ(msg[1].getSign(), SIGNED);
  EXPECT_EQ(msg[1].getFactor(), 1);
  EXPECT_EQ(msg[1].getOffset(), 0);
  EXPECT_EQ(msg[1].getMinimum(), 0);
  EXPECT_EQ(msg[1].getMaximum(), 0);
  EXPECT_EQ(msg[1].getUnit(), "");

  // IntelTestA
  msg = dbc[12];
  EXPECT_EQ(msg.getId(), 177u);
  EXPECT_EQ(msg.getName(), "IntelTestA");
  EXPECT_EQ(msg.getDlc(), 6u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");

  EXPECT_EQ(msg[0].getName(), "IntelSignalU16");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 32);
  EXPECT_EQ(msg[0].getLength(), 16);
  EXPECT_EQ(msg[0].getByteOrder(), INTEL);
  EXPECT_EQ(msg[0].getSign(), UNSIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");

  EXPECT_EQ(msg[1].getName(), "IntelSignal16");
  EXPECT_EQ(msg[1].getMultiplexor(), NONE);
  EXPECT_EQ(msg[1].getStartbit(), 16);
  EXPECT_EQ(msg[1].getLength(), 16);
  EXPECT_EQ(msg[1].getByteOrder(), INTEL);
  EXPECT_EQ(msg[1].getSign(), SIGNED);
  EXPECT_EQ(msg[1].getFactor(), 1);
  EXPECT_EQ(msg[1].getOffset(), 0);
  EXPECT_EQ(msg[1].getMinimum(), 0);
  EXPECT_EQ(msg[1].getMaximum(), 0);
  EXPECT_EQ(msg[1].getUnit(), "");
  
  EXPECT_EQ(msg[2].getName(), "IntelSignalU8");
  EXPECT_EQ(msg[2].getMultiplexor(), NONE);
  EXPECT_EQ(msg[2].getStartbit(), 8);
  EXPECT_EQ(msg[2].getLength(), 8);
  EXPECT_EQ(msg[2].getByteOrder(), INTEL);
  EXPECT_EQ(msg[2].getSign(), UNSIGNED);
  EXPECT_EQ(msg[2].getFactor(), 1);
  EXPECT_EQ(msg[2].getOffset(), 0);
  EXPECT_EQ(msg[2].getMinimum(), 0);
  EXPECT_EQ(msg[2].getMaximum(), 0);
  EXPECT_EQ(msg[2].getUnit(), "");
  
  EXPECT_EQ(msg[3].getName(), "IntelSignal8");
  EXPECT_EQ(msg[3].getMultiplexor(), NONE);
  EXPECT_EQ(msg[3].getStartbit(), 0);
  EXPECT_EQ(msg[3].getLength(), 8);
  EXPECT_EQ(msg[3].getByteOrder(), INTEL);
  EXPECT_EQ(msg[3].getSign(), SIGNED);
  EXPECT_EQ(msg[3].getFactor(), 1);
  EXPECT_EQ(msg[3].getOffset(), 0);
  EXPECT_EQ(msg[3].getMinimum(), 0);
  EXPECT_EQ(msg[3].getMaximum(), 0);
  EXPECT_EQ(msg[3].getUnit(), "");

  // BasicTest
  msg = dbc[13];
  EXPECT_EQ(msg.getId(), 161u);
  EXPECT_EQ(msg.getName(), "BasicTest");
  EXPECT_EQ(msg.getDlc(), 4u);
  EXPECT_EQ(msg.getFrom(), "Vector__XXX");

  EXPECT_EQ(msg[0].getName(), "BasicSignal4");
  EXPECT_EQ(msg[0].getMultiplexor(), NONE);
  EXPECT_EQ(msg[0].getStartbit(), 24);
  EXPECT_EQ(msg[0].getLength(), 1);
  EXPECT_EQ(msg[0].getByteOrder(), INTEL);
  EXPECT_EQ(msg[0].getSign(), UNSIGNED);
  EXPECT_EQ(msg[0].getFactor(), 1);
  EXPECT_EQ(msg[0].getOffset(), 0);
  EXPECT_EQ(msg[0].getMinimum(), 0);
  EXPECT_EQ(msg[0].getMaximum(), 0);
  EXPECT_EQ(msg[0].getUnit(), "");

  EXPECT_EQ(msg[1].getName(), "BasicSignal3");
  EXPECT_EQ(msg[1].getMultiplexor(), NONE);
  EXPECT_EQ(msg[1].getStartbit(), 16);
  EXPECT_EQ(msg[1].getLength(), 7);
  EXPECT_EQ(msg[1].getByteOrder(), INTEL);
  EXPECT_EQ(msg[1].getSign(), SIGNED);
  EXPECT_EQ(msg[1].getFactor(), 1);
  EXPECT_EQ(msg[1].getOffset(), 18);
  EXPECT_EQ(msg[1].getMinimum(), 0);
  EXPECT_EQ(msg[1].getMaximum(), 0);
  EXPECT_EQ(msg[1].getUnit(), "");

  EXPECT_EQ(msg[2].getName(), "BasicSignal2");
  EXPECT_EQ(msg[2].getMultiplexor(), NONE);
  EXPECT_EQ(msg[2].getStartbit(), 8);
  EXPECT_EQ(msg[2].getLength(), 7);
  EXPECT_EQ(msg[2].getByteOrder(), INTEL);
  EXPECT_EQ(msg[2].getSign(), SIGNED);
  EXPECT_EQ(msg[2].getFactor(), 1);
  EXPECT_EQ(msg[2].getOffset(), 8);
  EXPECT_EQ(msg[2].getMinimum(), 0);
  EXPECT_EQ(msg[2].getMaximum(), 0);
  EXPECT_EQ(msg[2].getUnit(), "");

  EXPECT_EQ(msg[3].getName(), "BasicSignal1");
  EXPECT_EQ(msg[3].getMultiplexor(), NONE);
  EXPECT_EQ(msg[3].getStartbit(), 0);
  EXPECT_EQ(msg[3].getLength(), 8);
  EXPECT_EQ(msg[3].getByteOrder(), INTEL);
  EXPECT_EQ(msg[3].getSign(), SIGNED);
  EXPECT_EQ(msg[3].getFactor(), 1);
  EXPECT_EQ(msg[3].getOffset(), 0);
  EXPECT_EQ(msg[3].getMinimum(), 0);
  EXPECT_EQ(msg[3].getMaximum(), 0);
  EXPECT_EQ(msg[3].getUnit(), "");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

