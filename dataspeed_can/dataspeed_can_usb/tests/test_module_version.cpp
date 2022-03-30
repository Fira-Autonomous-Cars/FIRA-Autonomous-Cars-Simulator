/*********************************************************************
 * C++ unit test for dataspeed_can_usb/ModuleVersion.h
 *********************************************************************/

#include <gtest/gtest.h>

// File under test
#include <dataspeed_can_usb/ModuleVersion.h>
using namespace dataspeed_can_usb;

// Test empty constructor
TEST(ModuleVersion, empty)
{
  EXPECT_EQ(0, ModuleVersion().major());
  EXPECT_EQ(0, ModuleVersion().minor());
  EXPECT_EQ(0, ModuleVersion().build());
  EXPECT_FALSE(ModuleVersion().valid());
}

// Test validity
TEST(ModuleVersion, valid)
{
  // Test zeros and ones
  EXPECT_FALSE(ModuleVersion(0,0,0).valid());
  EXPECT_TRUE(ModuleVersion(1,0,0).valid());
  EXPECT_TRUE(ModuleVersion(0,1,0).valid());
  EXPECT_TRUE(ModuleVersion(0,0,1).valid());

  // Test all valid values (non-zero)
  for (size_t i = 1; i <= UINT16_MAX; i++) {
    EXPECT_TRUE(ModuleVersion(i,0,0).valid()) << i;
    EXPECT_TRUE(ModuleVersion(0,i,0).valid()) << i;
    EXPECT_TRUE(ModuleVersion(0,0,i).valid()) << i;
    EXPECT_TRUE(ModuleVersion(i,i,i).valid()) << i;
  }
}

// Test fields (major,minor,build)
TEST(ModuleVersion, fields)
{
  EXPECT_EQ(1, ModuleVersion(1,2,3).major());
  EXPECT_EQ(2, ModuleVersion(1,2,3).minor());
  EXPECT_EQ(3, ModuleVersion(1,2,3).build());
}

// Test operators
TEST(ModuleVersion, operators)
{
  EXPECT_LT(ModuleVersion(1,0,0), ModuleVersion(2,3,4));
  EXPECT_LT(ModuleVersion(1,9,9), ModuleVersion(2,3,4));
  EXPECT_LE(ModuleVersion(1,9,9), ModuleVersion(2,3,4));
  EXPECT_LE(ModuleVersion(2,3,4), ModuleVersion(2,3,4));
  EXPECT_GT(ModuleVersion(2,3,4), ModuleVersion(1,0,0));
  EXPECT_GT(ModuleVersion(2,3,4), ModuleVersion(1,9,9));
  EXPECT_GE(ModuleVersion(2,3,4), ModuleVersion(1,9,9));
  EXPECT_GE(ModuleVersion(2,3,4), ModuleVersion(2,3,4));
  EXPECT_EQ(ModuleVersion(1,2,3), ModuleVersion(1,2,3));
  EXPECT_NE(ModuleVersion(1,2,3), ModuleVersion(3,2,1));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

