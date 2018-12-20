#include <akit_pick_place/akit_pick_place.h>
#include <gtest/gtest.h>

TEST(TestName, Subtest_1){

  // Arrange
  int value = 100;
  int increment = 5;

  // Act
  value += increment;

  // Assert
  ASSERT_EQ(value, 105);


}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
