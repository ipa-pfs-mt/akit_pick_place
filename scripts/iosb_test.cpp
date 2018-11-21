#include <akit_pick_place/akit_pick_place.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "iosb_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  akit_pick_place akit;

  geometry_msgs::Pose working;
  working.position.x = 2.40582;
  working.position.y = -0.88369;
  working.position.z = 1.47956;
  working.orientation.w = 0.984;
  working.orientation.x = 0.0;
  working.orientation.y = 0.0;
  working.orientation.z = -0.176;

  geometry_msgs::Pose not_working1 = working;
  not_working1.position.x = 5.0;

  geometry_msgs::Pose not_working2 = working;
  not_working2.position.x = 9.0;

  std::vector<geometry_msgs::Pose> test_poses;
  test_poses.push_back(not_working1);
  test_poses.push_back(not_working2);
  test_poses.push_back(working);

  std::string position = "pregrasp";
  akit.planAndExecute(test_poses, position);

  sleep(1.0);

  akit.executeAxisCartesianMotion(false, 0.5, 'z');

  //akit.openGripper();
}
