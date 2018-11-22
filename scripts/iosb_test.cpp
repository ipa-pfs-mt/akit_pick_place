#include <akit_pick_place/akit_pick_place.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "iosb_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  akit_pick_place akit;

  geometry_msgs::Pose working;
  working.position.x = 2.74662;
  working.position.y = -0.0353878;
  working.position.z = 1.6472;
  working.orientation.w = 0.999928;
  working.orientation.x = 9.60226e-07;
  working.orientation.y = 0.0119892;
  working.orientation.z = -8.00853e-05;

  geometry_msgs::Pose not_working1 = working;
  not_working1.position.x = 5.0;

  geometry_msgs::Pose not_working2 = working;
  not_working2.position.x = 9.0;

  std::vector<geometry_msgs::Pose> test_poses;
  test_poses.push_back(not_working1);
  test_poses.push_back(not_working2);
  test_poses.push_back(working);

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 3.0;
  pose.position.y = 0.0;
  pose.position.z = 0.17;

  std::string position = "pregrasp";

  moveit_msgs::CollisionObject cylinder = akit.addCollisionCylinder(pose,"cylinder", 0.5,0.2);
  akit.generateGrasps(pose, 0.5,0.2);

  if(!akit.pick(cylinder)){
    ROS_ERROR("Failed to pick");
    exit(1);
  }

  /*sleep(1.0);

  akit.allowObjectCollision("cylinder");

  sleep(1.0);

  akit.planAndExecute(test_poses, position);

  sleep(1.0);

  akit.executeAxisCartesianMotion(false, 0.25, 'z');

  sleep(1.0);

  akit.openGripper();*/

  /*sleep(10.0);

  akit.executeAxisCartesianMotion(true, 0.35 , 'z');*/

}










