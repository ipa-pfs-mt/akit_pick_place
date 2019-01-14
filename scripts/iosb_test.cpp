#include <akit_pick_place/akit_pick_place.h>

int main(int argc, char** argv)
{
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

  // std::string position = "pregrasp";

  moveit_msgs::CollisionObject cylinder = akit.addCollisionCylinder(pose, "cylinder", 0.5, 0.2);

  akit.generateGrasps(pose, 0.5, 0.2);

  if (!akit.pick(cylinder))
  {
    ROS_ERROR("Failed to pick");
    exit(1);
  }

  geometry_msgs::Pose place_pose = pose;
  place_pose.position.y = 1.5;

  akit.generateGrasps(place_pose, 0.5, 0.2);

  if (!akit.place(cylinder))
  {
    ROS_ERROR("Failed to place");
    exit(1);
  }

  /*std::vector<double> joint_states(4,0);
  joint_states.push_back(M_PI/2);

  akit.planAndExecuteJointGoals(joint_states);

  sleep(1.0);

  akit.allowObjectCollision("cylinder");

  sleep(1.0);

  akit.planAndExecuteCartesianGoals(test_poses, position);

  sleep(1.0);

  akit.executeAxisCartesianMotion(false, 0.25, 'z');

  sleep(1.0);

  akit.openGripper();

  sleep(1.0);

  akit.executeAxisCartesianMotion(true, 0.25 , 'z');*/
}
