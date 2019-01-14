#include <akit_pick_place/akit_pick_place.h>

double cylinder_radius = 0.2;
double cylinder_height = 0.5;
std::string cylinder_name = "cyliner";

double block_x = 0.35;
double block_z = 0.75;
std::string block_name = "block";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "iosb_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  akit_pick_place akit;

 /*
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

  std::string position = "pregrasp";

*/

  geometry_msgs::Pose pose;
  pose.position.x = 2.5;
  pose.position.y = 1.0;
  pose.position.z = 0.175;
  pose.orientation.w = 0.707;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.707;
  pose.orientation.z = 0.0;

  geometry_msgs::Pose place_pose = pose;
  place_pose.position.y = -1.0;

  /*moveit_msgs::CollisionObject cylinder = akit.addCollisionCylinder(pose, cylinder_name, cylinder_height, cylinder_radius);

  akit.generateGrasps(pose, cylinder_height, cylinder_radius);

  if (!akit.pick(cylinder))
  {
    ROS_ERROR("Failed to pick");
    exit(1);
  }

  akit.generateGrasps(place_pose, cylinder_height, cylinder_radius);

  if (!akit.place(cylinder))
  {
    ROS_ERROR("Failed to place");
    exit(1);
  }*/

  moveit_msgs::CollisionObject block = akit.addCollisionBlock(pose, block_name, block_x, block_x, block_z);

  akit.generateGrasps(pose, block_x, block_x, block_z);

  if (!akit.pick(block))
  {
    ROS_ERROR("Failed to pick");
    exit(1);
  }

  akit.generateGrasps(place_pose, block_x, block_x, block_z);

  if (!akit.place(block))
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
