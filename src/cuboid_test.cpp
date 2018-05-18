#include <akit_pick_place/akit_pick_place.h>

double CUBOID_X = 0.35;
double CUBOID_Y = 0.35;
double CUBOID_Z = 0.70;

int main(int argc, char **argv){

  ros::init(argc, argv, "cuboid_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  akit_pick_place akit;

  geometry_msgs::Pose cuboid_pose;
  cuboid_pose.position.x = 3.0;
  cuboid_pose.position.y = 2.0;
  cuboid_pose.position.z = 0.25;
  cuboid_pose.orientation.w = 0.707;
  cuboid_pose.orientation.x = 0.0;
  cuboid_pose.orientation.y = 0.707;
  cuboid_pose.orientation.z = 0.0;

  moveit_msgs::CollisionObject cuboid = akit.addCollisionBlock(cuboid_pose, "cuboid", CUBOID_X,CUBOID_Y,CUBOID_Z);
  akit.generateGrasps(cuboid_pose,CUBOID_X,CUBOID_Y,CUBOID_Z);
  akit.pick(cuboid);
}
