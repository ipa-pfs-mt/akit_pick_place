#include <akit_pick_place/akit_pick_place.h>

double CYLINDER_HEIGHT = 0.70;
double CYLINDER_RADIUS = 0.175;
double BLOCK_SIZE = 0.35;

int main(int argc, char **argv){

  ros::init(argc, argv , "orientation_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  akit_pick_place akit;

  //barrel pick Position
  geometry_msgs::Pose barrelPose;
  barrelPose.position.x = 3.0;
  barrelPose.position.y = -2.0;
  barrelPose.position.z = 0.25;
  barrelPose.orientation.w = 0.889;
  barrelPose.orientation.x = 0.115;
  barrelPose.orientation.y = 0.282;
  barrelPose.orientation.z = 0.342;

  //barrel place position
  geometry_msgs::Pose barrelPlace = barrelPose;
  barrelPlace.position.x = 2.0;

  //block pick position
  geometry_msgs::Pose blockPose;
  blockPose.position.x = 3.0;
  blockPose.position.y = -2.0;
  blockPose.position.z = 0.25;
  blockPose.orientation.w = 0.685;
  blockPose.orientation.x = 0.693;
  blockPose.orientation.y = -0.159;
  blockPose.orientation.z = -0.158;

  //block place position
  geometry_msgs::Pose blockPlace = blockPose;
  blockPlace.position.x = 2.0;

  moveit_msgs::CollisionObject cylinder =  akit.addCollisionCylinder(barrelPose,"cylinder",CYLINDER_HEIGHT,CYLINDER_RADIUS);

  akit.generateGrasps(barrelPose,CYLINDER_HEIGHT,CYLINDER_RADIUS);
  akit.pick(cylinder);
  akit.generateGrasps(barrelPlace, CYLINDER_HEIGHT,CYLINDER_RADIUS);
  akit.place(cylinder);

  /*moveit_msgs::CollisionObject block = akit.addCollisionBlock(blockPose,"block",BLOCK_SIZE,BLOCK_SIZE,BLOCK_SIZE);

  akit.generateGrasps(blockPose,BLOCK_SIZE);
  akit.pick(block);
  akit.generateGrasps(blockPlace,BLOCK_SIZE);
  akit.place(block);*/

}
