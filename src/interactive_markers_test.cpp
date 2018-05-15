#include <akit_pick_place/akit_pick_place.h>

const double BLOCK_SIZE = 0.35;
const double CYLINDER_HEIGHT = 0.35;
const double CYLINDER_RADIUS = 0.175;

int main(int argc, char **argv){

  ros::init(argc, argv, "interactive_markers_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  akit_pick_place akit;
  akit.addGround();

  //create object pose
  geometry_msgs::Pose blockPose;
  blockPose.position.x = 2.0;
  blockPose.position.y = -2.5;
  blockPose.position.z = 0.17;
  blockPose.orientation.x = blockPose.orientation.y = blockPose.orientation.z = 0.0;
  blockPose.orientation.w = 1.0;

  geometry_msgs::Pose placePose = blockPose;
  placePose.position.x = 1.0;

  akit.addCollisionCylinder(blockPose,"cylinder1",CYLINDER_HEIGHT,CYLINDER_RADIUS);
  akit.addCollisionBlock(placePose,"block1",BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE);

  sleep(1.0);
  placePose.position.x= 0.0;
  akit.addCollisionBlock(placePose,"block2" ,BLOCK_SIZE,BLOCK_SIZE, BLOCK_SIZE);

  placePose.position.x = -1.0;
  akit.addCollisionCylinder(placePose,"cylinder2",CYLINDER_HEIGHT,CYLINDER_RADIUS);

  sleep(1.0);

  akit.addInteractiveMarkers();


  return 0;
}
