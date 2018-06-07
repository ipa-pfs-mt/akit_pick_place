#include <akit_pick_place/akit_pick_place.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "test_scenario");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  akit_pick_place akit;

  geometry_msgs::Pose barrierPose;
  barrierPose.position.x = 3.0;
  barrierPose.position.y = 0.0;
  barrierPose.position.z = 0.45;
  barrierPose.orientation.w = 1.0;
  barrierPose.orientation.x = 0.0;
  barrierPose.orientation.y = 0.0;
  barrierPose.orientation.z = 0.0;

  geometry_msgs::Pose barrierPose2 = barrierPose;
  barrierPose2.position.x = -3.0;

  moveit_msgs::CollisionObject barrier = akit.addCollisionBlock(barrierPose, "barrier", 2.0, 0.05, 1.0);
  sleep(1.0);
  moveit_msgs::CollisionObject barrier2 = akit.addCollisionBlock(barrierPose2, "barrier2", 2.0, 0.05, 1.0);

  geometry_msgs::Pose boxPose = barrierPose;
  boxPose.position.y = -1.0;

  moveit_msgs::CollisionObject box = akit.addCollisionBlock(boxPose, "box", 0.35, 0.35, 0.35);

  geometry_msgs::Pose boxPlace = boxPose;
  boxPlace.position.y = 1.0;

  akit.generateGrasps(boxPose, 0.35);
  akit.pick(box);

  akit.generateGrasps(boxPlace, 0.35);
  akit.place(box);
}
