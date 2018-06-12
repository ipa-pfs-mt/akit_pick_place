#include <akit_pick_place/akit_pick_place.h>

int main(int argc, char **argv){

  ros::init(argc, argv, "collision_experiment_1");
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

  geometry_msgs::Pose barrierPose3;
  barrierPose3.position.x = 0.0;
  barrierPose3.position.y = 3.0;
  barrierPose3.position.z = 0.45;
  barrierPose3.orientation.w = 1.0;
  barrierPose3.orientation.x = 0.0;
  barrierPose3.orientation.y = 0.0;
  barrierPose3.orientation.z = 0.0;

  geometry_msgs::Pose barrierPose4 = barrierPose3;
  barrierPose3.position.y = -3.0;

  moveit_msgs::CollisionObject barrier = akit.addCollisionBlock(barrierPose, "barrier", 2.0, 0.1, 1.0);
  sleep(1.0);
  moveit_msgs::CollisionObject barrier2 = akit.addCollisionBlock(barrierPose2, "barrier2", 2.0, 0.1, 1.0);
  sleep(1.0);
  moveit_msgs::CollisionObject barrier3 = akit.addCollisionBlock(barrierPose3, "barrier3", 0.1, 2.0, 1.0);
  sleep(1.0);
  moveit_msgs::CollisionObject barrier4 = akit.addCollisionBlock(barrierPose4, "barrier4", 0.1, 2.0, 1.0);

}
