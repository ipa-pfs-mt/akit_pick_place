#include <akit_pick_place/akit_pick_place.h>

const double CYLINDER_HEIGHT = 0.35;
const double CYLINDER_RADIUS = 0.175;

int main(int argc, char **argv){

  ros::init(argc, argv, "collision_experiment_1");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  akit_pick_place akit;
  //akit.setPlannerID("LBKPIECEkConfigDefault");

  //collision barriers
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

  akit.addCollisionBlock(barrierPose, "barrier", 2.0, 0.1, 1.0);
  sleep(1.0);
  akit.addCollisionBlock(barrierPose2, "barrier2", 2.0, 0.1, 1.0);
  sleep(1.0);
  akit.addCollisionBlock(barrierPose3, "barrier3", 0.1, 2.0, 1.0);
  sleep(1.0);
  akit.addCollisionBlock(barrierPose4, "barrier4", 0.1, 2.0, 1.0);

  double y = -2.7;
  //create 50 objects in simple orientations and record planning time
  for (int i = 0; i < 50; ++i){

    geometry_msgs::Pose cylinderPose;
    cylinderPose.position.x = 2.5;
    cylinderPose.position.y = y;
    cylinderPose.position.z = 0.17;
    cylinderPose.orientation.x = 0.0;
    cylinderPose.orientation.y = 0.0;
    cylinderPose.orientation.z = 0.0;
    cylinderPose.orientation.w = 1.0;

    geometry_msgs::Pose cylinderPlace = cylinderPose;
    cylinderPlace.position.x = -2.5;

    moveit_msgs::CollisionObject cylinder = akit.addCollisionCylinder(cylinderPose,"cylinder",CYLINDER_HEIGHT,CYLINDER_RADIUS);
    akit.generateGrasps(cylinderPose,CYLINDER_HEIGHT,CYLINDER_RADIUS);
    //akit.addOrientationConstraints();
    if(!akit.pick(cylinder)){
      ROS_ERROR("Failed to pick");
      continue;
    }
    akit.generateGrasps(cylinderPlace,CYLINDER_HEIGHT,CYLINDER_RADIUS);
    //akit.addOrientationConstraints();
    if(!akit.place(cylinder)){
      ROS_ERROR("Failed to place");
      continue;
    }

    y+= 0.034;
  }

  double y1 = 2.7;
  //create 50 objects in simple orientations and record planning time
  for (int i = 0; i < 50; ++i){

    geometry_msgs::Pose cylinderPose;
    cylinderPose.position.x = -2.5;
    cylinderPose.position.y = y1;
    cylinderPose.position.z = 0.17;
    cylinderPose.orientation.x = 0.0;
    cylinderPose.orientation.y = 0.0;
    cylinderPose.orientation.z = 0.0;
    cylinderPose.orientation.w = 1.0;

    geometry_msgs::Pose cylinderPlace = cylinderPose;
    cylinderPlace.position.x = 2.5;

    moveit_msgs::CollisionObject cylinder = akit.addCollisionCylinder(cylinderPose,"cylinder",CYLINDER_HEIGHT,CYLINDER_RADIUS);
    akit.generateGrasps(cylinderPose,CYLINDER_HEIGHT,CYLINDER_RADIUS);
    if(!akit.pick(cylinder)){
      ROS_ERROR("Failed to pick");
      continue;
    }
    akit.generateGrasps(cylinderPlace,CYLINDER_HEIGHT,CYLINDER_RADIUS);
    if(!akit.place(cylinder)){
      ROS_ERROR("Failed to place");
      continue;
    }

    y1 -= 0.034;
  }

}
