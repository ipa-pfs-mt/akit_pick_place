#include <akit_pick_place/akit_pick_place.h>
#include <random_numbers/random_numbers.h>
#include <time.h>

double CUBOID_X = 0.30;
double CUBOID_Y = 0.30;
double CUBOID_Z = 0.70;
double CYLINDER_HEIGHT = 0.50;
double CYLINDER_RADIUS = 0.175;

double fRand(double fMin, double fMax){
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "collision_experiment_2");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  akit_pick_place akit;
  //akit.setPlannerID("LBKPIECEkConfigDefault");

  geometry_msgs::Pose barrierPose3;
  barrierPose3.position.x = 1.5;
  barrierPose3.position.y = -2.5;
  barrierPose3.position.z = 0.45;
  barrierPose3.orientation.w = 1.0;
  barrierPose3.orientation.x = 0.0;
  barrierPose3.orientation.y = 0.0;
  barrierPose3.orientation.z = 0.0;

  geometry_msgs::Pose barrierPose4 = barrierPose3;
  barrierPose3.position.x = -1.5;

  akit.addCollisionBlock(barrierPose3, "barrier3", 0.1, 2.0, 1.0);
  sleep(1.0);
  akit.addCollisionBlock(barrierPose4, "barrier4", 0.1, 2.0, 1.0);
  sleep(1.0);
  akit.addGround();

  for (int i = 0; i < 100; ++i){
    tf::Quaternion q = tf::createQuaternionFromRPY(fRand(0.0,2*M_PI),0.0,fRand(0,2*M_PI));

    geometry_msgs::Pose pose;
    pose.position.x = fRand(-2.0,2.0);
    pose.position.y = fRand(2.2,3.0);
    pose.position.z = 0.35;
    pose.orientation.w = q[0];
    pose.orientation.x = q[1];
    pose.orientation.y = q[2];
    pose.orientation.z = q[3];

    geometry_msgs::Pose place;
    place.position.x = 0.0;
    place.position.y = -2.5;
    place.position.z = 0.25;
    place.orientation.w = 1.0;
    place.orientation.x = place.orientation.y = place.orientation.z = 0.0;


   moveit_msgs::CollisionObject cylinder = akit.addCollisionCylinder(pose, "cylinder",CYLINDER_HEIGHT,CYLINDER_RADIUS);
    akit.generateGrasps(pose,CYLINDER_HEIGHT,CYLINDER_RADIUS);
    if(!akit.pick(cylinder)){
      ROS_ERROR("Failed to pick");
      continue;
    }
    akit.generateGrasps(place,CYLINDER_HEIGHT,CYLINDER_RADIUS);
    if(!akit.place(cylinder)){
      ROS_ERROR("Failed to place");
      continue;
    }
  }
}
