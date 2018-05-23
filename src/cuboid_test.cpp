#include <akit_pick_place/akit_pick_place.h>
#include <time.h>

double CUBOID_X = 0.35;
double CUBOID_Y = 0.35;
double CUBOID_Z = 0.70;



double fRand(double fMin, double fMax){ //make a new random number generator

  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void generateRandomPose(geometry_msgs::Pose& pose_)
{
  srand(time(NULL));
  // Position
  pose_.position.x = fRand(-2.0,2.0);
  pose_.position.y = fRand(2.0,3.0);
  pose_.position.z = 0.25;

  // Orientation
  double angle = M_PI * fRand(0.1,1);
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitX()));
  pose_.orientation.x = quat.x();
  pose_.orientation.y = quat.y();
  pose_.orientation.z = quat.z();
  pose_.orientation.w = quat.w();
}

int main(int argc, char **argv){

  ros::init(argc, argv, "cuboid_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  akit_pick_place akit;

  geometry_msgs::Pose cuboid_pose;
  //generateRandomPose(cuboid_pose);
  cuboid_pose.position.x = -2.0;
  cuboid_pose.position.y = 2.0;
  cuboid_pose.position.z = 0.25;
  cuboid_pose.orientation.w = 0.707;
  cuboid_pose.orientation.x = 0.0;
  cuboid_pose.orientation.y = 0.707;
  cuboid_pose.orientation.z = 0.0;

  geometry_msgs::Pose cuboid_place = cuboid_pose;
  cuboid_place.position.x = 2.5;
  //generateRandomPose(cuboid_place);

  moveit_msgs::CollisionObject cuboid = akit.addCollisionBlock(cuboid_pose, "cuboid", CUBOID_X,CUBOID_Y,CUBOID_Z);
  akit.generateGrasps(cuboid_pose,CUBOID_X,CUBOID_Y,CUBOID_Z);
  if(!akit.pick(cuboid)){
    ROS_ERROR("Failed to pick");
    exit(1);
  }
  akit.generateGrasps(cuboid_place,CUBOID_X,CUBOID_Y,CUBOID_Z);
  if(!akit.place(cuboid)){
    ROS_ERROR("Failed to place");
    exit(1);
  }
}
