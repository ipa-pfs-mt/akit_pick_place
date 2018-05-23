#include <akit_pick_place/akit_pick_place.h>
#include <time.h>
#include <random_numbers/random_numbers.h>

double CUBOID_X = 0.30;
double CUBOID_Y = 0.30;
double CUBOID_Z = 0.70;

double fRand(double fMin, double fMax){
  srand(time(NULL));
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "cuboid_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  akit_pick_place akit;
  akit.addGround();

  double quat[4];

  random_numbers::RandomNumberGenerator random_;
  random_.quaternion(quat);

  for (int i = 0; i < 11; ++i){
    tf::Quaternion q = tf::createQuaternionFromRPY(fRand(0.0,2*M_PI),0.0,fRand(0,2*M_PI)); //fix no rotation around pitch

    geometry_msgs::Pose cuboid_pose;
    cuboid_pose.position.x = fRand(-2.0,2.0);
    cuboid_pose.position.y = fRand(2.0,3.0);
    cuboid_pose.position.z = 0.5;
    cuboid_pose.orientation.w = q[0];
    cuboid_pose.orientation.x = q[1];
    cuboid_pose.orientation.y = q[2];
    cuboid_pose.orientation.z = q[3];

    geometry_msgs::Pose cuboid_place = cuboid_pose;
    cuboid_place.position.x = 2.0;

    moveit_msgs::CollisionObject cuboid = akit.addCollisionBlock(cuboid_pose, "cuboid", CUBOID_X,CUBOID_Y,CUBOID_Z);
    akit.generateGrasps(cuboid_pose,CUBOID_X,CUBOID_Y,CUBOID_Z);
    if(!akit.pick(cuboid)){
      ROS_ERROR("Failed to pick");
      continue;
    }
    akit.generateGrasps(cuboid_place,CUBOID_X,CUBOID_Y,CUBOID_Z);
    if(!akit.place(cuboid)){
      ROS_ERROR("Failed to place");
      continue;
    }
  }

}
