#include <akit_pick_place/akit_pick_place.h>
#include <time.h>
#include <random_numbers/random_numbers.h>

double CUBOID_X = 0.30;
double CUBOID_Y = 0.30;
double CUBOID_Z = 0.70;
double CYLINDER_HEIGHT = 0.70;
double CYLINDER_RADIUS = 0.175;

double fRand(double fMin, double fMax)
{
  srand(time(NULL));
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "orientation_test_2");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  akit_pick_place akit;
  // akit.addGround();

  double quat[4];

  random_numbers::RandomNumberGenerator random_;
  random_.quaternion(quat);

  for (int i = 0; i < 100; ++i)
  {
    // tf::Quaternion q = tf::createQuaternionFromRPY(fRand(0.0,2*M_PI),fRand(0.0,2*M_PI),fRand(0,2*M_PI)); //check more
    // rotation around y-axis
    tf::Quaternion q = tf::createQuaternionFromRPY(
        fRand(0.0, 2 * M_PI), 0.0, fRand(0, 2 * M_PI));  // check more rotation around y-axis

    geometry_msgs::Pose pose;
    pose.position.x = fRand(-2.0, 2.0);
    pose.position.y = fRand(2.5, 3.0);
    pose.position.z = 0.25;

    pose.orientation.w = q[0];
    pose.orientation.x = q[1];
    pose.orientation.y = q[2];
    pose.orientation.z = q[3];

    //    pose.orientation.w = 0.5;
    //    pose.orientation.x = 0.5;
    //    pose.orientation.y = 0.5;
    //    pose.orientation.z = 0.5;

    geometry_msgs::Pose place = pose;
    place.position.x = 2.0;

    moveit_msgs::CollisionObject cuboid = akit.addCollisionBlock(pose, "cuboid", CUBOID_X, CUBOID_Y, CUBOID_Z);
    akit.generateGrasps(pose, CUBOID_X, CUBOID_Y, CUBOID_Z);
    if (!akit.pick(cuboid))
    {
      ROS_ERROR("Failed to pick");
      continue;
    }
    akit.generateGrasps(place, CUBOID_X, CUBOID_Y, CUBOID_Z);
    if (!akit.place(cuboid))
    {
      ROS_ERROR("Failed to place");
      continue;
    }

    tf::Quaternion q1 = tf::createQuaternionFromRPY(
        fRand(0.0, 2 * M_PI), 0.0, fRand(0, 2 * M_PI));  // check more rotation around y-axis
    geometry_msgs::Pose pose1;
    pose1.position.x = fRand(-2.0, 2.0);
    pose1.position.y = fRand(2.5, 3.0);
    pose1.position.z = 0.25;

    pose1.orientation.w = q1[0];
    pose1.orientation.x = q1[1];
    pose1.orientation.y = q1[2];
    pose1.orientation.z = q1[3];

    //    pose1.orientation.w = 0.707;
    //    pose1.orientation.x = 0.707;
    //    pose1.orientation.y = 0.0;
    //    pose1.orientation.z = 0.0;

    geometry_msgs::Pose place1 = pose1;
    place1.position.x = 2.0;

    moveit_msgs::CollisionObject cube = akit.addCollisionBlock(pose1, "cuboid", CUBOID_X, CUBOID_X, CUBOID_X);
    akit.generateGrasps(pose1, CUBOID_X);
    if (!akit.pick(cube))
    {
      ROS_ERROR("Failed to pick");
      continue;
    }
    akit.generateGrasps(place1, CUBOID_X);
    if (!akit.place(cube))
    {
      ROS_ERROR("Failed to place");
      continue;
    }

    tf::Quaternion q2 = tf::createQuaternionFromRPY(
        fRand(0.0, 2 * M_PI), 0.0, fRand(0, 2 * M_PI));  // check more rotation around y-axis

    geometry_msgs::Pose pose2;
    pose2.position.x = fRand(-2.0, 2.0);
    pose2.position.y = fRand(2.5, 3.0);
    pose2.position.z = 0.25;

    pose2.orientation.w = q2[0];
    pose2.orientation.x = q2[1];
    pose2.orientation.y = q2[2];
    pose2.orientation.z = q2[3];

    //    pose2.orientation.w = 0.707;
    //    pose2.orientation.x = 0.707;
    //    pose2.orientation.y = 0.0;
    //    pose2.orientation.z = 0.0;

    geometry_msgs::Pose place2 = pose2;
    place2.position.x = 2.0;

    moveit_msgs::CollisionObject cylinder =
        akit.addCollisionCylinder(pose2, "cuboid", CYLINDER_HEIGHT, CYLINDER_RADIUS);
    akit.generateGrasps(pose2, CYLINDER_HEIGHT, CYLINDER_RADIUS);
    if (!akit.pick(cylinder))
    {
      ROS_ERROR("Failed to pick");
      continue;
    }
    akit.generateGrasps(place2, CYLINDER_HEIGHT, CYLINDER_RADIUS);
    if (!akit.place(cylinder))
    {
      ROS_ERROR("Failed to place");
      continue;
    }
  }
}
