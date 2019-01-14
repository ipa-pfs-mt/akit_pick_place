#include <akit_pick_place/akit_pick_place.h>

const double CYLINDER_HEIGHT = 0.35;
const double CYLINDER_RADIUS = 0.175;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_experiment_1");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  akit_pick_place akit;
  // akit.setPlannerID("LBKPIECEkConfigDefault");

  double y = -2.7;
  // create 100 objects in simple orientations and record planning time
  for (int i = 0; i < 100; ++i)
  {
    geometry_msgs::Pose cylinderPose;
    cylinderPose.position.x = 2.3;
    cylinderPose.position.y = y;
    cylinderPose.position.z = 0.17;
    cylinderPose.orientation.x = 0.0;
    cylinderPose.orientation.y = 0.0;
    cylinderPose.orientation.z = 0.0;
    cylinderPose.orientation.w = 1.0;

    geometry_msgs::Pose cylinderPlace = cylinderPose;
    cylinderPlace.position.x = -2.3;

    moveit_msgs::CollisionObject cylinder =
        akit.addCollisionCylinder(cylinderPose, "cylinder", CYLINDER_HEIGHT, CYLINDER_RADIUS);
    akit.generateGrasps(cylinderPose, CYLINDER_HEIGHT, CYLINDER_RADIUS);
    if (!akit.pick(cylinder))
    {
      ROS_ERROR("Failed to pick");
      continue;
    }
    akit.generateGrasps(cylinderPlace, CYLINDER_HEIGHT, CYLINDER_RADIUS);
    if (!akit.place(cylinder))
    {
      ROS_ERROR("Failed to place");
      continue;
    }

    y += 0.054;
  }
}
