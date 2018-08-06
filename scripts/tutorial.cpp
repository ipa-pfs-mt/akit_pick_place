#include <akit_pick_place/akit_pick_place.h>

const double CYLINDER_HEIGHT = 0.70;
const double CYLINDER_RADIUS = 0.175;
const std::string CYLINDER_NAME = "cylinder";

int main(int argc, char**argv){

  //initialize node
  ros::init(argc, argv, "tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //create akit pick place object
  akit_pick_place akit;

  //create collision object pose
  geometry_msgs::Pose pick_pose;
  pick_pose.position.x = 2.0;
  pick_pose.position.y = 2.0;
  pick_pose.position.z = 0.17;
  pick_pose.orientation.w = 1.0;
  pick_pose.orientation.x = 0.0;
  pick_pose.orientation.y = 0.0;
  pick_pose.orientation.z = 0.0;

  //create place pose
  geometry_msgs::Pose place_pose = pick_pose;
  place_pose.position.x = -2.0;

  //create collision object --> cylinder in world frame
  moveit_msgs::CollisionObject cylinder = akit.addCollisionCylinder(pick_pose,CYLINDER_NAME,CYLINDER_HEIGHT,CYLINDER_RADIUS);

  //generate grasp poses
  akit.generateGrasps(pick_pose, CYLINDER_HEIGHT,CYLINDER_RADIUS);

  //start pick routine
  if(!akit.pick(cylinder)){
    ROS_ERROR("Failed to pick");
    return false;
    exit(1);
  }

  //generate place pose using the grasp generator
  akit.generateGrasps(place_pose,CYLINDER_HEIGHT,CYLINDER_RADIUS);

  //start place routine
  if(!akit.place(cylinder)){
    ROS_ERROR("Failed to place");
    return false;
    exit(1);
  }
 return 0;
}
