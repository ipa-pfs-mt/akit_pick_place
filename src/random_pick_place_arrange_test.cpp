#include <akit_pick_place/akit_pick_place.h>

static const std::string BASE_LINK = "chassis";
const double CYLINDER_HEIGHT = 0.35;
const double CYLINDER_RADIUS = 0.175;

double fRand(double fMin, double fMax){ //make a new random number generator
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}
void generateRandomBlock(geometry_msgs::Pose& cylinder_pose)
{
  // Position
  cylinder_pose.position.x = fRand(-3.0,3.0);
  cylinder_pose.position.y = fRand(-2.2,-3.2);
  cylinder_pose.position.z = 0.17;

  // Orientation
  double angle = M_PI * fRand(0.1,1);
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  cylinder_pose.orientation.x = quat.x();
  cylinder_pose.orientation.y = quat.y();
  cylinder_pose.orientation.z = quat.z();
  cylinder_pose.orientation.w = quat.w();
}

moveit_msgs::CollisionObject createCollisionCylinder(geometry_msgs::Pose cylinder_pose,
                             std::string cylinder_name, double cylinder_height, double cylinder_radius){
  moveit_msgs::CollisionObject cylinder;
  cylinder.id = cylinder_name;
  cylinder.header.stamp = ros::Time::now();
  cylinder.header.frame_id = BASE_LINK;
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = cylinder_height;
  primitive.dimensions[1] = cylinder_radius;

  cylinder.primitives.push_back(primitive);
  cylinder.primitive_poses.push_back(cylinder_pose);
  cylinder.operation = moveit_msgs::CollisionObject::ADD;

  return cylinder;
}

int main(int argc, char**argv){

  ros::init(argc, argv, "random_pick_place_arrange_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
  akit_pick_place akit;

  //specify place position such that 2 cylinders above each other for 5 rows

  std::vector<geometry_msgs::Pose> place_locations_;
  geometry_msgs::Pose placeLocation1;
  placeLocation1.position.z = 0.17;
  placeLocation1.position.x = 2.0;
  placeLocation1.position.y = 3.0;
  placeLocation1.orientation.x = placeLocation1.orientation.y = placeLocation1.orientation.z = 0.0;
  placeLocation1.orientation.w = 1.0;
  geometry_msgs::Pose placeLocation2 = placeLocation1;
  placeLocation2.position.x = 1.0;
  geometry_msgs::Pose placeLocation3 = placeLocation1;
  placeLocation3.position.x = 0.0;
  geometry_msgs::Pose placeLocation4 = placeLocation1;
  placeLocation4.position.x = -1.0;
  geometry_msgs::Pose placeLocation6 = placeLocation1;
  placeLocation6.position.z = placeLocation1.position.z + CYLINDER_HEIGHT + 0.02;
  ROS_INFO_STREAM(placeLocation6.position.z);
  geometry_msgs::Pose placeLocation7 = placeLocation2;
  placeLocation7.position.z = placeLocation2.position.z + CYLINDER_HEIGHT + 0.02;
  geometry_msgs::Pose placeLocation8 = placeLocation3;
  placeLocation8.position.z = placeLocation3.position.z + CYLINDER_HEIGHT + 0.02;
  geometry_msgs::Pose placeLocation9 = placeLocation4;
  placeLocation9.position.z = placeLocation4.position.z + CYLINDER_HEIGHT + 0.02;

  place_locations_.push_back(placeLocation1);
  place_locations_.push_back(placeLocation2);
  place_locations_.push_back(placeLocation3);
  place_locations_.push_back(placeLocation4);
  place_locations_.push_back(placeLocation6);
  place_locations_.push_back(placeLocation7);
  place_locations_.push_back(placeLocation8);
  place_locations_.push_back(placeLocation9);


  //generate 10 random position cylinders
  std::vector<moveit_msgs::CollisionObject> collisionObjects(1);

  for (int i = 0; i < place_locations_.size(); ++i){
    geometry_msgs::Pose cylinder_pose;
    generateRandomBlock(cylinder_pose);
    moveit_msgs::CollisionObject cylinder = createCollisionCylinder(cylinder_pose, "cylinder" + std::to_string(i),
                                                                    CYLINDER_HEIGHT,CYLINDER_RADIUS);
    collisionObjects[0] = cylinder;
    ROS_INFO_STREAM("Adding Collision object to world");
    planningSceneInterface.addCollisionObjects(collisionObjects);

    akit.generateGrasps(cylinder_pose, CYLINDER_HEIGHT, CYLINDER_RADIUS);
    if(!akit.pick(cylinder.id)){
      ROS_ERROR("Failed to pick");
      continue;
    } else {
       akit.generateGrasps(place_locations_[i], CYLINDER_HEIGHT, CYLINDER_RADIUS);
       if (!akit.place(cylinder.id)){
         ROS_ERROR("Failed to place");
         continue;
       }
    }
 }

 return 0;

}
