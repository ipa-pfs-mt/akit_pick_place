#include <akit_pick_place/akit_pick_place.h>

const std::string BLOCK_NAME = "block";
const std::string CYLINDER_NAME = "cylinder";
const double BLOCK_SIZE = 0.35;
const double CYLINDER_HEIGHT = 0.35;
const double CYLINDER_RADIUS = 0.175;

int main(int argc, char**argv){

  ros::init(argc, argv, "pick_place_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  akit_pick_place akit;
  //akit.setWorldFrame("world");

  //std::string BASE_LINK = akit.getBaseLink();
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

  //create collision object
  moveit_msgs::CollisionObject Block;
  Block.id = BLOCK_NAME;
  Block.header.frame_id = "odom_combined";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = BLOCK_SIZE;
  primitive.dimensions[1] = BLOCK_SIZE;
  primitive.dimensions[2] = BLOCK_SIZE;

  moveit_msgs::CollisionObject Cylinder;
  Cylinder.id = CYLINDER_NAME;
  Cylinder.header.frame_id = "odom_combined";
  shape_msgs::SolidPrimitive cprimitive;
  cprimitive.type = cprimitive.CYLINDER;
  cprimitive.dimensions.resize(2);
  cprimitive.dimensions[0] = CYLINDER_HEIGHT;
  cprimitive.dimensions[1] = CYLINDER_RADIUS;

  //create object pose
  geometry_msgs::Pose blockPose;
  blockPose.position.x = 2.3;
  blockPose.position.y = 1.836;
  blockPose.position.z = 0.17;
  blockPose.orientation.x = 0.0;
  blockPose.orientation.y = 0.0;
  blockPose.orientation.z = 0.0;
  blockPose.orientation.w = 1.0;

  geometry_msgs::Pose placePose = blockPose;
  placePose.position.x = -2.5;

  Block.primitives.push_back(primitive);
  Block.primitive_poses.push_back(blockPose);

  Cylinder.primitives.push_back(cprimitive);
  Cylinder.primitive_poses.push_back(blockPose);

  std::vector<moveit_msgs::CollisionObject> collisionObjects;
  //collisionObjects.push_back(Block);
  collisionObjects.push_back(Cylinder);
  ROS_INFO_STREAM("Adding Collision object to world");
  planningSceneInterface.addCollisionObjects(collisionObjects);
  sleep(1.0);

  /*akit.generateGrasps(blockPose, BLOCK_SIZE);
  akit.pick(BLOCK_NAME);*/

  akit.generateGrasps(blockPose,CYLINDER_HEIGHT,CYLINDER_RADIUS);
  akit.addOrientationConstraints();
  akit.pick(Cylinder);

  /*akit.generateGrasps(placePose, BLOCK_SIZE);
  akit.place(BLOCK_NAME);*/

  akit.generateGrasps(placePose,CYLINDER_HEIGHT,CYLINDER_RADIUS);
  akit.addOrientationConstraints();
  akit.place(Cylinder);

  return 0;
}
