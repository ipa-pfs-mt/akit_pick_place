#include <akit_pick_place/akit_pick_place.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

const double BLOCK_SIZE = 0.35;

int main(int argc, char**argv){

  ros::init(argc, argv, "pick_place_test_2");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  akit_pick_place akit;

  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;

  //create collision blocks
  moveit_msgs::CollisionObject Block1;
  Block1.id = "block1";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = BLOCK_SIZE;
  primitive.dimensions[1] = BLOCK_SIZE;
  primitive.dimensions[2] = BLOCK_SIZE;
  Block1.primitives.push_back(primitive);

  moveit_msgs::CollisionObject Block2;
  Block2.id = "block2";
  Block2.primitives.push_back((primitive));

  moveit_msgs::CollisionObject Block3;
  Block3.id = "block3";
  Block3.primitives.push_back((primitive));

  moveit_msgs::CollisionObject Block4;
  Block4.id = "block4";
  Block4.primitives.push_back((primitive));

  moveit_msgs::CollisionObject Block5;
  Block5.id = "block5";
  Block5.primitives.push_back((primitive));

  //create object pose
  geometry_msgs::Pose block1Pose;
  block1Pose.position.x = 2.7;
  block1Pose.position.y = -2.0;
  block1Pose.position.z = 0.17;
  block1Pose.orientation.x = 0;
  block1Pose.orientation.y = 0;
  block1Pose.orientation.z = 0;
  block1Pose.orientation.w = 1;
  Block1.primitive_poses.push_back(block1Pose);

  //create object pose
  geometry_msgs::Pose block2Pose = block1Pose;
  block2Pose.position.y = -1.0;
  Block2.primitive_poses.push_back(block2Pose);

  geometry_msgs::Pose block3Pose = block1Pose;
  block3Pose.position.y = 0.0;
  Block3.primitive_poses.push_back(block3Pose);

  geometry_msgs::Pose block4Pose = block1Pose;
  block4Pose.position.y = 1.0;
  Block4.primitive_poses.push_back(block4Pose);

  geometry_msgs::Pose block5Pose = block1Pose;
  block5Pose.position.y = 2.0;
  Block5.primitive_poses.push_back(block5Pose);

  //adding collision objects to planning scene
  std::vector<moveit_msgs::CollisionObject> collisionObjects;
  collisionObjects.push_back(Block1);
  collisionObjects.push_back(Block2);
  collisionObjects.push_back(Block3);
  collisionObjects.push_back(Block4);
  collisionObjects.push_back(Block5);
  ROS_INFO_STREAM("Adding Collision object to world");

  planningSceneInterface.addCollisionObjects(collisionObjects);
  sleep(1.0);

  /*------------------------------------------------------------------------------------------------------*/
  //create pre-grasp pose from blender --> block1 pick
  geometry_msgs::Pose block1pick;
  block1pick.position.x = 2.51312;
  block1pick.position.y = -1.80605;
  block1pick.position.z = 1.41138;
  block1pick.orientation.x = 0.003;
  block1pick.orientation.y = 0.007;
  block1pick.orientation.z = -0.345;
  block1pick.orientation.w = 0.938;

  //create pre-place pose from blender --> block1 place
  geometry_msgs::Pose block1place;
  block1place.position.x = 1.82176;
  block1place.position.y = 2.27630;
  block1place.position.z = 1.41138;
  block1place.orientation.w = 0.884;
  block1place.orientation.x = -0.004;
  block1place.orientation.y = 0.007;
  block1place.orientation.z = 0.468;

  akit.setPreGraspPose(block1pick);
  akit.setPrePlacePose(block1place);
  akit.pick_place(Block1.id);

  /*------------------------------------------------------------------------------------------------------*/
  //create pre-grasp pose from blender --> block2 pick
  geometry_msgs::Pose block2pick;
  block2pick.position.x = 2.44903;
  block2pick.position.y = -0.89659;
  block2pick.position.z = 1.41138;
  block2pick.orientation.x = 0.001;
  block2pick.orientation.y = 0.007;
  block2pick.orientation.z = -0.20;
  block2pick.orientation.w = 0.980;

  //create pre-place pose from blender --> block2 place
  geometry_msgs::Pose block2place;
  block2place.position.x = 0.89085;
  block2place.position.y = 2.27630;
  block2place.position.z = 1.41138;
  block2place.orientation.w = 0.811;
  block2place.orientation.x = -0.004;
  block2place.orientation.y = 0.006;
  block2place.orientation.z = 0.585;

  akit.setPreGraspPose(block2pick);
  akit.setPrePlacePose(block2place);
  akit.pick_place(Block2.id);

  /*------------------------------------------------------------------------------------------------------*/
  //create pre-grasp pose from blender --> block3 pick
  geometry_msgs::Pose block3pick;
  block3pick.position.x = 2.42319;
  block3pick.position.y = -0.01204;
  block3pick.position.z = 1.41138;
  block3pick.orientation.x = -0.0001221;
  block3pick.orientation.y = 0.007;
  block3pick.orientation.z = 0.016;
  block3pick.orientation.w = 1.0;

  //create pre-place pose from blender --> block3 place
  geometry_msgs::Pose block3place;
  block3place.position.x = -0.01895;
  block3place.position.y = 2.23816;
  block3place.position.z = 1.41138;
  block3place.orientation.w = 0.695;
  block3place.orientation.x = -0.005;
  block3place.orientation.y = 0.005;
  block3place.orientation.z = 0.719;

  akit.setPreGraspPose(block3pick);
  akit.setPrePlacePose(block3place);
  akit.pick_place(Block3.id);

  /*------------------------------------------------------------------------------------------------------*/
  //create pre-grasp pose from blender --> block4 pick
  geometry_msgs::Pose block4pick;
  block4pick.position.x = 2.42319;
  block4pick.position.y = 0.92929;
  block4pick.position.z = 1.41138;
  block4pick.orientation.x = -0.001;
  block4pick.orientation.y = 0.007;
  block4pick.orientation.z = 0.158;
  block4pick.orientation.w = 0.987;

  //create pre-place pose from blender --> block4 place
  geometry_msgs::Pose block4place;
  block4place.position.x = -0.90238;
  block4place.position.y = 2.23816;
  block4place.position.z = 1.41138;
  block4place.orientation.w = 0.544;
  block4place.orientation.x = -0.006;
  block4place.orientation.y = 0.004;
  block4place.orientation.z = 0.839;

  akit.setPreGraspPose(block4pick);
  akit.setPrePlacePose(block4place);
  akit.pick_place(Block4.id);

  /*------------------------------------------------------------------------------------------------------*/
  //create pre-grasp pose from blender --> block5 pick
  geometry_msgs::Pose block5pick;
  block5pick.position.x = 2.47858;
  block5pick.position.y = 1.83254;
  block5pick.position.z = 1.41138;
  block5pick.orientation.x = -0.002;
  block5pick.orientation.y = 0.007;
  block5pick.orientation.z = 0.299;
  block5pick.orientation.w = 0.954;

  //create pre-place pose from blender --> block5 place
  geometry_msgs::Pose block5place;
  block5place.position.x = -1.79002;
  block5place.position.y = 2.23816;
  block5place.position.z = 1.41138;
  block5place.orientation.w = 0.379;
  block5place.orientation.x = -0.007;
  block5place.orientation.y = 0.003;
  block5place.orientation.z = 0.925;

  akit.setPreGraspPose(block5pick);
  akit.setPrePlacePose(block5place);
  akit.pick_place(Block5.id);

  return 0;
}
