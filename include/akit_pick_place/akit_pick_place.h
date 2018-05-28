#ifndef AKIT_PICK_PLACE_H
#define AKIT_PICK_PLACE_H
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>


#define UP true
#define DOWN false

typedef std::map<std::string, moveit_msgs::CollisionObject> CollisionObjectsMap;
typedef std::map<std::string, moveit_msgs::AttachedCollisionObject> AttachedCollisionObjectsMap;

class akit_pick_place {

private:

  //ros
  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  ros::Subscriber marker_sub;
  ros::ServiceClient planning_scene_diff_client;
  visualization_msgs::Marker marker;   //marker for grasp points
  tf::TransformListener transform_listener;

  //akit stuff
  std::string WORLD_FRAME;
  std::string BASE_LINK;
  std::string GRIPPER_FRAME;
  std::string BUCKET_FRAME;
  std::string PLANNING_GROUP_NAME;
  std::string EEF_PARENT_LINK; //last link in planning group kinematic chain "quickcoupler"
  std::string EEF_GROUP;
  double GRIPPER_JAW_LENGTH; //for cartesian motion
  double GRIPPER_LENGTH;   //for grasp generation
  double GRIPPER_SIDE_LENGTH;

  //cartesian motion
  bool gripperSuccess;
  bool akitSuccess;
  bool setFromGraspGenerator;
  bool side_grasps;

  //akit pick & cartesian stuff
  geometry_msgs::Pose pre_grasp_pose;
  geometry_msgs::Pose pre_place_pose;
  geometry_msgs::Pose grasp_pose;
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  std::vector<geometry_msgs::Pose> grasp_pose_vector;
  std::vector<moveit_msgs::CollisionObject> collision_objects_vector;
  CollisionObjectsMap collision_objects_map;
  AttachedCollisionObjectsMap attached_collision_objects_map;
  static geometry_msgs::Pose interactive_pose;
  static std::string interactive_name;


  //MoveIt! stuff
  moveit::planning_interface::MoveGroupInterface *akitGroup;
  moveit::planning_interface::MoveGroupInterface *gripperGroup;
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
  moveit::planning_interface::MoveGroupInterface::Plan MotionPlan;
  moveit::planning_interface::MoveGroupInterface::Plan gripperMotionPlan;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
  moveit::core::RobotStatePtr gripperState;
  moveit::core::RobotStatePtr akitState;
  moveit_msgs::PlanningScene planningSceneMsg;
  moveit_msgs::ApplyPlanningScene planningSceneSrv;
  std::vector<double> gripperJointPositions;
  std::vector<double> akitJointPositions;
  const robot_state::JointModelGroup *akitJointModelGroup;
  const robot_state::JointModelGroup *gripperJointModelGroup;
  robot_model_loader::RobotModelLoaderPtr robotModelLoader;
  robot_model::RobotModelPtr robotModelPtr;
  planning_scene::PlanningScenePtr planningScenePtr;
  moveit_msgs::PlanningScene planning_scene_msg_;
  moveit_msgs::ApplyPlanningScene planning_scene_srv;
  collision_detection::AllowedCollisionMatrix acm;

  //interactive markers
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  static void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void addInteractiveMarker(geometry_msgs::Pose marker_position, std::string marker_name,shape_msgs::SolidPrimitive shape);
  void displayTrajectory(moveit::planning_interface::MoveGroupInterface::Plan motion_plan_trajectory,
                               geometry_msgs::Pose published_pose_frame, std::string axis_name,
                               rviz_visual_tools::colors color);

public:

  //constructors
  akit_pick_place(std::string planning_group_, std::string eef_group_, std::string world_frame_,
                  std::string base_link_, std::string eef_parent_link_, std::string gripper_frame_,
                  std::string bucket_frame_, double gripper_length_, double gripper_jaw_length_,
                  double gripper_side_length_, bool set_from_grasp_generator_);
  akit_pick_place();
  ~akit_pick_place();

  //methods
  void initialize();
  void setDefaultPlanningGroup();
  void setPlanningGroup(std::string planning_group_);
  void setGripperGroup(std::string eef_group_);
  void setBaseLink(std::string base_link_);
  void setWorldFrame(std::string world_frame_);
  void setGripperFrame(std::string gripper_frame_);
  void setPreGraspPose(geometry_msgs::Pose preGraspPose);
  void setPrePlacePose(geometry_msgs::Pose prePlacePose);
  void setGripperLength(double gripper_length_);
  void setGripperSideLength(double gripper_side_length_);
  void setGripperJawLength(double gripper_jaw_length_);

  std::string getPlanningGroup();
  std::string getGripperGroup();
  std::string getBaseLink();
  std::string getWorldFrame();
  std::string getGripperFrame();
  double getGripperLength();
  double getGripperSideLength();
  double getGripperJawLength();

  bool generateGrasps(geometry_msgs::Pose block_pose_, double block_size_, bool sideGrasps = false, bool visualize = true);
  bool generateGrasps(geometry_msgs::Pose cylinder_pose_, double cylinder_height_, double cylinder_radius_,bool sideGrasps = false, bool visualize = true);
  bool generateGrasps(geometry_msgs::Pose cuboid_pose_, double cuboid_x_, double cuboid_y_, double cuboid_z_,bool sideGrasps = false, bool visualize = true);
  bool visualizeGrasps(std::vector<geometry_msgs::Pose> points, std::string frame);

  //choose best grasp -->later
  //rotate gripper body --> test

  bool rotateGripper();
  bool rotateGripper(moveit_msgs::CollisionObject object_);
  bool openGripper();
  bool closeGripper();
  bool executeCartesianMotion(bool direction, double cartesian_distance, char axis);
  void allowObjectCollision(std::string object_id);
  void allowToolCollision(std::string tool_id);
  void resetAllowedCollisionMatrix(std::string object_id);
  bool pick(moveit_msgs::CollisionObject object_);
  bool place(moveit_msgs::CollisionObject object_);
  bool pick_place(moveit_msgs::CollisionObject object_);
  bool interactive_pick_place(std::vector<geometry_msgs::Pose> place_positions);

  moveit_msgs::CollisionObject addCollisionCylinder(geometry_msgs::Pose cylinder_pose,std::string cylinder_name, double cylinder_height, double cylinder_radius);
  moveit_msgs::CollisionObject addCollisionBlock(geometry_msgs::Pose block_pose,std::string block_name,  double block_size_x, double block_size_y, double block_size_z);
  void addInteractiveMarkers();
  void addGround();

  bool attachTool(std::string tool_id);

};

#endif // AKIT_PICK_PLACE_H
