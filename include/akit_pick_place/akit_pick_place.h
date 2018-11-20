#ifndef AKIT_PICK_PLACE_H
#define AKIT_PICK_PLACE_H
#include <ros/ros.h>
#include <tf/tf.h>
#include <fstream>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <interactive_markers/interactive_marker_server.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <e1_motion_sequence/SetGoal.h>
#include <e1_motion_sequence/GoToGoal.h>
#include <e1_interface/E1Command.h>


#define UP true
#define DOWN false

typedef std::map<std::string, moveit_msgs::CollisionObject> CollisionObjectsMap;
typedef std::map<std::string, moveit_msgs::AttachedCollisionObject> AttachedCollisionObjectsMap;

/**
 * @brief The akit_pick_place class contains methods for the A-Kit project including pick and place methods,
 * interactive pick and place methods, collision objects publishing methods, tool attachement and detachments methods.
 */

class akit_pick_place {

private:

  //ros
  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  ros::Subscriber marker_sub;
  ros::ServiceClient planning_scene_diff_client;
  ros::ServiceClient planning_scene_diff_client_;
  ros::ServiceClient get_planning_scene_client;
  tf::TransformListener transform_listener;

  //akit stuff
  std::string WORLD_FRAME;
  std::string BASE_LINK;
  std::string GRIPPER_FRAME;
  std::string EEF_GROUP;
  std::string BUCKET_FRAME;
  std::string PLANNING_GROUP_NAME;
  std::string EEF_PARENT_LINK; //last link in planning group kinematic chain "quickcoupler"
  double GRIPPER_LENGTH;   //for grasp generation
  double GRIPPER_JAW_LENGTH; //for cartesian motion
  double GRIPPER_SIDE_LENGTH;
  bool gripperSuccess;
  bool akitSuccess;
  bool FromGraspGenerator;
  bool side_grasps;

  //akit pick place
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

  visualization_msgs::Marker marker;   //marker for grasp points

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

  //IOSB stuff
  e1_motion_sequence::SetGoal e1_set_goal_srv;
  e1_motion_sequence::GoToGoal e1_go_to_srv;
  ros::Publisher e1_gripper_pub;
  ros::ServiceClient e1_set_goal_client;
  ros::ServiceClient e1_go_to_goal_client;

  //interactive markers
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  /**
  * @brief feedback function required for interactive marker control, this function is not used by user, passed internally as a pointer.
  * @param feedback interactive marker mouse click feedback.
  * @return stores marker pose and name.
  */
  static void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
  * @brief Adds interactive marker with the given pose and shape.
  * @param marker_pose a geometry_msgs::Pose message containing the marker pose.
  * @param marker_name a string containing the marker name.
  * @param shape a shape_msgs::SolidPrimitive message specifying the marker's shape.
  */
  void addInteractiveMarker(geometry_msgs::Pose marker_pose, std::string marker_name, shape_msgs::SolidPrimitive shape);
  /**
  * @brief Displays the trajectory in cartesian space and an axis corresponding to first reachable grasp pose.
  * @param motion_plan is the motion plan generated from the planner.
  * @param axis_pose is the pose where the axis is published.
  * @param axis_name is the desired name of the published axis.
  * @param color is the color of the published trajectory.
  */
  void displayTrajectory(moveit::planning_interface::MoveGroupInterface::Plan motion_plan,
                               geometry_msgs::Pose axis_pose, std::string axis_name,
                               rviz_visual_tools::colors color);

public:

  //constructors
  /**
   * @brief akit_pick_place constructor
   * @param planning_group_ string containing planning group name
   * @param eef_group_ string containing end effector planning group
   * @param world_frame_ string specifying world frame name
   * @param base_link_ string specifying base link frame name
   * @param eef_parent_link_ string specifying end effector parent link name
   * @param gripper_frame_ string specifying gripper frame name
   * @param bucket_frame_ string specifying bucket frame name
   * @param gripper_length_ gripper length required for grasp pose calculations
   * @param gripper_jaw_length_ gripper jaw length for approach distance --> cartesian motion function
   * @param gripper_side_length_ gripper side length required for grasp pose calculations
   * @param set_from_grasp_generator_ bool for using grasp poses from the grasp pose generation instead of manually, set to true
   */
  akit_pick_place(std::string planning_group_, std::string eef_group_, std::string world_frame_,
                  std::string base_link_, std::string eef_parent_link_, std::string gripper_frame_,
                  std::string bucket_frame_, double gripper_length_, double gripper_jaw_length_,
                  double gripper_side_length_, bool set_from_grasp_generator_);
  /**
   * @brief akit_pick_place default constructor containing all necessary values and frame names for A-Kit
   */
  akit_pick_place();
  ~akit_pick_place();

  //setters
  /**
   * @brief setDefaultPlanningGroup set default planning group from moveit setup assistant = "e1_complete"
   */
  void setDefaultPlanningGroup();
  /**
   * @brief setPlanningGroup set planning group for motion planning
   * @param planning_group_ is the string containing the planning group name
   */
  void setPlanningGroup(std::string planning_group_);
  /**
   * @brief setGripperGroup set gripper group name
   * @param eef_group_ is the string containing the gripper planning group name
   */
  void setGripperGroup(std::string eef_group_);
  /**
   * @brief setBaseLink set base link
   * @param base_link_ is the string containing the base link frame name
   */
  void setBaseLink(std::string base_link_);
  /**
   * @brief setWorldFrame set world frame
   * @param world_frame_ is the string containing the world frame name
   */
  void setWorldFrame(std::string world_frame_);
  /**
   * @brief setGripperFrame set gripper frame
   * @param gripper_frame_ is the string containing the gripper frame name
   */
  void setGripperFrame(std::string gripper_frame_);
  /**
   * @brief setPreGraspPose for manual grasp pose setting instead of the grasp pose generator
   * @param preGraspPose is the geometry_msgs::Pose message provided by user
   */
  void setPreGraspPose(geometry_msgs::Pose preGraspPose);
  /**
   * @brief setPrePlacePose for manual grasp pose setting instead of the grasp pose generator
   * @param preGraspPose is the geometry_msgs::Pose message provided by user
   */
  void setPrePlacePose(geometry_msgs::Pose prePlacePose);
  /**
   * @brief setGripperLength set gripper length
   * @param gripper_length_ is the value of gripper length
   */
  void setGripperLength(double gripper_length_);
  /**
   * @brief setGripperSideLength set gripper side length
   * @param gripper_length_ is the value of gripper side length
   */
  void setGripperSideLength(double gripper_side_length_);
  /**
   * @brief setGripperJawLength set gripper jaw length
   * @param gripper_length_ is the value of gripper jaw length
   */
  void setGripperJawLength(double gripper_jaw_length_);
  /**
   * @brief setFromGraspGenerator use the grasp pose generator
   * @param grasp_generator true for use grasp pose generator
   */
  void setFromGraspGenerator(bool grasp_generator);
  /**
   * @brief setPlannerID use planner from OMPL
   * @param planner_id_ planner ID from configuration file
   */
  void setPlannerID(std::string planner_id_);

  //getters
  /**
   * @brief getPlanningGroup gets planning group
   * @return string containing planning group
   */
  std::string getPlanningGroup();
  /**
   * @brief getGripperGroup gets gripper group
   * @return string containing gripper group
   */
  std::string getGripperGroup();
  /**
   * @brief getBaseLink gets base link
   * @return string containing base link name
   */
  std::string getBaseLink();
  /**
   * @brief getWorldFrame gets world frame
   * @return string containing world frame name
   */
  std::string getWorldFrame();
  /**
   * @brief getGripperFrame gets gripper frame
   * @return string containing gripper frame name
   */
  std::string getGripperFrame();
  /**
   * @brief getGripperLength gets gripper length
   * @return value of gripper length
   */
  double getGripperLength();
  /**
   * @brief getGripperSideLength gets gripper side length
   * @return value of gripper side length
   */
  double getGripperSideLength();
  /**
   * @brief getGripperJawLength gets gripper jaw length
   * @return value of gripper jaw length
   */
  double getGripperJawLength();
  void addOrientationConstraints();
  void writeOutputPlanningTime(std::string file_name);
  void writeOutputTrajectoryLength(std::string file_name);

  //akit methods
  /**
   * @brief generateGrasps Grasp Pose generator for cubes, generates grasp poses for cubes pick and place pipeline
   * @param block_pose_ a geometry_msgs::Pose message corresponding to the pose of the cube
   * @param block_size_ side length of the cube
   * @param sideGrasps bool for side grasp generation. true --> generate side grasps
   * @param visualize bool for visualising grasp poses
   */
  void generateGrasps(geometry_msgs::Pose block_pose_, double block_size_, bool sideGrasps = false, bool visualize = true);
  /**
   * @brief generateGrasps Grasp Pose generator for cubes, generates grasp poses for cylinders pick and place pipeline
   * @param cylinder_pose_ a geometry_msgs::Pose message corresponding to the pose of the cylinder
   * @param cylinder_height_ value of cylinder height
   * @param cylinder_radius_ value of cylinder radius
   * @param sideGrasps bool for side grasp generation. true --> generate side grasps
   * @param visualize bool for visualising grasp poses
   */
  void generateGrasps(geometry_msgs::Pose cylinder_pose_, double cylinder_height_, double cylinder_radius_,bool sideGrasps = false, bool visualize = true);
  /**
   * @brief generateGrasps Grasp Pose generator for cubes, generates grasp poses for cuboids pick and place pipeline
   * @param cuboid_pose_ a geometry_msgs::Pose message corresponding to the pose of the cuboid
   * @param cuboid_x_ value of cuboid x side length
   * @param cuboid_y_ value of cuboid y side length
   * @param cuboid_z_ value of cuboid height
   * @param sideGrasps value of cuboid first side length
   * @param visualize value of cuboid first side length
   */
  void generateGrasps(geometry_msgs::Pose cuboid_pose_, double cuboid_x_, double cuboid_y_, double cuboid_z_,bool sideGrasps = false, bool visualize = true);
  /**
   * @brief visualizeGrasps publishes visualization messages to RViz
   * @param points a vector containing geometry_msgs::Pose messages to be published
   * @param frame frame of reference for pose publishing
   */
  void visualizeGrasps(std::vector<geometry_msgs::Pose> points, std::string frame);
  /**
   * @brief rotateGripper rotates the gripper rotator joint
   * @param angle_rad angle of rotation
   * @return true if rotation succeeds
   */
  bool rotateGripper(double angle_rad);
  /**
   * @brief rotateGripper adjusts the gripper orientation above the object
   * @param object_ a moveit_msgs::CollisionObject message describing the object to be picked
   * @return true if rotation succeeds
   */
  bool rotateGripper(moveit_msgs::CollisionObject object_);
  /**
   * @brief openGripper open gripper jaws
   * @return true if opening gripper succeeds
   */
  bool openGripper();
  /**
   * @brief closeGripper close gripper jaws according to object side lengths
   * @param object_ a moveit_msgs::CollisionObject message describing the object to be picked
   * @return true if closing gripper succeeds
   */
  bool closeGripper(moveit_msgs::CollisionObject object_);
  /**
   * @brief executeAxisCartesianMotion executes cartesian motion for the parent link of end effector
   * @param direction true for upwards axis motion, false for downwards axis motion
   * @param cartesian_distance value of distance required
   * @param axis 'x'y'z' axis of frame
   * @return true if cartesian motion succeeds
   */
  bool executeAxisCartesianMotion(bool direction, double cartesian_distance, char axis);
  /**
   * @brief planAndExecute motion planning and execution
   * @param poses geometry_msgs::Pose vector of poses to be planned and executed
   * @param pose string containing name of pose
   * @return true if motion planning succeeds
   */
  bool planAndExecute(std::vector<geometry_msgs::Pose> poses, std::string pose, std::string planning_group = "e1_stationary");
  /**
   * @brief allowObjectCollision allows collision of gripper links with object
   * @param object_id string containing objects id
   */
  void allowObjectCollision(std::string object_id);
  /**
   * @brief allowToolCollision allows collisoin between tool and quickcoupler
   * @param tool_id string containing tool frame id
   */
  void allowToolCollision(std::string tool_id);
  /**
   * @brief resetAllowedCollisionMatrix resets allowed collision matrix after place routine is finished
   * @param object_id object to be removed from acm
   */
  void resetAllowedCollisionMatrix(std::string object_id);
  /**
   * @brief attachCollisionObject attaches collision object to gripper frame using planning scene monitor
   * @param collisionObject moveit_msgs collision object to be grasped and attached
   * @return
   */
  bool attachCollisionObject(moveit_msgs::CollisionObject collisionObject);
  /**
   * @brief detachCollisionObject detaches collision object from gripper frame using planning scene monitor
   * @param collisionObject moveit_msgs collision object to be detached from gripper frame
   * @return
   */
  bool detachCollisionObject(moveit_msgs::CollisionObject collisionObject);
  /**
   * @brief pick is pick routine
   * @param object_ a moveit_msgs::CollisionObject message of the object to be picked
   * @return true if procedure succeeds
   */
  bool pick(moveit_msgs::CollisionObject object_);
  /**
   * @brief place is place routine
   * @param object_  a moveit_msgs::CollisionObject message of the object to be placed
   * @return true if procedure succeeds
   */
  bool place(moveit_msgs::CollisionObject object_);
  /**
   * @brief pick_place pick is the place routine
   * @param object_ a moveit_msgs::CollisionObject message of the object to be picked and placed
   * @return true if procedure succeeds
   */
  bool pick_place(moveit_msgs::CollisionObject object_);
  /**
   * @brief interactive_pick_place is the  pick and place routine using interactive markers
   * @param place_positions vector containing geometry_msgs::Pose messages for place locations
   * @return true if procedure succeeds
   */
  bool interactive_pick_place(std::vector<geometry_msgs::Pose> place_positions);
  /**
   * @brief addInteractiveMarkers uses addInteractiveMarker method to publish interactive markers over all collision objects
   */
  void addInteractiveMarkers();
  /**
   * @brief addGround adds simulated ground accounting for position constraints
   */
  void addGround();
  /**
   * @brief attachTool is the tool attachements procedure
   * @param tool_frame_id string containing tool frame id
   * @return true if procedure succeeds
   */
  bool attachTool(std::string tool_frame_id);
  /**
   * @brief detachTool is the tool detachement procedure
   * @param tool_frame_id string containing tool frame id
   * @return true if procedure succeeds
   */
  bool detachTool(std::string tool_frame_id);
  /**
   * @brief addCollisionCylinder publishes cylinder to planning scene
   * @param cylinder_pose a geometry_msgs::Pose message of cylinder pose
   * @param cylinder_name a string containing cylinder id
   * @param cylinder_height value of cylinder height
   * @param cylinder_radius value of cylinder radius
   * @return the object as moveit_msgs::CollisionObject message
   */
  moveit_msgs::CollisionObject addCollisionCylinder(geometry_msgs::Pose cylinder_pose,std::string cylinder_name, double cylinder_height, double cylinder_radius);
  /**
   * @brief addCollisionBlock publishes block to planning scene
   * @param block_pose a geometry_msgs::Pose message of block
   * @param block_name a string containing block id
   * @param block_size_x value of block x side length
   * @param block_size_y value of block y side length
   * @param block_size_z value of block z side length
   * @return the object as moveit_msgs::CollisionObject message
   */
  moveit_msgs::CollisionObject addCollisionBlock(geometry_msgs::Pose block_pose,std::string block_name,  double block_size_x, double block_size_y, double block_size_z);
};

#endif // AKIT_PICK_PLACE_H
