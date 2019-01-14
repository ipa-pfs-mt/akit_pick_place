#ifndef AKIT_PICK_PLACE_H
#define AKIT_PICK_PLACE_H

#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <e1_motion_sequence/SetGoal.h>
#include <e1_motion_sequence/GoToGoal.h>
#include <e1_interface/E1Command.h>
#include <std_srvs/Empty.h>

#define UP true
#define DOWN false

typedef std::map<std::string, moveit_msgs::CollisionObject> CollisionObjectsMap;
typedef std::map<std::string, moveit_msgs::AttachedCollisionObject> AttachedCollisionObjectsMap;

/**
 * @brief The akit_pick_place class contains methods for the A-Kit project including pick and place methods,
 * interactive pick and place methods, collision objects publishing methods, tool attachement and detachments methods.
 */

class akit_pick_place
{
private:
  // ros
  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  ros::Subscriber marker_sub;
  ros::ServiceClient planning_scene_diff_client;
  ros::ServiceClient planning_scene_diff_client_;
  ros::ServiceClient get_planning_scene_client;
  tf::TransformListener transform_listener;

  // akit stuff
  std::string WORLD_FRAME;
  std::string BASE_LINK;
  std::string GRIPPER_FRAME;
  std::string EEF_GROUP;
  std::string BUCKET_FRAME;
  std::string PLANNING_GROUP;
  std::string EEF_PARENT_LINK;
  double GRIPPER_LENGTH;      // for grasp generation
  double GRIPPER_JAW_LENGTH;  // for cartesian motion
  double GRIPPER_SIDE_LENGTH;
  bool FromGraspGenerator;
  bool side_grasps;

  // remove
  bool gripperSuccess;
  bool akitSuccess;

  // akit pick place
  geometry_msgs::Pose pre_grasp_pose;
  geometry_msgs::Pose pre_place_pose;
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  std::vector<geometry_msgs::Pose> grasp_pose_vector;
  CollisionObjectsMap collision_objects_map;
  AttachedCollisionObjectsMap attached_collision_objects_map;
  static geometry_msgs::Pose interactive_pose;
  static std::string interactive_name;

  visualization_msgs::Marker marker;  // marker for grasp points

  // remove
  moveit::planning_interface::MoveGroupInterface* akitGroup;
  moveit::planning_interface::MoveGroupInterface* gripperGroup;
  moveit::planning_interface::PlanningSceneInterface planningSceneInterface;
  moveit::planning_interface::MoveGroupInterface::Plan MotionPlan;
  moveit::planning_interface::MoveGroupInterface::Plan gripperMotionPlan;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
  moveit::core::RobotStatePtr gripperState;
  moveit::core::RobotStatePtr akitState;

  // MoveIt! stuff
  moveit_msgs::PlanningScene planningSceneMsg;
  moveit_msgs::ApplyPlanningScene planningSceneSrv;
  std::vector<double> gripperJointPositions;
  std::vector<double> akitJointPositions;
  const robot_state::JointModelGroup* akitJointModelGroup;
  const robot_state::JointModelGroup* gripperJointModelGroup;
  robot_model_loader::RobotModelLoaderPtr robotModelLoader;
  robot_model::RobotModelPtr robotModelPtr;
  planning_scene::PlanningScenePtr planningScenePtr;
  moveit_msgs::PlanningScene planning_scene_msg_;
  moveit_msgs::ApplyPlanningScene planning_scene_srv;
  collision_detection::AllowedCollisionMatrix acm;

  // IOSB stuff
  ros::Publisher e1_gripper_pub;
  ros::ServiceClient e1_set_goal_client;
  ros::ServiceClient e1_go_to_goal_client;
  ros::Subscriber e1_joint_states_subscriber;
  ros::Publisher e1_trajectory_publisher;
  ros::ServiceClient e1_compute_fk_client;
  ros::ServiceClient e1_cartesian_path_client;
  ros::ServiceClient e1_execute_traj_client;
  ros::ServiceClient e1_clear_traj_client;
  sensor_msgs::JointState e1_joint_states;

  // interactive markers
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  /**
  * @brief feedback function required for interactive marker control, this function is not used by user, passed
  * internally as a pointer.
  * @param feedback interactive marker mouse click feedback.
  * @return stores marker pose and name.
  */
  static void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

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
                         geometry_msgs::Pose axis_pose,
                         std::string axis_name,
                         rviz_visual_tools::colors color);

public:
  // constructors
  /**
   * @brief akit_pick_place default constructor containing all necessary values and frame names for A-Kit
   *  parameters filled in initialization.yaml file
   */
  akit_pick_place();
  ~akit_pick_place();

  // setters
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

  void writeOutputPlanningTime(std::string file_name);
  void writeOutputTrajectoryLength(std::string file_name);
  /**
   * @brief jointStatesCallback callback function to store joint states for trajectory execution services
   * @param joint_states_msg sensor msgs joint states message
   */
  void jointStatesCallback(const sensor_msgs::JointState joint_states_msg);

  // akit methods
  /**
   * @brief generateGrasps Grasp Pose generator for cubes, generates grasp poses for cubes pick and place pipeline
   * @param block_pose_ a geometry_msgs::Pose message corresponding to the pose of the cube
   * @param block_size_ side length of the cube
   * @param sideGrasps bool for side grasp generation. true --> generate side grasps
   * @param visualize bool for visualising grasp poses
   */
  void
  generateGrasps(geometry_msgs::Pose block_pose_, double block_size_, bool sideGrasps = false, bool visualize = true);
  /**
   * @brief generateGrasps Grasp Pose generator for cubes, generates grasp poses for cylinders pick and place pipeline
   * @param cylinder_pose_ a geometry_msgs::Pose message corresponding to the pose of the cylinder
   * @param cylinder_height_ value of cylinder height
   * @param cylinder_radius_ value of cylinder radius
   * @param sideGrasps bool for side grasp generation. true --> generate side grasps
   * @param visualize bool for visualising grasp poses
   */
  void generateGrasps(geometry_msgs::Pose cylinder_pose_,
                      double cylinder_height_,
                      double cylinder_radius_,
                      bool sideGrasps = false,
                      bool visualize = true);
  /**
   * @brief generateGrasps Grasp Pose generator for cubes, generates grasp poses for cuboids pick and place pipeline
   * @param cuboid_pose_ a geometry_msgs::Pose message corresponding to the pose of the cuboid
   * @param cuboid_x_ value of cuboid x side length
   * @param cuboid_y_ value of cuboid y side length
   * @param cuboid_z_ value of cuboid height
   * @param sideGrasps value of cuboid first side length
   * @param visualize value of cuboid first side length
   */
  void generateGrasps(geometry_msgs::Pose cuboid_pose_,
                      double cuboid_x_,
                      double cuboid_y_,
                      double cuboid_z_,
                      bool sideGrasps = false,
                      bool visualize = true);
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
   * @brief planAndExecute motion planning and execution with cartesian goals
   * @param poses geometry_msgs::Pose vector of poses to be planned and executed
   * @param pose string containing name of pose
   * @return true if motion planning succeeds
   */
  bool planAndExecuteCartesianGoals(std::vector<geometry_msgs::Pose> poses, std::string pose);
  /**
   * @brief planAndExecuteJointGoals motion planning and execution with joint state goal
   * @param joint_states joint state values
   * @param add_to_current_joint_states true if joint states vector is added to current, false if not
   * @return
   */
  bool planAndExecuteJointGoals(std::vector<double> joint_states, bool add_to_current_joint_states = true);
  /**
   * @brief allowObjectCollision allows collision of gripper links with object
   * @param object_id string containing objects id
   */
  bool allowObjectCollision(std::string object_id);
  /**
   * @brief allowToolCollision allows collisoin between tool and quickcoupler
   * @param tool_id string containing tool frame id
   */
  bool allowToolCollision(std::string tool_id);
  /**
   * @brief resetAllowedCollisionMatrix resets allowed collision matrix after place routine is finished
   * @param object_id object to be removed from acm
   */
  bool resetAllowedCollisionMatrix(std::string object_id);
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
  * @brief interactive_pick_place is the  pick and place routine using interactive markers
  * @param place_positions vector containing geometry_msgs::Pose messages for place locations
  * @return true if procedure succeeds
  */
  bool interactive_pick_place(std::vector<geometry_msgs::Pose> place_positions);
  /**
   * @brief addInteractiveMarkers uses addInteractiveMarker method to publish interactive markers over all collision
   * objects
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
  moveit_msgs::CollisionObject addCollisionCylinder(geometry_msgs::Pose cylinder_pose,
                                                    std::string cylinder_name,
                                                    double cylinder_height,
                                                    double cylinder_radius);
  /**
   * @brief addCollisionBlock publishes block to planning scene
   * @param block_pose a geometry_msgs::Pose message of block
   * @param block_name a string containing block id
   * @param block_size_x value of block x side length
   * @param block_size_y value of block y side length
   * @param block_size_z value of block z side length
   * @return the object as moveit_msgs::CollisionObject message
   */
  moveit_msgs::CollisionObject addCollisionBlock(geometry_msgs::Pose block_pose,
                                                 std::string block_name,
                                                 double block_size_x,
                                                 double block_size_y,
                                                 double block_size_z);
  CollisionObjectsMap getSceneCollisionObjects();
  AttachedCollisionObjectsMap getAttachedCollisionObjects();
};

#endif  // AKIT_PICK_PLACE_H
