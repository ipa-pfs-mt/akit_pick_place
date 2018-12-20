#ifndef AKIT_PICK_PLACE_H
#define AKIT_PICK_PLACE_H
#include <ros/ros.h>
#include <tf/tf.h>
#include <fstream>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
  ros::Publisher pose_pub;
  ros::Subscriber marker_sub;
  ros::ServiceClient planning_scene_diff_client;
  ros::ServiceClient planning_scene_diff_client_;
  ros::ServiceClient get_planning_scene_client;
  tf::TransformListener transform_listener;

  // akit
  std::string WORLD_FRAME;
  std::string BASE_LINK;
  std::string GRIPPER_FRAME;
  std::string EEF_GROUP;
  std::string BUCKET_FRAME;
  std::string PLANNING_GROUP;
  std::string EEF_PARENT_LINK;
  double GRIPPER_LENGTH;
  double GRIPPER_JAW_LENGTH;
  double GRIPPER_SIDE_LENGTH;
  bool gripperSuccess;
  bool akitSuccess;
  bool FromGraspGenerator;
  bool side_grasps = true;

  // akit pick place
  geometry_msgs::Pose pre_grasp_pose;
  geometry_msgs::Pose pre_place_pose;
  geometry_msgs::Pose grasp_pose;
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  std::vector<geometry_msgs::Pose> grasp_pose_vector;
  std::vector<geometry_msgs::PoseStamped> grasps;

  std::vector<moveit_msgs::CollisionObject> collision_objects_vector;
  CollisionObjectsMap collision_objects_map;
  AttachedCollisionObjectsMap attached_collision_objects_map;
  static geometry_msgs::Pose interactive_pose;
  static std::string interactive_name;
  visualization_msgs::Marker marker;  // marker for grasp points

  // MoveIt!
  moveit::planning_interface::MoveGroupInterface* akitGroup;
  moveit::planning_interface::MoveGroupInterface* gripperGroup;
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
  const robot_state::JointModelGroup* akitJointModelGroup;
  const robot_state::JointModelGroup* gripperJointModelGroup;
  robot_model_loader::RobotModelLoaderPtr robotModelLoader;
  robot_model::RobotModelPtr robotModelPtr;
  planning_scene::PlanningScenePtr planningScenePtr;
  moveit_msgs::PlanningScene planning_scene_msg_;
  moveit_msgs::ApplyPlanningScene planning_scene_srv;
  collision_detection::AllowedCollisionMatrix acm;

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
  */
  akit_pick_place();
  ~akit_pick_place();

  void writeOutputPlanningTime(std::string file_name);
  void writeOutputTrajectoryLength(std::string file_name);
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
  bool rotateGripper(std::string object_id);
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
  bool closeGripper(std::string object_id);
  /**
   * @brief executeAxisCartesianMotion executes cartesian motion for the parent link of end effector
   * @param direction true for upwards axis motion, false for downwards axis motion
   * @param cartesian_distance value of distance required
   * @param axis 'x'y'z' axis of frame
   * @return true if cartesian motion succeeds
   */
  bool executeAxisCartesianMotion(bool direction, double cartesian_distance, char axis);
  /**
   * @brief planAndExecuteCartesianGoals motion planning and execution
   * @param poses geometry_msgs::Pose vector of poses to be planned and executed
   * @param pose string containing name of pose
   * @return true if motion planning succeeds
   */
  bool planAndExecuteCartesianGoals(std::vector<geometry_msgs::Pose> poses, std::string pose);
  /**
   * @brief planAndExecuteJointGoals motion planning and execution
   * @param group planning group name
   * @param joint_states joint states
   * @param add_to_current_joint_states
   * @return
   */
  bool planAndExecuteJointGoals(std::string group,
                                std::vector<double> joint_states,
                                bool add_to_current_joint_states = true);
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
  bool attachCollisionObject(std::string object_id);
  /**
     * @brief detachCollisionObject detaches collision object from gripper frame using planning scene monitor
     * @param collisionObject moveit_msgs collision object to be detached from gripper frame
     * @return
     */
  bool detachCollisionObject(std::string object_id);
  /**
   * @brief pick is pick routine
   * @param object_ a moveit_msgs::CollisionObject message of the object to be picked
   * @return true if procedure succeeds
   */
  bool pick(std::string object_idm, bool new_grasp_generation = false);
  /**
   * @brief place is place routine
   * @param object_  a moveit_msgs::CollisionObject message of the object to be placed
   * @return true if procedure succeeds
   */
  bool place(std::string object_id);
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

  void broadcastFrame(geometry_msgs::PoseStamped pose, std::string frame_id);

  tf::Quaternion rotateX(geometry_msgs::PoseStamped pose, double angle);
  tf::Quaternion rotateY(geometry_msgs::PoseStamped pose, double angle);
  tf::Quaternion rotateZ(geometry_msgs::PoseStamped pose, double angle);
  std::vector<geometry_msgs::PoseStamped> generateGrasps(std::string object, bool visualize_grasps = true);
  void transformGrasps(std::vector<geometry_msgs::PoseStamped>& grasps);
  void visualizeGraspPose(std::vector<geometry_msgs::PoseStamped>& grasps);
  void scoreGrasps(std::vector<geometry_msgs::PoseStamped>& grasps);
};

#endif  // AKIT_PICK_PLACE_H
