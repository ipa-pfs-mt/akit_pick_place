#include <akit_pick_place/akit_pick_place.h>

geometry_msgs::Pose akit_pick_place::interactive_pose;
std::string akit_pick_place::interactive_name;

akit_pick_place::akit_pick_place(){

  //check all parameters are loaded

  if (!nh.hasParam("/planning_group")){
    ROS_ERROR_STREAM("planning_group parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/planning_group", PLANNING_GROUP);

  if (!nh.hasParam("/world_frame")){
    ROS_ERROR_STREAM("world_frame parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/world_frame", WORLD_FRAME);

  if (!nh.hasParam("/eef_group")){
    ROS_ERROR_STREAM("eef_group parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/eef_group", EEF_GROUP);

  if (!nh.hasParam("/base_link")){
    ROS_ERROR_STREAM("base_link parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/base_link", BASE_LINK);

  if (!nh.hasParam("/eef_parent_link")){
    ROS_ERROR_STREAM("eef_parent_link parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/eef_parent_link", EEF_PARENT_LINK);

  if (!nh.hasParam("/gripper_frame")){
    ROS_ERROR_STREAM("gripper_frame parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/gripper_frame", GRIPPER_FRAME);

  if (!nh.hasParam("/bucket_frame")){
    ROS_ERROR_STREAM("bucket_frame parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/bucket_frame", BUCKET_FRAME);

  if (!nh.hasParam("/gripper_length")){
    ROS_ERROR_STREAM("gripper_length parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/gripper_length", GRIPPER_LENGTH);

  if (!nh.hasParam("/gripper_jaw_length")){
    ROS_ERROR_STREAM("gripper_jaw_length parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/gripper_jaw_length", GRIPPER_JAW_LENGTH);

  if (!nh.hasParam("/gripper_side_length")){
    ROS_ERROR_STREAM("gripper_side_length parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/gripper_side_length", GRIPPER_SIDE_LENGTH);

  if (!nh.hasParam("/side_grasps")){
    ROS_ERROR_STREAM("side_grasps parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/side_grasps", side_grasps);

  if (!nh.hasParam("/from_grasp_generator")){
    ROS_ERROR_STREAM("from_grasp_generator parameter not loaded, did you load initialization data yaml file ?");
    exit(1);
  }

  nh.getParam("/from_grasp_generator", FromGraspGenerator);

  waypoints = std::vector<geometry_msgs::Pose>(1); //not needed remove later

  robotModelLoader.reset(new robot_model_loader::RobotModelLoader("/e1/moveit_ik/robot_description"));

  robotModelPtr = robotModelLoader->getModel();

  planningScenePtr.reset(new planning_scene::PlanningScene(robotModelPtr));

  server.reset(new interactive_markers::InteractiveMarkerServer("akit_pick_place","",false));

  visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("chassis", "visualization_marker"));

  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",10);

  e1_gripper_pub = nh.advertise<e1_interface::E1Command>("/e1/e1_command", 10);

  e1_trajectory_publisher = nh.advertise<moveit_msgs::MoveGroupActionResult>("/e1/moveit_ik/move_group/result", 10);

  planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("/e1/moveit_ik/apply_planning_scene");

  planning_scene_diff_client_ = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("/e1/moveit_ik/apply_planning_scene");

  get_planning_scene_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("/e1/moveit_ik/get_planning_scene");

  e1_set_goal_client = nh.serviceClient<e1_motion_sequence::SetGoal>("/e1_motion_sequence/add_goal");

  e1_go_to_goal_client = nh.serviceClient<e1_motion_sequence::GoToGoal>("/e1_motion_sequence/go_to_goal");

  e1_compute_fk_client = nh.serviceClient<moveit_msgs::GetPositionFK>("/e1_moveit_interface/compute_fk");

  e1_cartesian_path_client = nh.serviceClient<moveit_msgs::GetCartesianPath>("/e1/moveit_ik/compute_cartesian_path");

  e1_execute_traj_client = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>("/e1_moveit_interface/execute_kinematic_path");

  e1_joint_states_subscriber = nh.subscribe("/e1_interface/joint_states", 1000, &akit_pick_place::jointStatesCallback, this);
}

//destructor
akit_pick_place::~akit_pick_place(){
}

void akit_pick_place::setPreGraspPose(geometry_msgs::Pose preGraspPose){
  pre_grasp_pose = preGraspPose;
}
void akit_pick_place::setPrePlacePose(geometry_msgs::Pose prePlacePose){
  pre_place_pose = prePlacePose;
}

void akit_pick_place::addOrientationConstraints(){ //remove after testing
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "quickcoupler";
  ocm.header.frame_id = "chassis";
  ocm.orientation.w = 1.0;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.absolute_x_axis_tolerance = 1.0;
  ocm.absolute_y_axis_tolerance = 1.0;
  ocm.absolute_z_axis_tolerance = 1.7;
  ocm.weight = 1.0;
  //Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  akitGroup->setPathConstraints(test_constraints);
}

void akit_pick_place::writeOutputPlanningTime(std::string file_name){
  ROS_INFO_STREAM("Planning time = " << MotionPlan.planning_time_);

  std::ofstream fout;
  fout.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);
  fout << MotionPlan.planning_time_ << std::endl;
  fout.close();
}

void akit_pick_place::writeOutputTrajectoryLength(std::string file_name){

  std::ofstream fout;
  fout.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);

  robot_trajectory::RobotTrajectory traj_obj(akitGroup->getRobotModel(), akitGroup->getName());
  moveit::core::RobotState ref_state_obj(akitGroup->getRobotModel());
  traj_obj.setRobotTrajectoryMsg(ref_state_obj, MotionPlan.trajectory_);

  const std::string tool_link = akitGroup->getLinkNames().back();
  auto euclidean_distance = [&tool_link](const robot_state::RobotState& state1,const robot_state::RobotState& state2){
  auto tool_pose1 = state1.getGlobalLinkTransform(tool_link);
  auto tool_pose2 = state2.getGlobalLinkTransform(tool_link);
  return (tool_pose1.translation() - tool_pose2.translation()).norm();
  };

  double traj_length_cartesian_space = 0.0000;
  for (size_t p = 1; p < traj_obj.getWayPointCount(); ++p)
  {
    traj_length_cartesian_space += euclidean_distance(traj_obj.getWayPoint(p), traj_obj.getWayPoint(p-1));
  }
  fout << traj_length_cartesian_space << std::endl;
  ROS_INFO_STREAM("trajectory_length = " << traj_length_cartesian_space);
  fout.close();
}

void akit_pick_place::displayTrajectory(moveit::planning_interface::MoveGroupInterface::Plan motion_plan,
                                        geometry_msgs::Pose axis_pose, std::string axis_name,
                                        rviz_visual_tools::colors color){

  visual_tools->publishAxisLabeled(axis_pose, axis_name , rviz_visual_tools::scales::LARGE);
  visual_tools->publishTrajectoryLine(motion_plan.trajectory_, akitJointModelGroup,color);
  visual_tools->trigger();
}

void akit_pick_place::generateGrasps(geometry_msgs::Pose block_pose_, double block_size_,bool sideGrasps, bool visualize){

  geometry_msgs::Pose grasp_pose;
  geometry_msgs::PoseStamped box_in_chassis_frame,box_in_world_frame;

  box_in_world_frame.pose = block_pose_;
  box_in_world_frame.header.frame_id = WORLD_FRAME;

  //transform object pose from world frame to chassis frame
  transform_listener.waitForTransform(BASE_LINK,WORLD_FRAME, ros::Time::now(), ros::Duration(0.1)); //avoid time difference exception
  transform_listener.transformPose(BASE_LINK,ros::Time(0), box_in_world_frame, WORLD_FRAME, box_in_chassis_frame);

  //create yaw angle (rotation around z-axis)
  double yaw = atan2(box_in_chassis_frame.pose.position.y,box_in_chassis_frame.pose.position.x);

  if (!sideGrasps){

    //calculate length between base frame origin to object frame
    double line_length = sqrt(pow(box_in_chassis_frame.pose.position.x,2)+pow(box_in_chassis_frame.pose.position.y,2));
    double number_of_steps = 10.0;

    //grasp distance covered = length of block hypotenuse + 2*gripper side length
    double blockHypotenuse = sqrt(pow(block_size_,2)+pow(block_size_,2));
    double covered_distance = blockHypotenuse + (2 * GRIPPER_SIDE_LENGTH);
    double starting_point = line_length - (blockHypotenuse/2) - GRIPPER_SIDE_LENGTH;
    double step_size = covered_distance / number_of_steps;

    tf::Quaternion q = tf::createQuaternionFromRPY(0.0,0.0,yaw); //fix rotation to be only around z-axis
    for (double i = step_size; i <= covered_distance; i += step_size){
      grasp_pose.position.x = (starting_point + i) * cos(yaw);
      grasp_pose.position.y = (starting_point + i) * sin(yaw);
      grasp_pose.position.z = GRIPPER_LENGTH + box_in_chassis_frame.pose.position.z + (block_size_/2); //divide by 2 --> center of gravity of cube
      grasp_pose.orientation.x = q[0];
      grasp_pose.orientation.y = q[1];
      grasp_pose.orientation.z = q[2];
      grasp_pose.orientation.w = q[3];
      grasp_pose_vector.push_back(grasp_pose);
    }

    //visualization of grasp points
    if(visualize){
      this->visualizeGrasps(grasp_pose_vector, BASE_LINK);
    }

  } else { //side grasps

    side_grasps = true;
    double blockHypotenuse = sqrt(pow(block_size_,2)+pow(block_size_,2)+pow(block_size_,2) ); //internal diagonal of cube
    double pitch_min =  - M_PI / 3; //60 deg
    double pitch_max = - M_PI / 9;  //20 deg
    double angle_incr= M_PI / 90;   // step --> 2 deg --> 20 steps

    double pos_pitch = M_PI / 9;    //20 deg
    double roll = 0.0;

    //circle in xz plane tilted around z-axis ,0.05 to avoid collision
    double radius = GRIPPER_LENGTH + (blockHypotenuse/2) + 0.05;

    //start grasp generation
    for (double pitch = pitch_min; pitch <= pitch_max; pitch += angle_incr, pos_pitch += angle_incr){

      tf::Quaternion q = tf::createQuaternionFromRPY(roll,pitch,yaw);
      grasp_pose.orientation.x = q[0];
      grasp_pose.orientation.y = q[1];
      grasp_pose.orientation.z = q[2];
      grasp_pose.orientation.w = q[3];
      //semi-circle in xz-plane with a tilt around z-axis (rotation matrix)
      grasp_pose.position.x = box_in_chassis_frame.pose.position.x - radius * cos(pos_pitch) * cos(yaw);
      grasp_pose.position.y = box_in_chassis_frame.pose.position.y -  radius * cos(pos_pitch) * sin(yaw);
      grasp_pose.position.z = box_in_chassis_frame.pose.position.z  + radius * sin(pos_pitch);
      grasp_pose_vector.push_back(grasp_pose);
    }
    if(visualize){
      this->visualizeGrasps(grasp_pose_vector, BASE_LINK);
    }
  }
}

void akit_pick_place::generateGrasps(geometry_msgs::Pose cuboid_pose_, double cuboid_x_, double cuboid_y_, double cuboid_z_, bool sideGrasps, bool visualize){

  geometry_msgs::Pose grasp_pose;

  //calculate roll,pitch,yaw of the object relative to the world frame for test variable (top grasping)
  tf::Quaternion qq(cuboid_pose_.orientation.x, cuboid_pose_.orientation.y,cuboid_pose_.orientation.z, cuboid_pose_.orientation.w);
  tf::Matrix3x3 m(qq);
  double roll_, pitch_, yaw_;
  m.getRPY(roll_, pitch_, yaw_);

  //testing if the orientation of the object is greater or lower than 45deg
  double test = sin(M_PI/2 - roll_) * sin(M_PI/2 - pitch_);

  //transformation to base_link (chassis frame)
  geometry_msgs::PoseStamped cuboid_in_chassis_frame,cuboid_in_world_frame;

  cuboid_in_world_frame.pose = cuboid_pose_;
  cuboid_in_world_frame.header.frame_id = WORLD_FRAME;

  //transform object pose from world frame to chassis frame
  transform_listener.waitForTransform(BASE_LINK,WORLD_FRAME, ros::Time::now(), ros::Duration(0.1)); //avoid time difference exception
  transform_listener.transformPose(BASE_LINK,ros::Time(0), cuboid_in_world_frame, WORLD_FRAME, cuboid_in_chassis_frame);

  //create yaw angle (rotation around z-axis) of Grasp points
  double yaw = atan2(cuboid_in_chassis_frame.pose.position.y,cuboid_in_chassis_frame.pose.position.x);
  double cuboidDiagonal = sqrt(pow(cuboid_x_,2)+pow(cuboid_z_,2));
  double cuboidHypotenuse = sqrt(pow(cuboid_x_,2)+pow(cuboid_y_,2));

  if(!sideGrasps){

    //calculate length between base frame origin to object frame
    double line_length = sqrt(pow(cuboid_in_chassis_frame.pose.position.x,2)+pow(cuboid_in_chassis_frame.pose.position.y,2));
    double number_of_steps = 10.0;

    //grasp distance covered = length of cuboid hypotenuse + 2*gripper_side_length
    double covered_distance = cuboidHypotenuse + (2 * GRIPPER_SIDE_LENGTH);
    double starting_point = line_length - (cuboidHypotenuse/2) - GRIPPER_SIDE_LENGTH;
    double step_size = covered_distance / number_of_steps;

    //grasp_pose_vector = std::vector<geometry_msgs::Pose>(number_of_steps); //initialize
    tf::Quaternion q = tf::createQuaternionFromRPY(0.0,0.0,yaw); //rotation to be only around z-axis
    for (double i = step_size; i <= covered_distance; i += step_size){

      /*if the object's orientation in roll or pitch is between -45deg and 45deg
       *then the added distance is the cuboidDiagonal/2
       *if the orientation is outside this range then added distance is the cuboidHypotenuse/2 */

      if(std::abs(test) >= sin(M_PI/4)){
        grasp_pose.position.z = GRIPPER_LENGTH + cuboid_in_chassis_frame.pose.position.z + (cuboidDiagonal/2);
      } else if (std::abs(test) <= sin(M_PI/4)){
        grasp_pose.position.z = GRIPPER_LENGTH + cuboid_in_chassis_frame.pose.position.z + (cuboidHypotenuse/2);
      }
      grasp_pose.position.x = (starting_point + i) * cos(yaw);
      grasp_pose.position.y = (starting_point + i) * sin(yaw);
      grasp_pose.orientation.x = q[0];
      grasp_pose.orientation.y = q[1];
      grasp_pose.orientation.z = q[2];
      grasp_pose.orientation.w = q[3];
      grasp_pose_vector.push_back(grasp_pose);
    }

    if(visualize){
      this->visualizeGrasps(grasp_pose_vector, BASE_LINK);
    }

 } else { //side grasps

    side_grasps = true;
    double pitch_min = - M_PI / 3;  //60 deg
    double pitch_max = - M_PI / 9;  //20 deg
    double angle_incr= M_PI / 90;   // step --> 2 deg --> 20 steps

    double pos_pitch = M_PI / 9;    //20 deg
    double roll = 0.0; //fix rotation around x-axis to zero

    //make a circle in xz plane tilted around z-axis, 0.05 to avoid collision
    double radius = 0.0;
    if(std::abs(test) >= sin(M_PI/4)){
      radius = GRIPPER_LENGTH + (cuboidDiagonal/2) + 0.05;
    } else if (std::abs(test) <= sin(M_PI/4)){
      radius = GRIPPER_LENGTH + (cuboidHypotenuse/2) + 0.05;
    }
     //start grasp generation
    for (double pitch = pitch_min; pitch <= pitch_max; pitch += angle_incr, pos_pitch += angle_incr){

      tf::Quaternion q = tf::createQuaternionFromRPY(roll,pitch,yaw);
      grasp_pose.orientation.x = q[0];
      grasp_pose.orientation.y = q[1];
      grasp_pose.orientation.z = q[2];
      grasp_pose.orientation.w = q[3];
      //semi-circle in xz-plane with a tilt around z-axis (rotation matrix)
      grasp_pose.position.x = cuboid_in_chassis_frame.pose.position.x - radius * cos(pos_pitch) * cos(yaw);
      grasp_pose.position.y = cuboid_in_chassis_frame.pose.position.y -  radius * cos(pos_pitch) * sin(yaw);
      grasp_pose.position.z = cuboid_in_chassis_frame.pose.position.z + radius * sin(pos_pitch);
      grasp_pose_vector.push_back(grasp_pose);
    }
    if(visualize){
      this->visualizeGrasps(grasp_pose_vector, BASE_LINK);
    }
  }
}

void akit_pick_place::generateGrasps(geometry_msgs::Pose cylinder_pose_, double cylinder_height_, double cylinder_radius_,bool sideGrasps, bool visualize){

  geometry_msgs::Pose grasp_pose;

  //calculate roll,pitch,yaw of the object relative to the world frame for test variable (top grasping)
  tf::Quaternion qq(cylinder_pose_.orientation.x, cylinder_pose_.orientation.y,cylinder_pose_.orientation.z, cylinder_pose_.orientation.w);
  tf::Matrix3x3 m(qq);
  double roll_, pitch_, yaw_;
  m.getRPY(roll_, pitch_, yaw_);

  ROS_INFO_STREAM("roll = " << roll_ << " pitch = " << pitch_ << "yaw = " << yaw_);

  //testing if the orientation of the object (in x,y) is greater or lower than 45deg
  double test = sin(M_PI/2 - roll_) * sin(M_PI/2 - pitch_);

  //transformation to base_link (chassis frame)
  geometry_msgs::PoseStamped cylinder_in_chassis_frame,cylinder_in_world_frame;

  cylinder_in_world_frame.pose = cylinder_pose_;
  cylinder_in_world_frame.header.frame_id = WORLD_FRAME;

  //transform object pose from world frame to chassis frame
  transform_listener.waitForTransform(BASE_LINK,WORLD_FRAME, ros::Time::now(), ros::Duration(0.1)); //avoid time difference exception
  transform_listener.transformPose(BASE_LINK,ros::Time(0), cylinder_in_world_frame, WORLD_FRAME, cylinder_in_chassis_frame);

  //create yaw angle (rotation around z-axis) of grasp points
  double yaw = atan2(cylinder_in_chassis_frame.pose.position.y,cylinder_in_chassis_frame.pose.position.x);
  double cylinderInternalDiagonal = sqrt(pow(2 * cylinder_radius_,2)+pow(cylinder_height_,2));

  if (!sideGrasps){

    //calculate length between base frame origin to object frame
    double line_length = sqrt(pow(cylinder_in_chassis_frame.pose.position.x,2)+pow(cylinder_in_chassis_frame.pose.position.y,2));
    double number_of_steps = 10.0;

    //grasp distance covered = diameter + 2*gripper side length
    double cylinderDiameter = 2 * cylinder_radius_;
    double covered_distance = cylinderDiameter + (2 * GRIPPER_SIDE_LENGTH);
    double starting_point = line_length - cylinder_radius_ - GRIPPER_SIDE_LENGTH;
    double step_size = covered_distance / number_of_steps;

    tf::Quaternion q = tf::createQuaternionFromRPY(0.0,0.0,yaw); //fix rotation to be only around z-axis
    for (double i = step_size; i <= covered_distance; i += step_size){

      /*if the object's orientation in world frame --> roll or pitch is between -45deg and 45deg
       *then the added distance is the cylinderInternalDiagonal/2
       *if the orientation is outside this range then added distance is the cylinder radius*/

      if(std::abs(test) >= sin(M_PI/4)){
        grasp_pose.position.z = GRIPPER_LENGTH + cylinder_in_chassis_frame.pose.position.z + (cylinderInternalDiagonal/2);
      } else if (std::abs(test) <= sin(M_PI/4)){
        grasp_pose.position.z = GRIPPER_LENGTH + cylinder_in_chassis_frame.pose.position.z + (cylinder_radius_);
      }
      grasp_pose.position.x = (starting_point + i) * cos(yaw);
      grasp_pose.position.y = (starting_point + i) * sin(yaw);
      grasp_pose.orientation.x = q[0];
      grasp_pose.orientation.y = q[1];
      grasp_pose.orientation.z = q[2];
      grasp_pose.orientation.w = q[3];
      grasp_pose_vector.push_back(grasp_pose);
    }

    if(visualize){
      this->visualizeGrasps(grasp_pose_vector, BASE_LINK);
    }

  } else { //side grasps

    side_grasps = true;
    double pitch_min = - M_PI / 3;  //60 deg
    double pitch_max = - M_PI / 9;  //20 deg
    double angle_incr= M_PI / 90;   // step --> 2 deg --> 20 steps

    double pos_pitch = M_PI / 9;    //20 deg
    double roll = 0.0; //fix rotation around x-axis to zero

    //make a circle in xz plane tilted around z-axis, tolerance 0.05 to avoid collision
    double radius = 0.0;
    if(std::abs(test) >= sin(M_PI/4)){
      radius = GRIPPER_LENGTH + (cylinderInternalDiagonal/2) + 0.05;
    } else if (std::abs(test) <= sin(M_PI/4)){
      radius = GRIPPER_LENGTH + cylinder_radius_ + 0.05;
    }
    //start grasp generation
    for (double pitch = pitch_min; pitch <= pitch_max; pitch += angle_incr, pos_pitch += angle_incr){

      tf::Quaternion q = tf::createQuaternionFromRPY(roll,pitch,yaw);
      grasp_pose.orientation.x = q[0];
      grasp_pose.orientation.y = q[1];
      grasp_pose.orientation.z = q[2];
      grasp_pose.orientation.w = q[3];
      //semi-circle in xz-plane with a tilt around z-axis (rotation matrix)
      grasp_pose.position.x = cylinder_in_chassis_frame.pose.position.x - radius * cos(pos_pitch) * cos(yaw);
      grasp_pose.position.y = cylinder_in_chassis_frame.pose.position.y -  radius * cos(pos_pitch) * sin(yaw);
      grasp_pose.position.z = cylinder_in_chassis_frame.pose.position.z + radius * sin(pos_pitch);
      grasp_pose_vector.push_back(grasp_pose);
    }
    if(visualize){
      this->visualizeGrasps(grasp_pose_vector, BASE_LINK);
    }
  }
}

void akit_pick_place::visualizeGrasps(std::vector<geometry_msgs::Pose> points, std::string frame){

  ROS_INFO_STREAM("---------- Points Visualization ----------");
  uint32_t shape = visualization_msgs::Marker::ARROW;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.type = shape;
  marker.scale.x = 0.15;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  for (int i = 0; i < points.size(); ++i){
    marker.id = i;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = points[i];
    while (marker_pub.getNumSubscribers() < 1)
    {
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
  }
}
bool akit_pick_place::rotateGripper(double angle_rad){

  //update start state to current state
  gripperState = gripperGroup->getCurrentState();
  gripperGroup->setStartState(*gripperState);

  gripperJointPositions[0] += angle_rad; //gripper rotator joint
  gripperGroup->setJointValueTarget(gripperJointPositions);

  //plan and execute motion plan
  gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (gripperSuccess){
    gripperGroup->execute(gripperMotionPlan);
    ROS_INFO_STREAM("Gripper Motion Plan: " << (gripperSuccess ? "Rotated gripper" : "FAILED TO ROTATE GRIPPER"));
  }

  return (gripperSuccess ? true : false);
}

bool akit_pick_place::rotateGripper(moveit_msgs::CollisionObject object_){ //needs adjusting !!

  //update start state to current state
  gripperState = gripperGroup->getCurrentState();
  gripperGroup->setStartState(*gripperState);

  geometry_msgs::PoseStamped object_in_world_frame, object_in_gripper_frame;
  object_in_world_frame.pose = object_.primitive_poses[0];
  object_in_world_frame.header.frame_id = object_.header.frame_id;

  //transform object from world frame to gripper rotator frame, wait to avoid time difference exceptions
  transform_listener.waitForTransform(GRIPPER_FRAME, WORLD_FRAME, ros::Time::now(), ros::Duration(0.1));
  transform_listener.transformPose(GRIPPER_FRAME,ros::Time(0), object_in_world_frame, WORLD_FRAME, object_in_gripper_frame);

  //get roll, pitch, yaw between object frame and gripper frame
  tf::Quaternion qq(object_in_gripper_frame.pose.orientation.x, object_in_gripper_frame.pose.orientation.y,
                    object_in_gripper_frame.pose.orientation.z, object_in_gripper_frame.pose.orientation.w);

  //get rotation matrix from quaternion
  tf::Matrix3x3 m(qq);

  //get roll,pitch,yaw from rotation matrix
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //account for angles in different quadrants
  if (yaw <= 0.0){
    gripperJointPositions[0] = (M_PI/2) + yaw;
  } else {
    gripperJointPositions[0] =  yaw - (M_PI/2);
  }

  gripperGroup->setJointValueTarget(gripperJointPositions);

  //plan and execute motion plan
  gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (gripperSuccess){
    gripperGroup->execute(gripperMotionPlan);
    ROS_INFO_STREAM("Gripper Motion Plan: " << (gripperSuccess ? "Rotated gripper" : "FAILED TO ROTATE GRIPPER"));
  }

  return (gripperSuccess ? true : false);
}

bool akit_pick_place::openGripper(){

  /*double gripper_open_angle = M_PI/3; //60 deg

  //update start state to current state
  gripperState = gripperGroup->getCurrentState();
  gripperGroup->setStartState(*gripperState);

  gripperJointPositions[1] = gripper_open_angle;
  gripperJointPositions[2] = gripper_open_angle;
  gripperGroup->setJointValueTarget(gripperJointPositions);

  //plan and execute motion plan
  gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (gripperSuccess){
    gripperGroup->execute(gripperMotionPlan);
    ROS_INFO_STREAM("Gripper Motion Plan: " << (gripperSuccess ? "Opened gripper" : "FAILED TO OPEN GRIPPER"));
  }
  return (gripperSuccess ? true : false);*/

  //temporary open gripper function until joint measurements is available in reality
  e1_interface::E1Command gripper_command;
  gripper_command.header.seq = 0;
  gripper_command.header.stamp.sec = 0;
  gripper_command.header.stamp.nsec = 0;
  gripper_command.blade[0] = false;
  gripper_command.blade[1] = false;
  gripper_command.track_valve_opening[0] = 0;
  gripper_command.track_valve_opening[1] = 0;
  gripper_command.slewing_valve_opening = 0;
  gripper_command.boom_valve_opening = 0;
  gripper_command.stick_valve_opening = 0;
  gripper_command.bucket_valve_opening = 0;
  gripper_command.cabin_rotation_valve_opening = 0;
  gripper_command.auxiliary_hydraulic_valve_opening = -500;  //open gripper to near max value (fixed)
  gripper_command.third_control_circuit_valve_opening = 0;
  gripper_command.auxiliary_hydraulic_switch_valve = false;

  ros::Time start_time = ros::Time::now();
  ros::Duration timeout(2.0); // Timeout of 2 seconds
  while(ros::Time::now() - start_time < timeout) {
    e1_gripper_pub.publish(gripper_command);
  }
  ROS_INFO_STREAM("Opened Gripper successfully");
  return true;
}

bool akit_pick_place::closeGripper(moveit_msgs::CollisionObject object_){

  //relate close gripper to the side lengths of the object --> gripper close angle is related to the minimum side
  double max_open_length = 0.7; //measured in Rviz
  double tolerance = 0.01; //for better caging
  double min_side = object_.primitives[0].dimensions[0];

  //compute minimum side
  for(int i = 0.0; i < object_.primitives[0].dimensions.size(); ++i){
    if (object_.primitives[0].dimensions[i] < min_side){
      min_side = object_.primitives[0].dimensions[i];
    }
  }
  //to get diameter not radius --> not needed for cubes and cuboids
  if(object_.primitives[0].type == shape_msgs::SolidPrimitive::CYLINDER){
    min_side *= 2;
  }

  double gripper_close_angle = ((min_side * (M_PI/3)) / max_open_length) - tolerance;

  //update start state to current state
  gripperState = gripperGroup->getCurrentState();
  gripperGroup->setStartState(*gripperState);

  gripperJointPositions[1] = gripper_close_angle;
  gripperJointPositions[2] = gripper_close_angle;
  gripperGroup->setJointValueTarget(gripperJointPositions);

  //plan and execute motion plan
  gripperSuccess = (gripperGroup->plan(gripperMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (gripperSuccess){
    gripperGroup->execute(gripperMotionPlan);
    ROS_INFO_STREAM("Gripper Motion Plan: " << (gripperSuccess ? "Closed gripper" : "FAILED TO CLOSE GRIPPER"));
  }
  return (gripperSuccess ? true : false);
}

void akit_pick_place::jointStatesCallback(const sensor_msgs::JointState joint_states_msg){
   e1_joint_states = joint_states_msg;
}

bool akit_pick_place::executeAxisCartesianMotion(bool direction, double cartesian_distance, char axis){

  //cartesian motion function for iosb interface

  //get current pose

  //getPosition fk object
  moveit_msgs::GetPositionFK getPositionFK_msg;
  getPositionFK_msg.request.header.frame_id = BASE_LINK;

  if (!nh.hasParam(PLANNING_GROUP)){
    ROS_ERROR_STREAM(PLANNING_GROUP + " parameter not loaded, did you load planning groups yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam(PLANNING_GROUP , getPositionFK_msg.request.fk_link_names);

  for (int i = 0; i < getPositionFK_msg.request.fk_link_names.size(); ++i){
    getPositionFK_msg.request.robot_state.joint_state.name.push_back(e1_joint_states.name[i]);
    getPositionFK_msg.request.robot_state.joint_state.position.push_back(e1_joint_states.position[i]);
    getPositionFK_msg.request.robot_state.joint_state.velocity.push_back(e1_joint_states.velocity[i]);
  }

  //call forward kinematics service
  e1_compute_fk_client.call(getPositionFK_msg);
  if (getPositionFK_msg.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
    ROS_INFO_STREAM("Successfully computed forward kinematics to get current pose in " + BASE_LINK + " frame");
  } else {
    ROS_ERROR("Failed to compute forward kinematics, current pose unknown");
    return false;
    exit(1);
  }

  //start transformation to quickcoupler frame
  geometry_msgs::PoseStamped pose_in_base_link_frame, pose_in_eef_frame;

  pose_in_base_link_frame = getPositionFK_msg.response.pose_stamped.back(); //eef == quickcoupler is the last link in the chain
  pose_in_base_link_frame.header.frame_id = BASE_LINK;  //add frame reference

  //cartesian motion service object
  moveit_msgs::GetCartesianPath cartesian_path_msg;

  //push back first cartesian path point before transformation, service fails if only one waypoint is given
  cartesian_path_msg.request.waypoints.push_back(pose_in_base_link_frame.pose);

  //modify second way point
  //transform from base link frame to quickcoupler frame, wait to avoid time difference exceptions
  transform_listener.waitForTransform(EEF_PARENT_LINK, BASE_LINK, ros::Time::now(), ros::Duration(5.0));
  transform_listener.transformPose(EEF_PARENT_LINK,ros::Time(0), pose_in_base_link_frame, BASE_LINK, pose_in_eef_frame);

  if (!direction){        //downwards cartesian motion in quickcoupler frame
    switch (axis){
    case 'x':
      pose_in_eef_frame.pose.position.x -= cartesian_distance;
      break;
    case 'y':
      pose_in_eef_frame.pose.position.y -= cartesian_distance;
      break;
    case 'z':
      pose_in_eef_frame.pose.position.z -= cartesian_distance;
      break;
    }
  } else {                //upwards cartesian motion in quickcoupler frame
    switch (axis){
    case 'x':
      pose_in_eef_frame.pose.position.x += cartesian_distance;
      break;
    case 'y':
      pose_in_eef_frame.pose.position.y += cartesian_distance;
      break;
    case 'z':
      pose_in_eef_frame.pose.position.z += cartesian_distance;
      break;
    }
  }
  //transform back to base frame, wait to avoid time difference exceptions
  transform_listener.waitForTransform(BASE_LINK, EEF_PARENT_LINK, ros::Time::now(), ros::Duration(5.0));
  transform_listener.transformPose(BASE_LINK,ros::Time(0), pose_in_eef_frame, EEF_PARENT_LINK, pose_in_base_link_frame);

  //header frame for the specified waypoints
  cartesian_path_msg.request.header.frame_id = BASE_LINK;
  //start state of the cartesian path is the current state of the robot
  cartesian_path_msg.request.start_state = getPositionFK_msg.request.robot_state;

  cartesian_path_msg.request.group_name = PLANNING_GROUP;
  cartesian_path_msg.request.link_name =  EEF_PARENT_LINK;

  //second waypoint
  cartesian_path_msg.request.waypoints.push_back(pose_in_base_link_frame.pose);
  cartesian_path_msg.request.max_step = 0.05;
  cartesian_path_msg.request.jump_threshold = 0.0;

  //call service server
   e1_cartesian_path_client.call(cartesian_path_msg);

   if (cartesian_path_msg.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
      ROS_INFO_STREAM("Successfully computed "  << cartesian_path_msg.response.fraction * 100 << "% cartesian path");
   } else {
     ROS_ERROR("Failed to compute cartesian path");
     return false;
     exit(1);
   }

   sleep(0.5);

//   ROS_INFO_STREAM("cartesian path solution points");
//   for (int i = 0; i < 5; ++i){
//       ROS_INFO_STREAM(cartesian_path_msg.response.solution.joint_trajectory.joint_names[i]);
//          for (int j = 0; j < 5; ++j){
//                  ROS_INFO_STREAM(cartesian_path_msg.response.solution.joint_trajectory.points[i].positions[j]);
//     }
//   }

   if ((cartesian_path_msg.response.fraction * 100) >= 50.0){
      moveit_msgs::MoveGroupActionResult traj_msg;

      //trajectory message for planned traj is the cartesian path trajectory
      traj_msg.result.planned_trajectory = cartesian_path_msg.response.solution;

      //set the error code to SUCCESS and status to SUCCEEDED for the planner
      traj_msg.status.status = 3;
      traj_msg.result.error_code.val = 1;
      traj_msg.result.trajectory_start.joint_state.header.frame_id = BASE_LINK;

      //add current joint states
      traj_msg.result.trajectory_start = cartesian_path_msg.request.start_state;

//      ROS_INFO_STREAM("joint states trajectory start");
//      for (int i = 0; i < traj_msg.result.trajectory_start.joint_state.name.size(); i++){
//        ROS_INFO_STREAM(traj_msg.result.trajectory_start.joint_state.name[i]);
//        ROS_INFO_STREAM(traj_msg.result.trajectory_start.joint_state.position[i]);
//      }

      e1_trajectory_publisher.publish(traj_msg);

      //execute trajectory service parameters left empty
      moveit_msgs::ExecuteKnownTrajectory execute_traj_srv;
      e1_execute_traj_client.call(execute_traj_srv);


      if (execute_traj_srv.response.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
        ROS_INFO_STREAM("Successfully executed cartesian trajectory");
        return true;
      } else {
        ROS_ERROR_STREAM("failed to execute cartesian trajectorty = " << execute_traj_srv.response.error_code.val);
        return false;
        exit(1);
      }
   } else {
     ROS_ERROR("Cannot execute cartesian motion, plan < 50 %%");
     return false;
   }
}

//executes first pose reached in input position vector
bool akit_pick_place::planAndExecuteCartesianGoals(std::vector<geometry_msgs::Pose> poses, std::string pose){

  int count = 0;
  e1_motion_sequence::SetGoal e1_set_goal_srv;
  e1_motion_sequence::GoToGoal e1_go_to_srv;

  e1_set_goal_client.waitForExistence();

  //planning service client unchanged variables
  e1_set_goal_srv.request.planning_group = PLANNING_GROUP;
  e1_set_goal_srv.request.cartesian_goal_pose = true;
  e1_set_goal_srv.request.project_pose = false;
  e1_set_goal_srv.request.composite_path = false;
  e1_set_goal_srv.request.action = 1;

  //execution service client unchanged variables

  for (int i = 0; i < poses.size(); i++){

    //clear vector to add new pose
    e1_set_goal_srv.request.goal_position.clear();

    //grasp poses
    e1_set_goal_srv.request.goal_position.push_back(poses[i].position.x);
    e1_set_goal_srv.request.goal_position.push_back(poses[i].position.y);
    e1_set_goal_srv.request.goal_position.push_back(poses[i].position.z);
    e1_set_goal_srv.request.goal_position.push_back(poses[i].orientation.x);
    e1_set_goal_srv.request.goal_position.push_back(poses[i].orientation.y);
    e1_set_goal_srv.request.goal_position.push_back(poses[i].orientation.z);
    e1_set_goal_srv.request.goal_position.push_back(poses[i].orientation.w);

    //call motion planning service
    e1_set_goal_client.call(e1_set_goal_srv);

    //check that setting the goal was successful
    if (e1_set_goal_srv.response.success == true){
      ROS_INFO_STREAM("Successfully added pose to list");
    } else {
      ROS_ERROR("Failed to add pose to list");
      return false;
      exit(1);
    }

    //fill GoToGoal srv
    e1_go_to_srv.request.goal_number = e1_set_goal_srv.response.goal_number;

    //call motion execution service
    e1_go_to_goal_client.call(e1_go_to_srv);

    if (e1_go_to_srv.response.error.val != moveit_msgs::MoveItErrorCodes::SUCCESS){

       ROS_ERROR_STREAM("motion planning to " << pose << " position " << count << " failed on error code = " << e1_go_to_srv.response.error.val);
       ROS_INFO_STREAM("Replanning");
       count++;
         if (count == poses.size()){
           ROS_ERROR_STREAM("Failed to plan to " << pose << " position");
           return false;
           exit(1);
         }
       continue;

    } else {

      ROS_INFO_STREAM("Cartesian goal execution is successfull");
      return true;
      break;
    }
  }
}

bool akit_pick_place::planAndExecuteJointGoals(std::vector<double> joint_states, bool add_to_current_joint_states){

  //wait for joint states subscription
  sleep(1.0);

  e1_motion_sequence::SetGoal e1_set_goal_srv;
  e1_motion_sequence::GoToGoal e1_go_to_srv;

  e1_set_goal_client.waitForExistence();

  //planning service client unchanged variables
  e1_set_goal_srv.request.planning_group = PLANNING_GROUP;
  e1_set_goal_srv.request.cartesian_goal_pose = false;
  e1_set_goal_srv.request.project_pose = false;
  e1_set_goal_srv.request.composite_path = false;
  e1_set_goal_srv.request.action = 1;

  if (add_to_current_joint_states){
    for (int i = 0; i < joint_states.size(); ++i){
      e1_set_goal_srv.request.goal_position.push_back(e1_joint_states.position[i] + joint_states[i]);
    }
  } else {
    for (int i = 0; i < joint_states.size(); ++i){
      e1_set_goal_srv.request.goal_position.push_back(joint_states[i]);
    }
  }

  e1_set_goal_client.call(e1_set_goal_srv);

  //check that setting the goal was successful
  if (e1_set_goal_srv.response.success == true){
    ROS_INFO_STREAM("Successfully added pose to list");
  } else {
    ROS_ERROR("Failed to add pose to list");
    return false;
    exit(1);
  }

  //fill GoToGoal srv
  e1_go_to_srv.request.goal_number = e1_set_goal_srv.response.goal_number;

  //call motion execution service
  e1_go_to_goal_client.call(e1_go_to_srv);

  if (e1_go_to_srv.response.error.val == moveit_msgs::MoveItErrorCodes::SUCCESS){
      ROS_INFO_STREAM("joint goal execution is successfull");
      return true;
  } else {
      ROS_ERROR_STREAM("motion planning in joint space failed, error code = " << e1_go_to_srv.response.error.val);
      return false;
      exit(1);
  }
}

//allow gripper to touch object to be picked
bool akit_pick_place::allowObjectCollision(std::string object_id){

  acm = planningScenePtr->getAllowedCollisionMatrix();
  std::vector<std::string> gripper_links;

  if (!nh.hasParam(EEF_GROUP)){
    ROS_ERROR_STREAM(EEF_GROUP + " parameter not loaded, did you load planning groups yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam(EEF_GROUP, gripper_links);

  for (int i= 0; i < gripper_links.size(); ++i){
    acm.setEntry(gripper_links[i], object_id, true);
  }

  acm.getMessage(planning_scene_msg_.allowed_collision_matrix);
  planning_scene_msg_.is_diff = true;
  planning_scene_srv.request.scene = planning_scene_msg_;

  if (planning_scene_diff_client.call(planning_scene_srv)){
    return true;
  } else {
    return false;
  }
}

//reset acm to restore object as a collision object
bool akit_pick_place::resetAllowedCollisionMatrix(std::string object_id){

  acm = planningScenePtr->getAllowedCollisionMatrix();
  std::vector<std::string> gripper_links;

  if (!nh.hasParam(EEF_GROUP)){
    ROS_ERROR_STREAM(EEF_GROUP + " parameter not loaded, did you load planning groups yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam(EEF_GROUP, gripper_links);

  for (int i= 0; i < gripper_links.size(); ++i){
    acm.removeEntry(gripper_links[i], object_id);
  }

  acm.getMessage(planning_scene_msg_.allowed_collision_matrix);
  planning_scene_msg_.is_diff = true;
  planning_scene_srv.request.scene = planning_scene_msg_;

  if (planning_scene_diff_client.call(planning_scene_srv)){
    return true;
  } else {
    return false;
  }
}

//allow collision during tool exchange
void akit_pick_place::allowToolCollision(std::string tool_id){
  acm = acm = planningScenePtr->getAllowedCollisionMatrix();
  std::transform(tool_id.begin(), tool_id.end(), tool_id.begin(), ::tolower);

  if (tool_id == "bucket_raedlinger")
  {
    acm.setEntry(EEF_PARENT_LINK,BUCKET_FRAME, true);
    acm.setEntry("stick",BUCKET_FRAME,true);
    acm.setEntry("bucket_lever_2",BUCKET_FRAME,true);
  }
  else if (tool_id == "gripper_rotator")
  {
    acm.setEntry(EEF_PARENT_LINK,GRIPPER_FRAME, true);
    acm.setEntry("stick",GRIPPER_FRAME,true);
  }
  acm.getMessage(planning_scene_msg_.allowed_collision_matrix);
  planning_scene_msg_.is_diff = true;
  planning_scene_srv.request.scene = planning_scene_msg_;
  planning_scene_diff_client.call(planning_scene_srv);
}


//attach object to gripper using planning scene services
bool akit_pick_place::attachCollisionObject(moveit_msgs::CollisionObject collisionObject){

  //create attached collision object + planning scene instances
  moveit_msgs::AttachedCollisionObject attached_collision_object;
  moveit_msgs::ApplyPlanningScene planningSceneSrv_;
  moveit_msgs::PlanningScene planning_scene;

  //filling attached collision object instance
  attached_collision_object.object.header.frame_id = collisionObject.header.frame_id;
  attached_collision_object.object.id = collisionObject.id;
  attached_collision_object.link_name = GRIPPER_FRAME;
  attached_collision_object.object.primitives.push_back(collisionObject.primitives[0]);
  attached_collision_object.object.primitive_poses.push_back(collisionObject.primitive_poses[0]);
  attached_collision_object.object.operation = collisionObject.operation; //operation bit is ADD

  //remove collision object from enviroment
  ROS_INFO_STREAM("removing collision object from the enviroment");
  attached_collision_object.object.operation = attached_collision_object.object.REMOVE; //operation bit is REMOVE from world

  //add "removed" collisin object to planning scene
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_collision_object.object);

  //add collision object to attached collision objects
  attached_collision_object.object.operation = attached_collision_object.object.ADD; //operation bit is ADD to attached
  planning_scene.robot_state.attached_collision_objects.push_back(attached_collision_object);
  planning_scene.is_diff = true;

  //call service
  planningSceneSrv_.request.scene = planning_scene;
  planning_scene_diff_client_.call(planningSceneSrv_);

  if (planningSceneSrv_.response.success == true){

    ROS_INFO_STREAM("successfully attached collision object to gripper");
    return true;

  } else {

    ROS_INFO_STREAM("failed to attach collision object to gripper");
    return false;
  }
}

//detach object to gripper using planning scene services
bool akit_pick_place::detachCollisionObject(moveit_msgs::CollisionObject collisionObject){

  //remove first from attached objects and add to the planning scene

  //create necessary objects for services
  moveit_msgs::AttachedCollisionObject detached_collision_object;
  moveit_msgs::ApplyPlanningScene planningSceneSrv_;
  moveit_msgs::PlanningScene planning_scene;
  moveit_msgs::GetPlanningScene get_planning_scene_srv; //get planning scene service client object

  //get robot state from service call --> returns  CURRENT attached collision object info so that object is returned to env at correct pose
  get_planning_scene_srv.request.components.components = 4;
  get_planning_scene_client.call(get_planning_scene_srv);

  //filling detached collision object instance
  detached_collision_object.object.header.frame_id = get_planning_scene_srv.response.scene.robot_state.attached_collision_objects[0].object.header.frame_id;
  detached_collision_object.object.id = get_planning_scene_srv.response.scene.robot_state.attached_collision_objects[0].object.id;
  detached_collision_object.link_name = get_planning_scene_srv.response.scene.robot_state.attached_collision_objects[0].link_name;
  detached_collision_object.object.primitives.push_back(get_planning_scene_srv.response.scene.robot_state.attached_collision_objects[0].object.primitives[0]);
  detached_collision_object.object.primitive_poses.push_back(get_planning_scene_srv.response.scene.robot_state.attached_collision_objects[0].object.primitive_poses[0]);
  detached_collision_object.object.operation = collisionObject.operation; //operation bit is ADD

  //removing from attached collision objects
  detached_collision_object.object.operation = detached_collision_object.object.REMOVE; //operation bit is REMOVE from attached
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detached_collision_object);
  planning_scene.robot_state.is_diff = true;

  //adding to world again
  detached_collision_object.object.operation = detached_collision_object.object.ADD; //operation bit is ADD to world again
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(detached_collision_object.object); //spawns the object relative to gripper frame !! check
  planning_scene.is_diff = true;

  planningSceneSrv_.request.scene = planning_scene;
  planning_scene_diff_client_.call(planningSceneSrv_);

  if (planningSceneSrv_.response.success == true){

    ROS_INFO_STREAM("successfully detached collision object from gripper");
    return true;

  } else {

    ROS_INFO_STREAM("failed to detach collision object from gripper");
    return false;
  }
}

bool akit_pick_place::pick(moveit_msgs::CollisionObject object_){
  ROS_INFO_STREAM("---------- Starting Pick Routine ----------");

   //move from home position to pre-grasp position
  if(FromGraspGenerator){

    //loop through all grasp poses
    if(!this->planAndExecuteCartesianGoals(grasp_pose_vector, "pre_grasp")){
      ROS_ERROR("Failed to plan and execute");
      grasp_pose_vector.clear();
      return false;
      exit(1);
    }

  } else { //if grasp poses are entered separatly from blender(needed if object is not a box,cuboid,cylinder!)

    std::vector<geometry_msgs::Pose> positions;
    positions.push_back(pre_grasp_pose);

    if(!this->planAndExecuteCartesianGoals(positions, "pre-grasp")){
      ROS_ERROR("Failed to plan and execute");
      return false;
      exit(1);
    }
  }

  //clear grasp_pose_vector
  grasp_pose_vector.clear();

  //opening gripper
  if (!this->openGripper()){
    ROS_ERROR("Failed to open Gripper");
    return false;
    exit(1);
  }

  //temporary until joint measurements for gripper are implemented from iosb
  visual_tools->prompt("please rotate gripper then press next");

  /*if (!side_grasps){   //rotating gripper to adjust with different orientations (only works with top grasping)
    if (!this->rotateGripper(object_)){
      ROS_ERROR("Failed to rotate Gripper");
      return false;
      exit(1);
    }
  }*/

  //cartesian motion downwards
  if (!this->executeAxisCartesianMotion(DOWN, GRIPPER_JAW_LENGTH, 'z')){
    ROS_ERROR("Failed to execute downwards cartesian motion");
    return false;
    exit(1);
  }

  /*int count = 0.0;
  while (!this->executeAxisCartesianMotion(DOWN, GRIPPER_JAW_LENGTH, 'z')){
    this->rotateGripper(M_PI/6);
    count++;
    if (count == 6.0) {
      ROS_ERROR("Failed to execute downwards cartesian motion");
      return false;
      exit(1);
    }
  }*/

  //add allowed collision matrix
  if (!this->allowObjectCollision(object_.id)){
    ROS_ERROR("Failed to allow collision with object ");
    return false;
    exit(1);
  }

  //temporary until joint measurements for gripper are implemented from iosb
  visual_tools->prompt("please close gripper then press next");

  //closing gripper
  /*if (!this->closeGripper(object_)){
    ROS_ERROR("Failed to close Gripper");
    return false;
    exit(1);
  }*/

  if (!this->attachCollisionObject(object_)){
    ROS_ERROR("Failed to attach collision object ");
    return false;
    exit(1);
  }

  //give time for planning scene to process
  ros::Duration(1.0).sleep();

  //cartesian motion upwards (post-grasp position)
  if (!this->executeAxisCartesianMotion(UP, GRIPPER_JAW_LENGTH, 'z')){
    ROS_ERROR("Failed to execute upwards cartesian motion");
    return false;
    exit(1);
  }
  visual_tools->deleteAllMarkers();

  return true;
}

bool akit_pick_place::place(moveit_msgs::CollisionObject object_){
  ROS_INFO_STREAM("---------- Starting Place Routine ----------");

  //moving from post-grasp position to pre-place position
  if(FromGraspGenerator){

    //loop through all grasp poses
    if(!this->planAndExecuteCartesianGoals(grasp_pose_vector, "pre_place")){
      ROS_ERROR("Failed to plan and execute");
      grasp_pose_vector.clear();
      return false;
      exit(1);
    }

  } else { //if place poses are entered separatly from blender(needed if object is not a box,cuboid,cylinder!)

    std::vector<geometry_msgs::Pose> positions;
    positions.push_back(pre_place_pose);

    if(!this->planAndExecuteCartesianGoals(positions, "pre_place")){
      ROS_ERROR("Failed to plan and execute");
      return false;
      exit(1);
    }
  }

  //clear grasp pose vector
  grasp_pose_vector.clear();

  //cartesian motion downwards
  if(!this->executeAxisCartesianMotion(DOWN, GRIPPER_JAW_LENGTH, 'z')){ //experiment and change gripper_jaw_length --> drop the object rather than place it
    ROS_ERROR("Failed to execute downwards cartesian motion");
    return false;
    exit(1);
  }

  if (!this->detachCollisionObject(object_)){
    ROS_ERROR("Failed to detach collision object ");
    return false;
    exit(1);
  }

  //give time for planning scene to process
  ros::Duration(1.0).sleep();

  //opening gripper
  if(!this->openGripper()){
    ROS_ERROR("Failed to open Gripper");
    return false;
    exit(1);
  }

  //temporary
  visual_tools->prompt("when the gripper is completely opened press next");

  //cartesian motion upwards
  if(!this->executeAxisCartesianMotion(UP, GRIPPER_JAW_LENGTH, 'z')){
    ROS_ERROR("Failed to execute upwards cartesian motion");
    return false;
    exit(1);
  }
  //delete published trajectory
  visual_tools->deleteAllMarkers();

  //reset allowed collision matrix
  if (!this->resetAllowedCollisionMatrix(object_.id)){
    ROS_ERROR("Failed to reset allowed collision matrix. object still not in collision");
    return false;
    exit(1);
  }
  return true;
}

//-----------------------------world interaction methods------------------------------

moveit_msgs::CollisionObject akit_pick_place::addCollisionCylinder(geometry_msgs::Pose cylinder_pose,
                                                                   std::string cylinder_name, double cylinder_height, double cylinder_radius){
  //service objects
  moveit_msgs::ApplyPlanningScene planningSceneSrv_;
  moveit_msgs::PlanningScene planningSceneMsg_;
  moveit_msgs::CollisionObject cylinder;
  cylinder.id = cylinder_name;
  cylinder.header.stamp = ros::Time::now();
  cylinder.header.frame_id = WORLD_FRAME;
  //primitives
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = cylinder_height;
  primitive.dimensions[1] = cylinder_radius;

  cylinder.primitives.push_back(primitive);
  cylinder.primitive_poses.push_back(cylinder_pose);
  cylinder.operation = moveit_msgs::CollisionObject::ADD;

  //calling apply planning scene service
  planningSceneMsg_.world.collision_objects.push_back(cylinder);
  planningSceneMsg_.is_diff = true;
  planningSceneSrv_.request.scene = planningSceneMsg_;
  planning_scene_diff_client_.call(planningSceneSrv_);

  return cylinder;
}

moveit_msgs::CollisionObject akit_pick_place::addCollisionBlock(geometry_msgs::Pose block_pose, std::string block_name, double block_size_x, double block_size_y, double block_size_z ){

  //service objects
  moveit_msgs::ApplyPlanningScene planningSceneSrv_;
  moveit_msgs::PlanningScene planningSceneMsg_;
  moveit_msgs::CollisionObject block;
  block.id = block_name;
  block.header.stamp = ros::Time::now();
  block.header.frame_id = WORLD_FRAME;
  //primitives
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = block_size_x;
  primitive.dimensions[1] = block_size_y;
  primitive.dimensions[2] = block_size_z;
  block.primitives.push_back(primitive);
  block.primitive_poses.push_back(block_pose);
  block.operation = moveit_msgs::CollisionObject::ADD;

  //calling apply planning scene service
  planningSceneMsg_.world.collision_objects.push_back(block);
  planningSceneMsg_.is_diff = true;
  planningSceneSrv_.request.scene = planningSceneMsg_;
  planning_scene_diff_client_.call(planningSceneSrv_);

  return block;
}
//instead of position constraints --> no motion in -z direction of world frame
void akit_pick_place::addGround(){
  geometry_msgs::Pose groundPose;
  groundPose.position.x = 0.0;
  groundPose.position.y = 0.0;
  groundPose.position.z = -0.05;
  groundPose.orientation.x = 0.0;
  groundPose.orientation.y = 0.0;
  groundPose.orientation.z = 0.0;
  groundPose.orientation.w = 1.0;
  this->addCollisionBlock(groundPose, "ground", 10.0,10.0,0.1);
  ros::Duration(1.0).sleep();
}

CollisionObjectsMap akit_pick_place::getSceneCollisionObjects(){

  //call get planning scene service
  moveit_msgs::GetPlanningScene get_planning_scene_msg;
  get_planning_scene_msg.request.components.components = 16; //WORLD_OBJECT_GEOMETRY
  get_planning_scene_client.call(get_planning_scene_msg);

  std::vector<moveit_msgs::CollisionObject> scene_objects = get_planning_scene_msg.response.scene.world.collision_objects;
  CollisionObjectsMap CollisionObjectsMap_;

  for (int i = 0; i < scene_objects.size(); ++i){
    CollisionObjectsMap_.insert({scene_objects[i].id, scene_objects[i]});
  }
  return CollisionObjectsMap_;
}

AttachedCollisionObjectsMap akit_pick_place::getAttachedCollisionObjects(){
  //call get planning scene service
  moveit_msgs::GetPlanningScene get_planning_scene_msg;
  get_planning_scene_msg.request.components.components = 4; //ROBOT_STATE_ATTACHED_OBJECTS
  get_planning_scene_client.call(get_planning_scene_msg);

  std::vector<moveit_msgs::AttachedCollisionObject> attached_objects = get_planning_scene_msg.response.scene.robot_state.attached_collision_objects;
  AttachedCollisionObjectsMap AttachedCollisionObjectsMap_;

  for (int i = 0; i < attached_objects.size(); ++i){
    AttachedCollisionObjectsMap_.insert({attached_objects[i].object.id, attached_objects[i]});
  }
  return AttachedCollisionObjectsMap_;
}

void akit_pick_place::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  //stores feedback
  if(feedback->MOUSE_DOWN){
    interactive_pose = feedback->pose;
    interactive_name = feedback->marker_name;
  }
}

void akit_pick_place::addInteractiveMarker(geometry_msgs::Pose marker_pose,
                                           std::string marker_name, shape_msgs::SolidPrimitive shape){

  visualization_msgs::Marker i_marker; // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = BASE_LINK;
  int_marker.pose.position = marker_pose.position;
  int_marker.pose.orientation = marker_pose.orientation;
  int_marker.name = marker_name;
  i_marker.color.r = 0.5;
  i_marker.color.g = 0.5;
  i_marker.color.b = 0.5;
  i_marker.color.a = 1.0;
  double tolerance = 0.01;

  if (shape.type == shape_msgs::SolidPrimitive::BOX){ //fix for cuboids
    i_marker.type = visualization_msgs::Marker::CUBE;
    i_marker.scale.x = i_marker.scale.y = i_marker.scale.z = shape.dimensions[0] + tolerance; //cannot be same size as collision object, won't return feedback
  } else if (shape.type == shape_msgs::SolidPrimitive::CYLINDER){
    i_marker.type = visualization_msgs::Marker::CYLINDER;
    i_marker.scale.x = i_marker.scale.y = (2 * shape.dimensions[1]) + tolerance;
    i_marker.scale.z = shape.dimensions[0] + tolerance;
  }

  // add the control to the interactive marker
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(i_marker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_3D;
  int_marker.controls.push_back(control);

  //add the interactive marker to our collection & tell the server to call processFeedback() when feedback arrives for it
  server->insert(int_marker);
  server->setCallback(int_marker.name, processFeedback);
  server->applyChanges();
}

void akit_pick_place::addInteractiveMarkers(){ //server
  ros::Rate rate(0.75); //updates planning scene --> interactive marker position changes after pick
  while(ros::ok())
  {
    collision_objects_map = this->getSceneCollisionObjects(); //return all collision objects in planning scene
    CollisionObjectsMap::iterator it;
    for (it = collision_objects_map.begin(); it != collision_objects_map.end(); ++it){
      if (it->first == "ground")
        continue;  //skip adding an interactive marker to the ground object
      this->addInteractiveMarker(it->second.primitive_poses[0], it->first, it->second.primitives[0]);
    }
    rate.sleep(); // or use ros::spin(); after removing rate&while loop --> but planning scene is not updated
  }
}

bool akit_pick_place::interactive_pick_place(std::vector<geometry_msgs::Pose> place_positions){

  /*
   * takes input a vector of user desired place locations and waits for user to choose an object to pick
   * pick is interactive --> place is predefined
   */

  marker_sub = nh.subscribe("/akit_pick_place/feedback", 10, processFeedback);
  collision_objects_map = this->getSceneCollisionObjects(); //return all collision objects in planning scene
  CollisionObjectsMap::iterator it;
  AttachedCollisionObjectsMap::iterator a_it;
  int count = 0;

  while (ros::ok() && count < place_positions.size()){
    ROS_INFO_STREAM("Please choose object to pick ");
    //wait for user input
    boost::shared_ptr<const visualization_msgs::InteractiveMarkerFeedback> msgReceived =
        ros::topic::waitForMessage<visualization_msgs::InteractiveMarkerFeedback>("/akit_pick_place/feedback");
    if (msgReceived){
      ROS_INFO_STREAM("-------------------Received object to grasp msg-------------------");
      //loop through all collision objects until matching collision object is found
      for (it = collision_objects_map.begin(); it != collision_objects_map.end(); ++it){
        if (interactive_name == it->first){
          if (it->second.primitives[0].type == shape_msgs::SolidPrimitive::CYLINDER){
            this->generateGrasps(interactive_pose, it->second.primitives[0].dimensions[0], it->second.primitives[0].dimensions[1]);
            if(!this->pick(it->second)){
              ROS_ERROR("Failed to pick");
              return false;
              exit(1);
            }
            break;
          } else if (it->second.primitives[0].type == shape_msgs::SolidPrimitive::BOX){
            this->generateGrasps(interactive_pose, it->second.primitives[0].dimensions[0]);
            if (!this->pick(it->second)){
              ROS_ERROR("Failed to pick");
              return false;
              exit(1);
            }
            break;
          }
        } else {
          ROS_INFO_STREAM("No Collision Object Selected!");
        }
      }
    }
    //placing of attached object
    attached_collision_objects_map = this->getAttachedCollisionObjects();
    for(a_it = attached_collision_objects_map.begin();a_it != attached_collision_objects_map.end(); ++a_it){
      if (a_it->second.object.primitives[0].type == shape_msgs::SolidPrimitive::CYLINDER){
        this->generateGrasps(place_positions[count],a_it->second.object.primitives[0].dimensions[0], a_it->second.object.primitives[0].dimensions[1]);
        if(!this->place(a_it->second.object)){
          ROS_ERROR("Failed to place");
          return false;
          exit(1);
        }
      } else if (a_it->second.object.primitives[0].type == shape_msgs::SolidPrimitive::BOX){
        this->generateGrasps(place_positions[count],a_it->second.object.primitives[0].dimensions[0]);
        if(!this->place(a_it->second.object)){
          ROS_ERROR("Failed to place");
          return false;
          exit(1);
        }
      }
      count++;
    }
  }
  return true;
}

bool akit_pick_place::attachTool(std::string tool_frame_id){ //change to tool frame id and remove if conditions!

  std::transform(tool_frame_id.begin(), tool_frame_id.end(), tool_frame_id.begin(), ::tolower);

  if (tool_frame_id != GRIPPER_FRAME && tool_frame_id != BUCKET_FRAME){
    ROS_ERROR_STREAM("Unknown tool, please write correct tool name");
    return false;
    exit(1);
  }

  //parameters
  double quickcoupler_z, quickcoupler_x, distance_above_gripper;

  if (!nh.hasParam("quickcoupler_z")){
    ROS_ERROR_STREAM("tool exchange parameter (quickcoupler_z) not loaded, did you load initialization yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam("quickcoupler_z", quickcoupler_z);

  if (!nh.hasParam("quickcoupler_x")){
    ROS_ERROR_STREAM("tool exchange parameter (quickcoupler_x) not loaded, did you load initialization yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam("quickcoupler_x", quickcoupler_x);

  if (!nh.hasParam("distance_above_gripper")){
    ROS_ERROR_STREAM("tool exchange parameter (distance_above_gripper) not loaded, did you load initialization yaml file ?");
    return false;
    exit(1);
  }

  nh.getParam("distance_above_gripper", distance_above_gripper);

  tf::Quaternion q = tf::createQuaternionFromRPY(0.0,-M_PI/2,0.0); //rotate 90deg around y-axis
  geometry_msgs::PoseStamped initial_pose_tool_frame, initial_pose_base_frame;

  //create initial pose for quickcoupler
  initial_pose_tool_frame.header.frame_id = tool_frame_id;
  initial_pose_tool_frame.pose.position.x = - quickcoupler_z; //translate the quickcoupler so that both locks match
  initial_pose_tool_frame.pose.position.y = 0.0;
  initial_pose_tool_frame.pose.position.z = distance_above_gripper;
  initial_pose_tool_frame.pose.orientation.x = q[0];
  initial_pose_tool_frame.pose.orientation.y = q[1];
  initial_pose_tool_frame.pose.orientation.z = q[2];
  initial_pose_tool_frame.pose.orientation.w = q[3];

  //allow collision between tool group and quickcoupler --> eef parent link
  this->allowToolCollision(tool_frame_id);

  //transform pose from gripper/bucket frame to base link frame
  transform_listener.waitForTransform(BASE_LINK,tool_frame_id, ros::Time::now(), ros::Duration(0.1)); //avoid time difference exception
  transform_listener.transformPose(BASE_LINK,ros::Time(0), initial_pose_tool_frame, tool_frame_id, initial_pose_base_frame);

  //visualize point
  std::vector<geometry_msgs::Pose> visualize_point;
  visualize_point.push_back(initial_pose_tool_frame.pose);
  this->visualizeGrasps(visualize_point, tool_frame_id);

  //motion planning
  std::vector<geometry_msgs::Pose> points;
  points.push_back(initial_pose_base_frame.pose);
  if(!this->planAndExecuteCartesianGoals(points, "initial pose")){
    ROS_ERROR("Failed to plan and execute");
    return false;
    exit(1);
  }

  //promt user  //add position error detection
  visual_tools->prompt("proceed ? ");

  //execute cartesian motion in -x axis direction
  if (!this->executeAxisCartesianMotion(DOWN, distance_above_gripper + quickcoupler_x, 'x')){
    ROS_ERROR("Failed to execute cartesian motion");
    return false;
    exit(1);
  }

  //rotation of the quickcoupler +90deg






  //get current joint state
  akitState = akitGroup->getCurrentState();
  akitState->copyJointGroupPositions(akitJointModelGroup,akitJointPositions);

  //promt user
  visual_tools->prompt("proceed ? ");

  //update start state to current state
  akitState = akitGroup->getCurrentState();
  akitGroup->setStartState(*akitState);

  //rotate quickcoupler +90deg
  akitJointPositions[4] += M_PI/2;
  akitGroup->setJointValueTarget(akitJointPositions);
  akitSuccess = (akitGroup->plan(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (akitSuccess){
    akitGroup->execute(MotionPlan);
    ROS_INFO_STREAM("Rotation of quickcoupler --> success");
  } else {
    ROS_ERROR_STREAM("Failed to rotate quickcoupler");
    return false;
    exit(1);
  }

  //activate lock --> no control in rviz --> use e1 command topic

  ROS_INFO_STREAM(tool_frame_id << " Attached Successfully");
}


bool akit_pick_place::detachTool(std::string tool_frame_id){

  std::transform(tool_frame_id.begin(), tool_frame_id.end(), tool_frame_id.begin(), ::tolower);
  if (tool_frame_id != "gripper_rotator" && tool_frame_id != "bucket_raedlinger"){
    ROS_ERROR_STREAM("Unknown tool, please write correct tool name");
    return false;
    exit(1);
  }
  //variables
  double quickcoupler_x = 0.035; //distance between quickcoupler frame origin and edge in x-direction
  double distance_above_gripper = 0.25; //25 cm above gripper

  //allow collision between tool group and quickcoupler --> eef parent link
  this->allowToolCollision(tool_frame_id);

  //de-activate lock --> no control in rviz --> use e1 command topic

  //update start state to current state
  akitState = akitGroup->getCurrentState();
  akitGroup->setStartState(*akitState);
  akitState->copyJointGroupPositions(akitJointModelGroup,akitJointPositions);

  //rotate quickcoupler -90deg
  akitJointPositions[4] -= M_PI/2;
  akitGroup->setJointValueTarget(akitJointPositions);
  akitSuccess = (akitGroup->plan(MotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (akitSuccess){
    akitGroup->execute(MotionPlan);
    ROS_INFO_STREAM("Rotation of quickcoupler --> success");
  } else {
    ROS_ERROR_STREAM("Failed to rotate quickcoupler");
    return false;
    exit(1);
  }

  //execute cartesian motion in +x axis direction
  this->executeAxisCartesianMotion(UP, distance_above_gripper + quickcoupler_x, 'x');

   ROS_INFO_STREAM(tool_frame_id << " Detached Successfully");
}
